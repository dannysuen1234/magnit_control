#!/usr/bin/env python
# SPDX-License-Identifier: LGPL-2.1-or-later

###################################################################
#
# This is a unified test sample for BT Mesh
#
# To run the test:
#     test-mesh [token]
#
#            'token' is an optional argument. It must be a 16-digit
#            hexadecimal number. The token must be associated with
#            an existing node. The token is generated and assigned
#            to a node as a result of successful provisioning (see
#            explanation of "join" option).
#            When the token is set, the menu operations "attach"
#            and "remove" may be performed on a node specified
#            by this token.
#
#      The test imitates a device with 2 elements:
#            element 0: OnOff Server model
#                       Sample Vendor model
#            element 1: OnOff Client model
#
# The main menu:
#       token
#       join
#       attach
#       remove
#       dest
#       uuid
#       app-index
#       client-menu
#       exit
#
# The main menu options explained:
#     token
#            Set the unique node token.
#            The token can be set from command line arguments as
#            well.
#
#     join
#            Request provisioning of a device to become a node
#            on a mesh network. The test generates device UUID
#            which is displayed and will need to be provided to
#            an outside entity that acts as a Provisioner. Also,
#            during the provisioning process, an agent that is
#            part of the test, will request (or will be requested)
#            to perform a specified operation, e.g., a number will
#            be displayed and this number will need to be  entered
#            on the Provisioner's side.
#            In case of successful provisioning, the application
#            automatically attaches as a node to the daemon. A node
#            'token' is returned to the application and is used
#            for the runtime of the test.
#
#     attach
#            Attach the application to bluetoothd-daemon as a node.
#            For the call to be successful, the valid node token must
#            be already set, either from command arguments or by
#            executing "set token" operation or automatically after
#            successfully executing "join" operation in the same
#            test run.
#
#     remove
#           Permanently removes any node configuration from daemon
#           and persistent storage. After this operation, the node
#           is permanently forgotten by the daemon and the associated
#           node token is no longer valid.
#
#     dest
#           Set destination address to send messages: 4 hex digits
#
#     app-index
#           Set AppKey index to indicate which application key to use
#           to encode outgoing messages: up to 3 hex digits
#
#     vendor-send
#           Allows to send an arbitrary endor message.
#           The destination is set based on previously executed "dest"
#           command (if not set, the outbound message will fail).
#           User is prompted to enter hex bytearray payload.
#           The message is originated from the vendor model registered
#           on element 0. For the command to succeed, the AppKey index
#           that is set by executing "app-key" must correspond to the
#           application key to which the Sample Vendor model is bound.
#
#     client-menu
#           Enter On/Off client submenu.
#
#     quit
#           Exits the test.
#
###################################################################
from __future__ import print_function
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import struct
import fcntl
import os
import numpy
import random
import dbus
import dbus.service
import dbus.exceptions

from threading import Timer
import time
import uuid

try:
   from gi.repository import GLib
except ImportError:
   import glib as GLib

from dbus.mainloop.glib import DBusGMainLoop

try:
  from termcolor import colored, cprint
  set_error = lambda x: colored('!' + x, 'red', attrs=['bold'])
  set_cyan = lambda x: colored(x, 'cyan', attrs=['bold'])
  set_green = lambda x: colored(x, 'green', attrs=['bold'])
  set_yellow = lambda x: colored(x, 'yellow', attrs=['bold'])
except ImportError:
  print('!!! Install termcolor module for better experience !!!')
  set_error = lambda x: x
  set_cyan = lambda x: x
  set_green = lambda x: x
  set_yellow = lambda x: x

# Provisioning agent
try:
  import agent
except ImportError:
  agent = None

# ROS
#from bluetooth_mesh.srv import setToken
from bluetooth_mesh.msg import Message, Meshmsg
import rospy

MESH_SERVICE_NAME = 'org.bluez.mesh'
DBUS_PROP_IFACE = 'org.freedesktop.DBus.Properties'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'

MESH_MGR_IFACE = 'org.bluez.mesh.Management1'
MESH_NETWORK_IFACE = 'org.bluez.mesh.Network1'
MESH_NODE_IFACE = 'org.bluez.mesh.Node1'
MESH_APPLICATION_IFACE = 'org.bluez.mesh.Application1'
MESH_ELEMENT_IFACE = 'org.bluez.mesh.Element1'

#APP_COMPANY_ID = 0x05f1
#APP_COMPANY_ID = 0x0574
APP_COMPANY_ID = 0xa000
APP_PRODUCT_ID = 0x0001
APP_VERSION_ID = 0x0001

VENDOR_ID_NONE = 0xffff
#VENDOR_ID_NONE = 0xa000

TRANSACTION_TIMEOUT = 6

app = None
bus = None
mainloop = None
node = None
node_mgr = None
mesh_net = None

dst_addr = 0x0000
app_idx = 0

# Node token housekeeping
token = None
have_token = False
attached = False

# Remote device UUID
have_uuid = False
remote_uuid = None

# Menu housekeeping
MAIN_MENU = 0
ON_OFF_CLIENT_MENU = 1

INPUT_NONE = 0
INPUT_TOKEN = 1
INPUT_DEST_ADDRESS = 2
INPUT_APP_KEY_INDEX = 3
INPUT_MESSAGE_PAYLOAD = 4
INPUT_UUID = 5

menus = []
current_menu = None

user_input = 0
input_error = False

send_opts = dbus.Dictionary(signature='sv')
send_opts = {'ForceSegmented' : dbus.Boolean(True)}

def raise_error(str_value):
	global input_error

	input_error = True
	print(set_error(str_value))

def clear_error():
	global input_error
	input_error = False

def is_error():
	return input_error

def app_exit():
	global mainloop
	global app

	for el in app.elements:
		for model in el.models:
			if model.timer != None:
				model.timer.cancel()
	mainloop.quit()

def set_token(str_value):
	global token
	global have_token

	if len(str_value) != 16:
		raise_error('Expected 16 digits')
		return

	try:
		input_number = int(str_value, 16)
	except ValueError:
		raise_error('Not a valid hexadecimal number')
		return

	token = numpy.uint64(input_number)
	have_token = True

def set_uuid(str_value):
	global remote_uuid
	global have_uuid

	if len(str_value) != 32:
		raise_error('Expected 32 digits')
		return

	remote_uuid = bytearray.fromhex(str_value)
	have_uuid = True

def array_to_string(b_array):
	str_value = ""
	for b in b_array:
		if type(b) == int:
			str_value += "%02x" % b
		else:
			str_value += "%02x" % ord(b)		#python2
	return str_value

def generic_error_cb(error):
	print(set_error('D-Bus call failed: ') + str(error))

def generic_reply_cb():
	return

def attach_app_error_cb(error):
	print(set_error('Failed to register application: ') + str(error))

def attach(token):
	print('Attach mesh node to bluetooth-meshd daemon')
	print(app.get_path())
	mesh_net.Attach(app.get_path(), token,
					reply_handler=attach_app_cb,
					error_handler=attach_app_error_cb)

def join_cb():
	print('Join procedure started')

def join_error_cb(reason):
	print('Join procedure failed: ', reason)

def remove_node_cb():
	global attached
	global have_token

	print(set_yellow('Node removed'))
	attached = False
	have_token = False

def unwrap(item):
	if isinstance(item, dbus.Boolean):
		return bool(item)
	if isinstance(item, (dbus.UInt16, dbus.Int16, dbus.UInt32, dbus.Int32,
						dbus.UInt64, dbus.Int64)):
		return int(item)
	if isinstance(item, dbus.Byte):
		return bytes([int(item)])
	if isinstance(item, dbus.String):
			return item
	if isinstance(item, (dbus.Array, list, tuple)):
		return [unwrap(x) for x in item]
	if isinstance(item, (dbus.Dictionary, dict)):
		return dict([(unwrap(x), unwrap(y)) for x, y in item.items()])

	print(set_error('Dictionary item not handled: ') + type(item))

	return item

def attach_app_cb(node_path, dict_array):
	global attached

	attached = True

	print(set_yellow('Mesh app registered: ') + set_green(node_path))

	obj = bus.get_object(MESH_SERVICE_NAME, node_path)

	global node_mgr
	node_mgr = dbus.Interface(obj, MESH_MGR_IFACE)

	global node
	node = dbus.Interface(obj, MESH_NODE_IFACE)

	els = unwrap(dict_array)

	for el in els:
		#idx = struct.unpack('b', el[0])[0]
		idx = eval(el[0])[0]		#python2

		models = el[1]
		element = app.get_element(idx)
		element.set_model_config(models)

def interfaces_removed_cb(object_path, interfaces):
	print('Removed')
	if not mesh_net:
		return

	print(object_path)
	if object_path == mesh_net[2]:
		print('Service was removed')
		app_exit()

def print_state(state):
	print('State is ', end='')
	if state == 0:
		print('OFF')
	elif state == 1:
		print('ON')
	else:
		print('UNKNOWN')
class ModTimer():
	def __init__(self):
		self.seconds = None
		self.func = None
		self.thread = None
		self.busy = False

	def _timeout_cb(self):
		self.func()
		self.busy = True
		self._schedule_timer()
		self.busy =False

	def _schedule_timer(self):
		self.thread = Timer(self.seconds, self._timeout_cb)
		self.thread.start()

	def start(self, seconds, func):
		self.func = func
		self.seconds = seconds
		if not self.busy:
			self._schedule_timer()

	def cancel(self):
		if self.thread is not None:
			self.thread.cancel()
			self.thread = None

class Application(dbus.service.Object):

	def __init__(self, bus):
		self.path = '/example'
		self.agent = None
		self.elements = []
		dbus.service.Object.__init__(self, bus, self.path)

	def set_agent(self, agent):
		self.agent = agent

	def get_path(self):
		return dbus.ObjectPath(self.path)

	def add_element(self, element):
		self.elements.append(element)

	def get_element(self, idx):
		for ele in self.elements:
			if ele.get_index() == idx:
				return ele

	def get_properties(self):
		return {
			MESH_APPLICATION_IFACE: {
				'CompanyID': dbus.UInt16(APP_COMPANY_ID),
				'ProductID': dbus.UInt16(APP_PRODUCT_ID),
				'VersionID': dbus.UInt16(APP_VERSION_ID)
			}
		}

	@dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
	def GetManagedObjects(self):
		response = {}
		response[self.path] = self.get_properties()
		response[self.agent.get_path()] = self.agent.get_properties()
		for element in self.elements:
			response[element.get_path()] = element.get_properties()
		return response

	@dbus.service.method(MESH_APPLICATION_IFACE,
					in_signature="t", out_signature="")
	def JoinComplete(self, value):
		global token
		global have_token
		global attach

		print(set_yellow('Joined mesh network with token ') +
				set_green(format(value, '016x')))

		token = value
		have_token = True

	@dbus.service.method(MESH_APPLICATION_IFACE,
					in_signature="s", out_signature="")
	def JoinFailed(self, value):
		print(set_error('JoinFailed '), value)


class Element(dbus.service.Object):
	PATH_BASE = '/example/ele'

	def __init__(self, bus, index):
		self.path = self.PATH_BASE + format(index, '02x')
		self.models = []
		self.bus = bus
		self.index = index
		dbus.service.Object.__init__(self, bus, self.path)

	def _get_sig_models(self):
		mods = []
		for model in self.models:
			opts = []
			id = model.get_id()
			vendor = model.get_vendor()
			if vendor == VENDOR_ID_NONE:
				mod = (id, opts)
				mods.append(mod)
		return mods

	def _get_v_models(self):
		mods = []
		for model in self.models:
			opts = []
			id = model.get_id()
			v = model.get_vendor()
			if v != VENDOR_ID_NONE:
				mod = (v, id, opts)
				mods.append(mod)
		return mods

	def get_properties(self):
		vendor_models = self._get_v_models()
		sig_models = self._get_sig_models()

		props = {'Index' : dbus.Byte(self.index)}
		props['Models'] = dbus.Array(sig_models, signature='(qa{sv})')
		props['VendorModels'] = dbus.Array(vendor_models,
							signature='(qqa{sv})')
		#print(props)
		return { MESH_ELEMENT_IFACE: props }

	def add_model(self, model):
		model.set_path(self.path)
		self.models.append(model)

	def get_index(self):
		return self.index

	def set_model_config(self, configs):
		for config in configs:
			mod_id = config[0]
			self.update_model_config(mod_id, config[1])

	@dbus.service.method(MESH_ELEMENT_IFACE,
					in_signature="qqvay", out_signature="")
	def MessageReceived(self, source, key, dest, data):
		print(('Message Received on Element %02x') % self.index, end='')
		print(', src=', format(source, '04x'), end='')

		if isinstance(dest, int):
			print(', dst=%04x' % dest)
		elif isinstance(dest, dbus.Array):
			dst_str = array_to_string(dest)
			print(', dst=' + dst_str)

		for databyte in data:
			print(('%02x ') % databyte, end='')
		print('data received')

		for model in self.models:
			model.process_message(source, dest, key, data)

	@dbus.service.method(MESH_ELEMENT_IFACE,
					in_signature="qa{sv}", out_signature="")

	def UpdateModelConfiguration(self, model_id, config):
		cfg = unwrap(config)
		print(cfg)
		self.update_model_config(model_id, cfg)

	def update_model_config(self, model_id, config):
		print(('Update Model Config '), end='')
		print(format(model_id, '04x'))
		for model in self.models:
			if model_id == model.get_id():
				model.set_config(config)
				return

	@dbus.service.method(MESH_ELEMENT_IFACE,
					in_signature="", out_signature="")

	def get_path(self):
		return dbus.ObjectPath(self.path)

class Model():
	def __init__(self, model_id):
		self.cmd_ops = []
		self.model_id = model_id
		self.vendor = VENDOR_ID_NONE
		self.bindings = []
		self.pub_period = 0
		self.pub_id = 0
		self.path = None
		self.timer = None

	def set_path(self, path):
		self.path = path

	def get_id(self):
		return self.model_id

	def get_vendor(self):
		return self.vendor

	def process_message(self, source, dest, key, data):
		return

	def set_publication(self, period):
		self.pub_period = period

	def send_publication(self, data):
		pub_opts = dbus.Dictionary(signature='sv')

		print('Send publication ', end='')
		print(data)
		node.Publish(self.path, self.model_id, pub_opts, data,
						reply_handler=generic_reply_cb,
						error_handler=generic_error_cb)

	def send_message(self, dest, key, data):
		global send_opts
		for databyte in data:
			if type(databyte) == int:
				print(('%02x ') % databyte, end='')
			else:
				print(('%02x ') % ord(databyte), end='')	#python2
		print('data send to dest %04x' % dest)

		node.Send(self.path, dest, key, send_opts, data,
						reply_handler=generic_reply_cb,
						error_handler=generic_error_cb)

	def set_config(self, config):
		if 'Bindings' in config:
			self.bindings = config.get('Bindings')
			print('Bindings: ', end='')
			print(self.bindings)
		if 'PublicationPeriod' in config:
			self.set_publication(config.get('PublicationPeriod'))
			print('Model publication period ', end='')
			print(self.pub_period, end='')
			print(' ms')
		if 'Subscriptions' in config:
			print('Model subscriptions ', end='')
			self.print_subscriptions(config.get('Subscriptions'))
			print()

	def print_subscriptions(self, subscriptions):
		for sub in subscriptions:
			if isinstance(sub, int):
				print('%04x,' % sub, end=' ')

			if isinstance(sub, list):
				label = uuid.UUID(bytes=b''.join(sub))
				print(label, ',', end=' ')

########################
# On Off Server Model
########################
class OnOffServer(Model):
	def __init__(self, model_id):
		Model.__init__(self, model_id)
		self.tid = None
		self.last_src = 0x0000
		self.last_dst = 0x0000
		self.cmd_ops = { 0x8201,  # get
				 0x8202,  # set
				 0x8203,  # set unacknowledged
				 0x8204 } # status

		print("OnOff Server ")
		self.state = 0
		print_state(self.state)
		self.pub_timer = ModTimer()
		self.t_timer = ModTimer()

	def process_message(self, source, dest, key, data):
		datalen = len(data)

		# data has 0x00 0x00 at the end
		# so len is not 4
		if datalen > 4:
			datamsg = data[0:4]
			datalen = 4
		else:
			datamsg = data

		if datalen != 2 and datalen != 4:
			# The opcode is not recognized by this model
			return

		if datalen == 2:
			#op_tuple=struct.unpack('>H',bytes(data))
			#python2
			#op_tuple=struct.unpack('>H',bytes(datamsg))
			#opcode = op_tuple[0]
			opcode = (datamsg[0]*16**2) + (datamsg[1])

			if opcode != 0x8201:
				# The opcode is not recognized by this model
				return
			print('Get state')
		elif datalen == 4:
			#opcode,self.state, tid = struct.unpack('>HBB', bytes(data))
			#python2
			#opcode,self.state, tid = struct.unpack('>HBB', bytes(datamsg))
			opcode = (datamsg[0]*16**2) + (datamsg[1])
			self.state = datamsg[2]
			tid = datamsg[3]

			if opcode != 0x8202 and opcode != 0x8203:
				# The opcode is not recognized by this model
				return
			print_state(self.state)

			if (self.tid != None and self.tid == tid and
						self.last_src == source and
						self.last_dst == dest):
				# Ignore duplicate transaction
				return

			self.t_timer.cancel()
			self.tid = tid
			self.last_src = source
			self.last_dst = dest
			self.t_timer.start(TRANSACTION_TIMEOUT, self.t_track)

			# Unacknowledged "set"
			if opcode == 0x8203:
				return

		rsp_data = struct.pack('>HB', 0x8204, self.state)
		self.send_message(source, key, rsp_data)

	def t_track(self):
			self.t_timer.cancel()
			self.tid = None
			self.last_src = 0x0000
			self.last_dst = 0x0000

	def set_publication(self, period):

		self.pub_period = period
		if period == 0:
			self.pub_timer.cancel()
			return

		# We do not handle ms in this example
		if period < 1000:
			return

		self.pub_timer.start(period/1000, self.publish)

	def publish(self):
		print('Publish')
		data = struct.pack('>HB', 0x8204, self.state)
		self.send_publication(data)

########################
# On Off Client Model
########################
class OnOffClient(Model):
	def __init__(self, model_id):
		Model.__init__(self, model_id)
		self.tid = 0
		self.data = None
		self.cmd_ops = { 0x8201,  # get
				 0x8202,  # set
				 0x8203,  # set unacknowledged
				 0x8204 } # status
		print('OnOff Client')

	def _send_message(self, dest, key, data):
		print('OnOffClient send command')
		self.send_message(dest, key, data)

	def get_state(self, dest, key):
		opcode = 0x8201
		self.data = struct.pack('>H', opcode)
		self._send_message(dest, key, self.data)

	def set_state(self, dest, key, state):
		if (dst_addr <= 0x7fff):
			opcode = 0x8202
		else:
			opcode = 0x8203
		print('Set state:', state)
		self.data = struct.pack('>HBB', opcode, state, self.tid)
		self.tid = (self.tid + 1) % 255
		self._send_message(dest, key, self.data)

	def repeat(self, dest, key):
		if self.data != None:
			self._send_message(dest, key, self.data)
		else:
			print('No previous command stored')

	def process_message(self, source, dest, key, data):
		print('OnOffClient process message len = ', end = '')
		datalen = len(data)
		print(datalen)

		if datalen != 3:
			# The opcode is not recognized by this model
			return

		opcode, state = struct.unpack('>HB',bytes(data))

		if opcode != 0x8204 :
			# The opcode is not recognized by this model
			return

		print(set_yellow('Got state '), end = '')

		state_str = "ON"
		if state == 0:
			state_str = "OFF"

		print(set_green(state_str), set_yellow('from'),
						set_green('%04x' % source))

########################
# Sample Vendor Model
########################
class SampleVendor(Model):
	def __init__(self, model_id):
		Model.__init__(self, model_id)
		self.vendor = 0xa000
		#self.vendor = 0x0574
		#self.vendor = 0x05F1 # Linux Foundation Company ID
		#self.vendor = 0xFFFF

	def process_message(self, source, dest, key, data):
		print('vendorClient process message len = ', end = '')
		datalen = len(data)
		print(datalen)
		
		datamessage = data
		#while len(datamessage) < 4:
		#	datamessage.append(0)

		# print('opcode=%02x' % (datamessage[0]), end = '')
		# if len(datamessage) >= 3:	
		# 	print(' ,cid=%02x' % (datamessage[2]), end = '')
		# 	print('%02x' % (datamessage[1]), end = '')
		# 	print(' data=%02x' % (datamessage[3]))
		
		datamsg = []
		for x in range(datalen):
			datamsg.append(datamessage[x])
		# datamsg = datamessage[1:]
		tempmsg = Message()
		tempmsg.msg.dest = numpy.uint16(dest)
		tempmsg.msg.source = numpy.uint16(source)
		# tempmsg.msg.opcode = numpy.uint8(datamessage[0])
		#tempTuple.data = numpy.asarray(data[3:],dtype=numpy.uint8)
		tempmsg.msg.data = datamsg

		rospub.publish(tempmsg)
		#rospub.publish(dest, source, data[0], data[3:])
		

		#opcode, cid, state = struct.unpack('B>HB',bytes(data)
		#print("opcode={0:02x} cid={1:04x} data={2:02x}".format(opcode, cid, state))

		# if datalen != 3:
		# 	# The opcode is not recognized by this model
		# 	return

		# opcode, state = struct.unpack('>HB',bytes(data))

		# if opcode != 0x8204 :
		# 	# The opcode is not recognized by this model
		# 	return

		# print(set_yellow('Got state '), end = '')

		# state_str = "ON"
		# if state == 0:
		# 	state_str = "OFF"

		# print(set_green(state_str), set_yellow('from'),
		# 				set_green('%04x' % source))
						
########################
# Menu functions
########################
class MenuItem():
	def __init__(self, desc, func):
		self.desc = desc
		self.func = func

class Menu():
	def __init__(self, title, menu):
		self.title = title
		self.menu = menu

	def show(self):
		print(set_cyan('*** ' + self.title.upper() + ' ***'))
		for k, v in self.menu.items():
			print(set_green(k), set_cyan(v.desc))

	def process_cmd(self, str_value):
		if is_error():
			self.show()
			clear_error()
			return

		cmds = []
		for key in self.menu.keys():
			if key.startswith(str_value):
				cmds.append(key)

		if len(cmds) == 0:
			print(set_error('Unknown menu option: '), str_value)
			self.show()
			return
		if len(cmds) > 1:
			for cmd in cmds:
			     print(set_cyan(cmd + '?'))
			return

		self.menu.get(cmds[0]).func()

class MenuHandler(object):
	def __init__(self, callback):
		self.cb = callback
		flags = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
		flags |= os.O_NONBLOCK
		fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, flags)
		sys.stdin.flush()
		GLib.io_add_watch(sys.stdin, GLib.IO_IN, self.input_callback)

	def input_callback(self, fd, condition):
		chunk = fd.read()
		buffer = ''
		for char in chunk:
			buffer += char
			if char == '\n':
				self.cb(buffer)

		return True

def process_input(input_str):
	str_value = input_str.strip()

	# Allow entering empty lines for better output visibility
	if len(str_value) == 0:
		return

	current_menu.process_cmd(str_value)

def switch_menu(level):
	global current_menu

	if level >= len(menus):
		return

	current_menu = menus[level]
	current_menu.show()

########################
# Main menu class
########################
class MainMenu(Menu):
	def __init__(self):
		menu_items = {
			'token': MenuItem(' - set node ID (token)',
						self.__cmd_set_token),
			'join': MenuItem(' - join mesh network',
						self.__cmd_join),
			'attach': MenuItem(' - attach mesh node',
						self.__cmd_attach),
			'remove': MenuItem(' - delete node',
						self.__cmd_remove),
			'dest': MenuItem(' - set destination address',
						self.__cmd_set_dest),
			'uuid': MenuItem(' - set remote uuid',
						self.__cmd_set_uuid),
			'app-index': MenuItem(' - set AppKey index',
						self.__cmd_set_app_idx),
			'vendor-send': MenuItem(' - send raw vendor message',
						self.__cmd_vendor_msg),
			'client-menu': MenuItem(' - On/Off client menu',
						self.__cmd_client_menu),
			'quit': MenuItem(' - exit the test', app_exit)
		}

		Menu.__init__(self, 'Main Menu', menu_items)

	def __cmd_client_menu(self):
		if attached != True:
			print(set_error('Disallowed: node is not attached'))
			return
		switch_menu(ON_OFF_CLIENT_MENU)

	def __cmd_set_token(self):
		global user_input

		if have_token == True:
			print('Token already set')
			return

		user_input = INPUT_TOKEN
		print(set_cyan('Enter 16-digit hex node ID:'))

	def __cmd_set_dest(self):
		global user_input

		user_input = INPUT_DEST_ADDRESS
		print(set_cyan('Enter 4-digit hex destination address:'))

	def __cmd_set_uuid(self):
		global user_input

		user_input = INPUT_UUID
		print(set_cyan('Enter 32-digit hex remote UUID:'))

	def __cmd_set_app_idx(self):
		global user_input

		user_input = INPUT_APP_KEY_INDEX;
		print(set_cyan('Enter app key index (up to 3 digit hex):'))

	def __cmd_vendor_msg(self):
		global user_input

		user_input = INPUT_MESSAGE_PAYLOAD;
		print(set_cyan('Enter message payload (hex):'))

	def __cmd_join(self):
		if agent == None:
			print(set_error('Provisioning agent not found'))
			return

		uuid_bytes = uuid.uuid4().bytes
		uuid_str = array_to_string(uuid_bytes)

		print(set_yellow('Joining with UUID ') + set_green(uuid_str))
		#mesh_net.Join(app.get_path(), uuid_bytes,
		mesh_net.Join(app.get_path(), uuid_bytes,
			reply_handler=join_cb,
			error_handler=join_error_cb)

	def __cmd_attach(self):
		if have_token == False:
			print(set_error('Token is not set'))
			self.show()
			return

		attach(token)

	def __cmd_remove(self):
		if have_token == False:
			print(set_error('Token is not set'))
			self.show()
			return

		print('Removing mesh node')
		mesh_net.Leave(token, reply_handler=remove_node_cb,
					error_handler=generic_error_cb)

	def __send_vendor_msg(self, str_value):
		try:
			msg_data = bytearray.fromhex(str_value)
		except ValueError:
			raise_error('Not a valid hexadecimal input')
			return

		print(set_yellow('Send data: ' + set_green(str_value)))
		# Modle 3001 is Server
		# Model 3002 is Client
		# Use 3002 to send vendor message
		app.elements[1].models[1].send_message(dst_addr, app_idx,
							msg_data)

	def process_cmd(self, str_value):
		global user_input
		global dst_addr
		global app_idx

		if user_input == INPUT_TOKEN:
			set_token(str_value)
		elif user_input == INPUT_UUID:
			set_uuid(str_value)
		elif user_input == INPUT_DEST_ADDRESS:
			res = set_value(str_value, 4, 4)
			if is_error() != True:
				dst_addr = res
				print(set_yellow("Destination address: ") +
					set_green(format(dst_addr, '04x')))
		elif user_input == INPUT_APP_KEY_INDEX:
			res = set_value(str_value, 1, 3)
			if is_error() != True:
				app_idx = res
				print(set_yellow("Application index: ") +
					set_green(format(app_idx, '03x')))
		elif  user_input == INPUT_MESSAGE_PAYLOAD:
			self.__send_vendor_msg(str_value)

		if user_input != INPUT_NONE:
			user_input = INPUT_NONE
			if is_error() != True:
				return

		Menu.process_cmd(self, str_value)

##############################
# On/Off Client menu class
##############################
class ClientMenu(Menu):
	def __init__(self):
		menu_items = {
			'get-state': MenuItem(' - get server state',
						self.__cmd_get_state),
			'off': MenuItem(' - set state OFF',
						self.__cmd_set_state_off),
			'on': MenuItem(' - set state ON',
						self.__cmd_set_state_on),
			'repeat': MenuItem(' - repeat last command',
						self.__cmd_repeat_transaction),
			'back': MenuItem(' - back to main menu',
						self.__cmd_main_menu),
			'quit': MenuItem(' - exit the test', app_exit)
		}

		Menu.__init__(self, 'On/Off Client Menu', menu_items)

	def __cmd_main_menu(self):
		switch_menu(MAIN_MENU)

	def __cmd_get_state(self):
		app.elements[1].models[0].get_state(dst_addr, app_idx)

	def __cmd_set_state_off(self):
		app.elements[1].models[0].set_state(dst_addr, app_idx, 0)

	def __cmd_set_state_on(self):
		app.elements[1].models[0].set_state(dst_addr, app_idx, 1)

	def __cmd_repeat_transaction(self):
		app.elements[1].models[0].repeat(dst_addr, app_idx)

def set_value(str_value, min, max):

	if len(str_value) > max or len(str_value) < min:
		raise_error('Bad input length %d' % len(str_value))
		return -1

	try:
		value = int(str_value, 16)
	except ValueError:
		raise_error('Not a valid hexadecimal number')
		return -1

	return value

# set token and attach
# def ros_set_token(req):
# 	__cmd_remove()
# 	set_token(req.data)
# 	attach(token)
# 	return True

def vendor_send_callback(message):
	# on off client
	#get 82 01
	#set 82 02 on/off
	# if message.msg.opcode == 0x82:
	# 	if message.msg.data[0] == '\x01':
	# 		app.elements[1].models[0].get_state(message.msg.dest, app_idx)
	# 	elif message.msg.data[0] == '\x02':
	# 		data = struct.unpack("b",message.msg.data[1])[0]
	# 		app.elements[1].models[0].set_state(message.msg.dest, app_idx, data)
	if message.msg.data[0] == '\x82':
		if message.msg.data[1] == '\x01':
			app.elements[1].models[0].get_state(message.msg.dest, app_idx)
		elif message.msg.data[1] == '\x02':
			data = struct.unpack("b",message.msg.data[2])[0]
			app.elements[1].models[0].set_state(message.msg.dest, app_idx, data)

	#vendor
	#Cx xx xx
	else:
		app.elements[1].models[1].send_message(message.msg.dest, app_idx, message.msg.data)

########################
# Main entry
########################
def main():

	DBusGMainLoop(set_as_default=True)

	global bus
	bus = dbus.SystemBus()
	global mainloop
	global app
	global mesh_net
	global menu
	global current_menu
	global rospub
	global rossub

	#if len(sys.argv) > 1 :
	#	set_token(sys.argv[1])
	# set_token('597f5958703a5f07')

	mesh_net = dbus.Interface(bus.get_object(MESH_SERVICE_NAME,
						"/org/bluez/mesh"),
						MESH_NETWORK_IFACE)

	mesh_net.connect_to_signal('InterfacesRemoved', interfaces_removed_cb)

	app = Application(bus)

	# Provisioning agent
	if agent != None:
		app.set_agent(agent.Agent(bus))

	first_ele = Element(bus, 0x00)
	second_ele = Element(bus, 0x01)

	print(set_yellow('Register OnOff Server model on element 0'))
	first_ele.add_model(OnOffServer(0x1000))

	print(set_yellow('Register Vendor model on element 0'))
	#first_ele.add_model(SampleVendor(0x0001))
	first_ele.add_model(SampleVendor(0x3001))

	print(set_yellow('Register OnOff Client model on element 1'))
	second_ele.add_model(OnOffClient(0x1001))

	print(set_yellow('Register Vendor model on element 0'))
	#first_ele.add_model(SampleVendor(0x0001))
	second_ele.add_model(SampleVendor(0x3002))

	app.add_element(first_ele)
	app.add_element(second_ele)

	#set_token('597f5958703a5f07')
	#attach(token)

	#s = rospy.Service('/bluetooth_mesh/token', setToken, ros_set_token)
	rospub = rospy.Publisher('bluetooth_mesh/receive', Message ,queue_size = 10)
	rospy.init_node('bluetooth_mesh', anonymous=True)
	rospy.Subscriber('bluetooth_mesh/send', Message, vendor_send_callback)
	#rate = rospy.Rate(10)
	print("Ready ros setup")

	set_token(rospy.get_param('/bluetooth_mesh/token'))
	attach(token)

	#rospy.spin()

	mainloop = GLib.MainLoop()

	#menus.append(MainMenu())
	#menus.append(ClientMenu())
	#switch_menu(MAIN_MENU)

	#event_catcher = MenuHandler(process_input)
	mainloop.run()

if __name__ == '__main__':
	main()
