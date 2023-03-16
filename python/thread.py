import threading
import time

class MyThread (threading.Thread):
	die = False
	a = "a"
	def __init__(self, name):
		threading.Thread.__init__(self)
		self.name = name

	def run (self):
		i =1
		while i<10:
			time.sleep(1)
			print(self.a)
			i +=1
	def join(self):
		self.die = True
		super().join()



test_thread = MyThread("test")
test_thread.a = "P"
test_thread.start()
time.sleep(3)
print("done")
test_thread.join()
print("joined")
