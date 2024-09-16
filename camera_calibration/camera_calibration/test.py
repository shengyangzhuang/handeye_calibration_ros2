from pyk4a import PyK4A

k4a = PyK4A()
k4a.start()
print(k4a.get_capture())
k4a.stop()
