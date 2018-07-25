from Commands import MotorController
import time

a = MotorController(2)
a.update(loop=True)
while True:
    time.sleep(1)
