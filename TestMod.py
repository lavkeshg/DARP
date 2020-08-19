from Tests.Models import *
from threading import Thread

def something():
    print('Hello')


a = Thread(target=darp)
b = Thread(target=tabu)
c = Thread(target=lShaped)

b.start()
a.start()
c.start()
