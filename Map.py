import random
import math

class Rides:

    def __init__(self, N, start_time='8:00', service_time='6:00'):
        self.N_riders = N
        self.StartDay(start_time, service_time)
        self.pickup_time = self.pickup_time()

    def StartDay(self, s, st):
        self.starttime = 0
        self.servicetime = 0
        if not type(s) and type(st) is str:
            raise TypeError("Only string in 'HH::MM' format allowed")
        try:
            factor = [60, 1, 1 / 60]
            srtspt = s.split(':')
            setspt = st.split(':')
        except TypeError:
            raise TypeError("Only string in 'HH::MM' format allowed")
        except AttributeError:
            raise AttributeError("Only string in 'HH::MM' format allowed")
        for i, j, k in zip(factor, srtspt, setspt):
            self.starttime += i * int(j)
            self.servicetime += i * int(k)

    def pickup(self):
        return list(range(1, self.N_riders + 1))

    def dropoff(self):
        return list(range(self.N_riders + 1, 2 * self.N_riders + 1))

    def depot(self):
        return [0, 2 * self.N_riders + 1]

    def trips(self):
        return self.pickup() + self.dropoff()

    def nodes(self):
        return [0] + self.trips() + [2 * self.N_riders + 1]

    def pickup_time(self):
        p = {i: self.starttime + self.servicetime * random.random() for i in self.pickup()}
        return p

    def load(self):
        in_bus = {i: 1 for i in self.pickup()}
        out_bus = {i: -1 for i in self.dropoff()}
        in_bus.update(out_bus)
        in_bus[0] = 0
        in_bus[2 * self.N_riders + 1] = 0
        return in_bus


class Map(Rides):

    def __init__(self, N, start_time='8:00', service_time='6:00'):
        super().__init__(N, start_time, service_time)
        self.node = self.coordinates()
        self.distance = self.distance()
        self.run = self.time()
        self.dropoff_time = self.dropoff_time()

    def _calculateDistance(self,x,y):
        x1,y1 = x
        x2,y2 = y
        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return round(dist)

    def coordinates(self):
        n = {i: (random.random()*10, random.random()*10) for i in self.nodes()}
        n[self.depot()[1]] = n[0]
        return n

    def distance(self):
        dist = {(i, j): self._calculateDistance(self.node[i],self.node[j])\
                            for i in self.nodes() for j in self.nodes()}
        return dist

    def time(self):
        run = {(i, j): self.distance[i, j]*3 for i in self.nodes() for j in self.nodes()}
        return run

    def dropoff_time(self):
        p = self.pickup_time
        d = {i: p[i - self.N_riders] + self.run[i - self.N_riders, i] * 1.5 for i in self.dropoff()}
        return d