import random as rdm
from queue import PriorityQueue
from LinkedList import *


class Tabu():
    """Implements a tabu search heuristic for DARP problem"""

    def __init__(self, model):
        self.model = model
        self.bus = list(range(self.model.parameters.bus))
        self.N = self.model.parameters.rides
        self.pickup = self.model.parameters.pickup
        self.tabudict = {}
        self.best = None
        self.bestcandidate = None
        self.pickup_time = self.model.parameters.pickup_time
        self.early = {i: self.pickup_time[i] - 10 for i in self.pickup_time.keys()}
        self.late = {i: self.pickup_time[i] + 10 for i in self.pickup_time.keys()}
        self.early.update({0: 480, self.model.last: 1440})
        self.late.update({0: 480, self.model.last: 1440})
        self.time = self.model.parameters.time
        self.ridetime = self.model.parameters.ridetime
        self.routetime = self.model.mapObject.servicetime + 480

    def tabuheuristic(self):
        self.initialSolution()
        iter = 0
        while iter != self.tabuiter:
            for i, k in self.tabudict.keys():
                decr = self.tabudict[i, k] - 1
                self.tabudict[i, k] = max(0, decr)

    def initialSolution(self):
        solution = {i: LinkedList() for i in self.bus}
        for i in self.bus:
            solution[i].append(Node(0))
        rdm.shuffle(self.pickup)
        for i in self.pickup:
            j = rdm.randint(0, len(self.bus) - 1)
            solution[j].append(Node(i))
            solution[j].append(Node(i + self.N))
        [solution[k].append(Node(self.model.last)) for k in self.bus]
        return solution

    def neighborhoodGen(self):
        sol = self.bestcandidate
        neighborhood = []
        for k in self.bus:
            availbus = [i for i in self.bus if i != k]
            rdm.shuffle(availbus)
            for i in self.bestcandidate[k]:
                if i in self.pickup:
                    ngbr = {k: sol[k].copy() for k in sol.keys()}
                    self.tabudict.update({(i, k): self.tabu_status})
                    for b in availbus:
                        try:
                            if self.tabudict[i, b] == 0:
                                node = ngbr[k][i]
                                ngbr[k].remove(i)
                                node.bus = b
                                for n in ngbr[b]:
                                    n = ngbr[b][n]
                                    if n.next.time >= node.time or \
                                            n.next.key == self.model.last:
                                        temp = n.next
                                        n.next = node
                                        node.next = temp
                                        if node.key != i + self.N:
                                            node = ngbr[k][i + self.N]
                                            node.bus = b
                                        else:
                                            break
                                break
                        except KeyError:
                            self.tabudict[i, b] = 0
                            node = ngbr[k][i]
                            ngbr[k].remove(i)
                            node.bus = b
                            for n in ngbr[b]:
                                n = ngbr[b][n]
                                if n.next.time >= node.time or \
                                        n.next.key == self.model.last:
                                    temp = n.next
                                    n.next = node
                                    node.next = temp
                                    if node.key != i + self.N:
                                        node = ngbr[k][i + self.N]
                                        ngbr[k].remove(i + self.N)
                                        node.bus = b
                                    else:
                                        break
                            break
                    neighborhood.append(ngbr)
        return neighborhood

    def solutionEval(self, solution):
        """Evaluating a solution"""

        def compute(s, k):
            prev = s[0]
            d = schedule[k][prev][3]
            lstload = schedule[k][prev][4]
            for i in s[1:]:
                a = (d + self.time[prev, i])
                b = max(early[i], a)
                schedule[k].update({i:
                                        [a, b, (b - a), (b + srvc), lstload + self.model.parameters.load[i]]
                                    })
                d = b + srvc
                lstload += self.model.parameters.load[i]
                prev = i

        def slack(s, k):
            st = 0
            if type(s) == list:
                if s[st] != 0:
                    f = PriorityQueue()
                    f.put(min(self.late[s[st]] - schedule[k][s[st]][1],
                              self.ridetime - (schedule[k][s[st]][3] -
                                               schedule[k][s[st] + self.N][1])))
                for i in range(st + 1, len(s)):
                    w = 0
                    for j in range(st + 1, i + 1):
                        if s[j] != self.model.last:
                            w += schedule[k][s[j]][2]
                        if s[j] == self.model.last:
                            W = w
                    w += min(self.late[s[i]] - schedule[k][s[i]][1],
                             self.ridetime - (schedule[k][s[i]][3] -
                                              schedule[k][s[i] + self.N][1]))
                    f.put(w)
                return min(f.get(), W)
            else:
                raise TypeError('Slack cannot be computed for %g' % str(type(s)))

        srvc = self.model.parameters.service_time
        start = self.model.mapObject.starttime
        sol = {k: solution[k].list() for k in self.bus}
        schedule = {k: {0:
                            [0,  # Arrival-time
                             0,  # Start of service
                             0,  # Waiting before service
                             start,  # Departure time
                             0]  # Load on vehicle
                        } for k in self.bus}
        [compute(sol[k], k) for k in self.bus]
        F = {k: slack(sol[k], k) for k in self.bus}
        for k in self.bus:
            schedule[k][0][3] += F[k]
        [compute(sol[k], k) for k in self.bus]
        for k in sol.keys():
            for i in range(len(sol[k])):
                if sol[k][i] in self.pickup:
                    F = slack(sol[k][i:], k)
                    schedule[k][sol[k][i]][1] += F
                    schedule[k][sol[k][i]][2] = schedule[k][sol[k][i]][1] + srvc
                    compute(sol[k][i:], k)
        self.calculateObjective(sol, schedule)

    def calculateObjective(self, sol, schedule):
        cost = 0  # Distance cost of the route
        for k in sol.keys():
            for i in sol:
                # Load constraint of the route
                load = max(0, schedule[k][i] - self.model.parameters.capacity)
                # Time duration of entire trip
                dur = max(0, schedule[k][self.model.last][0] - schedule[k][0][3] - self.routetime)
                # Time window constraint
                tmwndw = max(0, schedule[k][i][1] - self.late[i])
                # Customer ride-time constraint
                ridetm = max(0, (schedule[k][i][3] - schedule[k][i + self.N][1])
                             - self.ridetime)
                sol[k][i].load = schedule[k][i][4]
                sol[k][i].value = PriorityQueue()
                [sol[k][i].value.put(i) for i in ((load, 'L'), (tmwndw, 'T'), (ridetm, 'R'))]
                cost += self.model.parameters.distance[sol[k][i - 1], sol[k][i]] \
                        + weight['alpha'] * load \
                        + weight['beta'] * dur \
                        + weight['gamma'] * tmwndw \
                        + weight['delta'] * ridetm
