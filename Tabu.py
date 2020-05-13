import random as rdm
from queue import PriorityQueue
from LinkedList import *


class Tabu():
    """Implements a tabu search heuristic for DARP problem"""

    def __init__(self, model, tabu_iterations=200, tabu_status=20):
        self.model = model
        self.bus = list(range(self.model.parameters.bus))
        self.N = self.model.parameters.rides
        self.pickup = self.model.parameters.pickup
        self.tabudict = {}
        self.tabuiter = tabu_iterations
        self.tabu_status = tabu_status
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
        self._init_weights()

    def _init_weights(self):
        self.weight = {
            'alpha': 1,
            'beta': 1,
            'gamma': 1,
            'delta': 1
        }

    def tabuheuristic(self):
        self.bestcandidate = self.initialSolution()
        self.solutionEval(self.bestcandidate)
        self.best = self.bestcandidate
        iter = 0
        while iter != self.tabuiter:
            for i, k in self.tabudict.keys():
                decr = self.tabudict[i, k] - 1
                self.tabudict[i, k] = max(0, decr)
            ngbr = self.neighborhoodGen()
            print('Neighborhood length: %d (iteration %d)' % (len(ngbr), iter))
            [self.solutionEval(n) for n in ngbr]
            for candidate in ngbr:
                if candidate[-1] < self.bestcandidate[-1]:
                    self.bestcandidate = candidate
            if self.bestcandidate[-1] < self.best[-1]:
                self.best = self.bestcandidate
                for k in self.bus:
                    for i in self.best[k]:
                        if i in self.pickup:
                            self.tabudict[i, k] = 0
                # for k in self.optmiter
                #     self.routeoptimization(self.best)
            iter += 1
        return self.best

    def initialSolution(self):
        solution = {i: LinkedList() for i in self.bus}
        for i in self.bus:
            solution[i].append(Node(0, bus=i))
        rdm.shuffle(self.pickup)
        for i in self.pickup:
            j = rdm.randint(0, len(self.bus) - 1)
            solution[j].append(Node(i))
            solution[j].append(Node(i + self.N))
        [solution[k].append(Node(self.model.last, bus=k)) for k in self.bus]
        return solution

    def neighborhoodGen(self):
        sol = self.bestcandidate
        neighborhood = []
        for k in self.bus:
            availbus = [i for i in self.bus if i != k]
            rdm.shuffle(availbus)
            for i in self.bestcandidate[k]:
                if i in self.pickup:
                    ngbr = {k: sol[k].copy() for k in self.bus}
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
                                            ngbr[k].remove(i + self.N)
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
                b = max(self.early[i], a)
                schedule[k].update({i:
                                        [a, b, (b - a), (b + srvc), lstload + self.model.parameters.load[i]]
                                    })
                d = b + srvc
                lstload += self.model.parameters.load[i]
                prev = i

        def slack(s, k):
            st = 0
            if type(s) == list:
                f = PriorityQueue()
                if s[st] != 0:
                    f.put(min(self.late[s[st]] - schedule[k][s[st]][1],
                              self.ridetime - (schedule[k][s[st]][3] -
                                               schedule[k][s[st] + self.N][1])))
                for i in range(st + 1, len(s)):
                    curr = schedule[k][s[i]]
                    w = 0
                    for j in range(st + 1, i + 1):
                        if s[j] != self.model.last:
                            w += schedule[k][s[j]][2]
                        if s[j] == self.model.last:
                            W = w
                    if s[i] in self.pickup:
                        w += min(self.late[s[i]] - curr[1],
                                 self.ridetime - (schedule[k][s[i] + self.N][1] -
                                                  curr[3]))
                    if s[i] not in self.pickup and s[i] != self.model.last:
                        w += min(self.late[s[i]] - curr[1],
                                 self.ridetime - (schedule[k][s[i] - self.N][3] -
                                                  curr[1]))
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
        self.calculateObjective(solution, sol, schedule)

    def calculateObjective(self, solution, sol, schedule):
        cost = 0  # Distance cost of the route
        xsol = [{}, {}, {}, {}]
        for k in sol.keys():
            dur = 0
            tmwndw = 0
            ridetm = 0
            for i in sol[k]:
                if i != 0:
                    xsol[0].update({(prev, i, k): 1})
                    xsol[1].update({(prev, i): schedule[k][i][2]})
                    xsol[3].update({i: schedule[k][i][1] - self.late[i]})
                prev = i
                if i == self.model.last:
                    solution[k][i].load = 0
                    solution[k][i].time = schedule[k][i][0]
                    break
                node = solution[k][i]
                # Load constraint of the route
                load = max(0, schedule[k][i][4] - self.model.parameters.capacity)
                if i == 0:
                    # Time duration of entire trip
                    dur = max(0, schedule[k][self.model.last][0] - schedule[k][0][3] - self.routetime)
                else:
                    # Time window constraint
                    tmwndw = max(0, schedule[k][i][1] - self.late[i])
                    # Customer ride-time constraint
                    ridetm = max(0, ((schedule[k][i + self.N][1] - schedule[k][i][3]) if i in self.pickup
                                     else (schedule[k][i][1] - schedule[k][i - self.N][1]))
                                 - self.ridetime)
                node.load = schedule[k][i][4]
                node.time = schedule[k][i][1]
                cost += self.model.parameters.distance[node.key, node.next.key] \
                        + self.weight['alpha'] * load \
                        + self.weight['beta'] * dur \
                        + self.weight['gamma'] * tmwndw \
                        + self.weight['delta'] * ridetm
                if i != 0:
                    node.value = PriorityQueue()
                    [node.value.put(i) for i in ((load, 'L'), (tmwndw, 'T'), (ridetm, 'R'))]
                else:
                    node.value = cost
        [self.model.submodel[i].fix_values(self, sol=xsol) for i in range(self.model.scenarios)]
        # Optimize Relaxed 2nd Stage
        [self.model.submodel[s].relax() for s in range(self.model.scenarios)]
        tsp = sum(self.model.scenarioprob[s] * self.model.submodel[s].relaxmod.ObjVal \
                  for s in range(self.model.scenarios))
        cost += tsp
        solution[-1] = cost
