import random as rdm
from queue import PriorityQueue
from LinkedList import *
import math


class Tabu():
    """Implements a tabu search heuristic for DARP problem"""

    def __init__(self, model, tabu_iterations=200, tabu_status=50, subset=5, rtoptm=20, roptiter=5):
        self.model = model
        self.bus = list(range(self.model.parameters.bus))
        self.N = self.model.parameters.rides
        self.pickup = self.model.parameters.pickup
        self.tabudict = {}
        self.tabuiter = tabu_iterations
        self.tabu_status = tabu_status
        self.rtoptm = rtoptm
        self.roptiter = roptiter
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
        self.scenarios = list(range(self.model.scenarios))
        self.subset = min(subset, self.model.scenarios)
        self.bestlist = []
        self.penalty = math.sqrt(len(self.bus)*len(self.pickup))*10
        self._init_weights()

    def _init_weights(self):
        self.weight = {
            'alpha': 100,
            'beta': 1,
            'gamma': 1,
            'delta': 1
        }

    def tabuheuristic(self):
        print('Building Initial Solution')
        self.bestcandidate = self.initialSolution()
        self.solutionEval(self.bestcandidate)
        self.best = self.bestcandidate
        self.bestlist.append(self.best[max(self.bus)].head.value)
        print('Starting Heuristic')
        for iter in range(self.tabuiter):
            for i, k in self.tabudict.keys():
                decr = self.tabudict[i, k][0] - 1
                self.tabudict[i, k][0] = max(0, decr)
            ngbr = self.neighborhoodGen()
            print('Neighborhood length: %d (iteration %d)' % (len(ngbr), iter+1))
            [self.solutionEval(n) for n in ngbr]
            for candidate in ngbr:
                if candidate[-1] < self.bestcandidate[-1]:
                    self.bestcandidate = candidate
            print(self.bestcandidate[-1],self.best[-1])
            if self.bestcandidate[-1] < self.best[-1] or iter % self.rtoptm == 0:
                if self.bestcandidate[-1] < self.best[-1]:
                    self.best = self.bestcandidate
                for k in self.bus:
                    for i in self.best[k]:
                        if i in self.pickup:
                            self.tabudict[i, k][0] = 0
                for itr in range(min(self.roptiter,2*len(self.pickup))):
                    sol = self.routeoptimization()
                    samesol = [self.best[k].list()==sol[k].list() for k in self.bus]
                    if sum(samesol) != len(self.bus):
                        self.solutionEval(sol)
                    if sol[-1] < self.best[-1]:
                        self.best = sol
                    # print(itr)
            self.bestlist.append(self.best[max(self.bus)].head.value)
        self.best[-1] = self.best[max(self.bus)].head.value
        return self.best

    def routeoptimization(self):
        sol = self.best
        prioritynodes = {k: PriorityQueue() for k in self.bus}
        opt = 0
        for k in self.bus:
            for i in sol[k]:
                if i != 0 and i != self.model.last:
                    if not sol[k][i].value.empty():
                        val, criteria = sol[k][i].value.get()
                        prioritynodes[k].put((val, criteria, i))
            if prioritynodes[k].empty():
                opt += 1
            if opt == len(self.bus):
                return sol
        for k in self.bus:
            sl = sol[k].copy()
            if not prioritynodes[k].empty():
                val, criteria, pnode = prioritynodes[k].get()
                val = -val
                node = sl[pnode]
                sl.remove(pnode)
                insrt = None
                insrtnw = 0
                if pnode not in self.pickup:
                    insrt = False
                    insrtnw = pnode - self.N
                for i in sl:
                    curr = sl[i]
                    if insrt or insrtnw == i:
                        if criteria == 'L' and i != self.model.last:
                            if curr.time > node.time and \
                            curr.load < self.model.parameters.capacity or curr.next.key == pnode+self.N:
                                node.next = curr.next
                                curr.next = node
                                break
                        if criteria == 'T' and i != self.model.last:
                            if curr.time > self.early[pnode] or \
                            curr.next.time > self.late[pnode] or curr.next.key == pnode+self.N:
                                node.next = curr.next
                                curr.next = node
                                break
                        insrt = True
                if pnode in sl.list():
                    sol[k] = sl
        return sol

    def initialSolution(self):
        solution = {i: LinkedList() for i in self.bus}
        solution[-1] = 0
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
                    ngbr[-1] = 0
                    try:
                        self.tabudict[i, k][0] = self.tabu_status
                    except KeyError:
                        self.tabudict.update({(i, k): [self.tabu_status, 1]})
                    for b in availbus:
                        try:
                            if self.tabudict[i, b][0] == 0:
                                node = ngbr[k][i]
                                ngbr[k].remove(i)
                                node.bus = b
                                for n in ngbr[b]:
                                    n = ngbr[b][n]
                                    if n.next.time >= node.time or \
                                            n.next.key == self.model.last:
                                        node.next = n.next
                                        n.next = node
                                        if node.key != i + self.N:
                                            node = ngbr[k][i + self.N]
                                            ngbr[k].remove(i + self.N)
                                            node.bus = b
                                        else:
                                            break
                                ngbr[-1] += self.penalty * self.tabudict[i, b][1]
                                break
                        except KeyError:
                            self.tabudict[i, b] = [0, 0]
                            node = ngbr[k][i]
                            ngbr[k].remove(i)
                            node.bus = b
                            for n in ngbr[b]:
                                n = ngbr[b][n]
                                if n.next.time >= node.time or \
                                        n.next.key == self.model.last:
                                    node.next = n.next
                                    n.next = node
                                    if node.key != i + self.N:
                                        node = ngbr[k][i + self.N]
                                        ngbr[k].remove(i + self.N)
                                        node.bus = b
                                    else:
                                        break
                            ngbr[-1] += self.penalty*self.tabudict[i, b][1]
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
                if i == self.model.last:
                    schedule[k].update({i: [(d + self.time[prev, i]), 0, 0, 0, lstload + 0]})
                    continue
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
                    f.put(max(0,
                    min(self.late[s[st]] - schedule[k][s[st]][1],
                              self.ridetime - (schedule[k][s[st]][3] -
                                               schedule[k][s[st] + self.N][1]))))
                for i in range(st + 1, len(s)):
                    curr = schedule[k][s[i]]
                    w = 0
                    for j in range(st + 1, i + 1):
                        if s[j] != self.model.last:
                            w += schedule[k][s[j]][2]
                        if s[j] == self.model.last:
                            W = w
                    if s[i] in self.pickup:
                        w += max(0,
                        min(self.late[s[i]] - curr[1],
                                 self.ridetime - (schedule[k][s[i] + self.N][1] -
                                                  curr[3])))
                    if s[i] not in self.pickup and s[i] != self.model.last:
                        w += max(0,
                        min(self.late[s[i]] - curr[1],
                                 self.ridetime - (schedule[k][s[i] - self.N][3] -
                                                  curr[1])))
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
                    schedule[k][sol[k][i]][3] = schedule[k][sol[k][i]][1] + srvc
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
                    xsol[3].update({i: max(0, schedule[k][i][1] - self.late[i])})
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
                    [node.value.put(i) for i in ((-load, 'L'), (-tmwndw, 'T'), (-ridetm, 'R'))]
                else:
                    node.time = schedule[k][0][3]
                    node.load = 0
        solution[-2] = cost
        # rdm.shuffle(self.scenarios)
        # S = self.scenarios[:self.subset]
        # [self.model.submodel[s].fix_values(self, sol=xsol) for s in S]
        # # Optimize Relaxed 2nd Stage
        # [self.model.submodel[s].relax() for s in S]
        # tsp = 0
        # for s in S:
        #     stat = self.model.submodel[s].relaxmod.status
        #     if stat == 4 or stat == 3:
        #         self.model.submodel[s].model.computeIIS()
        #         self.model.submodel[s].model.write('infeasible.ilp')
        #         self.model.submodel[s].model.write('infeasible.lp')
        #         exit(0)
        #     elif stat == 2:
        #         tsp += self.model.scenarioprob[s] * self.model.submodel[s].relaxmod.ObjVal
        #         # print(tsp)
        #         # self.model.submodel[s].relaxmod.write('feasible.lp')
        #         # self.model.submodel[s].relaxmod.write('feasible.sol')
        # cost += tsp
        solution[-1] += cost
