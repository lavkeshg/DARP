import random
from gurobipy import *
from beautifultable import BeautifulTable
from _datetime import timedelta


class Structure:
    pass


class MasterProblem:

    def __init__(self, Map, bus=1, probability=[0.8, 0.15, 0.03, 0.02]):
        self.model = Model()
        self.ClassProb = probability
        self.mapObject = Map
        self.last = self.mapObject.depot()[1]
        self.parameters = Structure()
        self.bus = bus
        self.variables = Structure()
        self.results = Structure()
        self.constraints = Structure()

        self.__ClassProb__()
        self._build_model()

    def _build_model(self):
        self._parameters()
        self._variables()
        self._constraints()
        # self._setobjective()
        self.head = ['Pick-up', 'Drop', 'Pickup time', 'Drop-off time']
        self.row = {}

    def optimize(self):
        # self.submodel = {i: SubProblem(self) for i in range(self.scenarios)}
        # self.masterLP = self.model.relax()
        # self.masterLP.optimize()
        # self.subLP = {i: self.submodel[i].relax() for i in range(self.scenarios)}
        #
        self.model.optimize()
        # self.printsol()

    def __ClassProb__(self):
        if len(self.ClassProb) != 4:
            raise TypeError("Expected array of length 4, got %g" % len(self.ClassProb))
        if sum(self.ClassProb) != 1:
            raise ValueError("Expected sum of probabilities to be 1")

    def _parameters(self):
        self.parameters.LowerBound = -1e7
        self.parameters.BigM = 1e5
        self.parameters.service_time = 5
        self.parameters.capacity = 3
        self.parameters.distance = self.mapObject.distance
        self.parameters.time = self.mapObject.run
        self.parameters.load = self.mapObject.load()
        self.parameters.pickup = self.mapObject.pickup()
        self.parameters.dropoff = self.mapObject.dropoff()
        self.parameters.pickup_time = self.mapObject.pickup_time
        self.parameters.dropoff_time = self.mapObject.dropoff_time
        self.parameters.pickup_time.update(self.parameters.dropoff_time)
        self.parameters.nodes = self.mapObject.nodes()
        self.parameters.rides = self.mapObject.N_riders
        self.parameters.edges = {(i, j) for i, j in self.parameters.distance.keys() if j != 0 and i != self.last and i != j}

    def _variables(self):
        m = self.model
        self.parameters.capacity = 3

        self.variables.x = m.addVars(self.parameters.edges, list(range(self.bus)),
                                     vtype=GRB.BINARY,
                                     obj={(i, j, k): self.parameters.distance[i, j] for i, j in
                                          self.parameters.distance.keys()
                                          for k in range(self.bus)}, name='x')
        self.variables.p_e = m.addVars(self.parameters.pickup + self.parameters.dropoff,
                                       vtype=GRB.CONTINUOUS, lb=0, obj=1, name='p_e')
        self.variables.p_l = m.addVars(self.parameters.pickup + self.parameters.dropoff,
                                       vtype=GRB.CONTINUOUS, lb=0, obj=1, name='p_l')
        self.variables.t = m.addVars(self.parameters.pickup + self.parameters.dropoff,
                                     vtype=GRB.CONTINUOUS, lb=0, obj=0, name='t')
        self.variables.td = m.addVars(self.mapObject.depot(), list(range(self.bus)),
                                      vtype=GRB.CONTINUOUS, lb=0, obj=0, name='t')
        self.variables.w = m.addVars(self.parameters.nodes, lb=0, ub=self.parameters.capacity, vtype=GRB.CONTINUOUS,
                                     name='w')
        self.variables.h = m.addVars(self.parameters.edges, vtype=GRB.CONTINUOUS, obj=1, name='h')
        # m.setObjective(quicksum(self.variables.x[i,j,k]*self.parameters.distance[i, j]
        #                         + self.variables.h[i,j]*self.variables.w[i]
        #                         for i,j in self.parameters.edges for k in range(self.bus))+
        #                quicksum(self.variables.p_e[i] + self.variables.p_l[i] for i in self.parameters.pickup +
        #                         self.parameters.dropoff))
        m.Params.NodefileStart = 0.5
        m.update()

    def _constraints(self):
        m = self.model
        b = range(self.bus)
        n = self.parameters.rides
        p = self.parameters.pickup
        d = self.parameters.dropoff

        self.constraints.demand = m.addConstrs((self.variables.x.sum(i, '*') == 1 for i in p + d),
                                               name='demand')
        self.constraints.flow = m.addConstrs(
            (self.variables.x.sum(i, '*', k) - self.variables.x.sum('*', i, k) == 0 for i in p +
             d for k in b), name='flow-constraint')
        m.addConstrs((quicksum(self.variables.x[0, j, k] for j in p + d) == 1
                      for k in b), name='depot-start')
        m.addConstrs((quicksum(self.variables.x[j, self.mapObject.depot()[1], k]
                               for j in p + d) == 1 for k in b), name='depot-end')
        m.addConstrs((self.variables.x.sum(i, '*', k) - self.variables.x.sum('*', n + i, k) == 0 for i in
                      p for k in b), name='pick-drop')
        m.addConstrs((self.variables.t[i] + self.parameters.time[i, i + n] <= self.variables.t[i + n] for i in
                      p), name='min-TOA')
        m.addConstrs(
            (self.variables.t[i] + self.parameters.time[i, j] + self.parameters.service_time + self.variables.h[i, j]
             <= self.variables.t[j] + self.parameters.BigM * (1 - self.variables.x[i, j, k]) for i in
             p + d
             for j in p + d
             for k in b if i != j), name='precedence-leq')
        m.addConstrs(
            (self.variables.t[i] + self.parameters.time[i, j] + self.parameters.service_time + self.variables.h[i, j]
             >= self.variables.t[j] + self.parameters.BigM * (self.variables.x[i, j, k] - 1) for i in
             p + d
             for j in p + d
             for k in b if i != j), name='precedence-geq')
        m.addConstrs((self.variables.td[0, k] + self.parameters.time[0, j] + self.variables.h[0, j] <=
                      self.variables.t[j] + self.parameters.BigM * (1 - self.variables.x[0, j, k]) for j in p + d
                      for k in b), name='start_time-leq')
        m.addConstrs((self.variables.td[0, k] + self.parameters.time[0, j] + self.variables.h[0, j] >=
                      self.variables.t[j] + self.parameters.BigM * (self.variables.x[0, j, k] - 1) for j in p
                      for k in b), name='start_time-geq')
        m.addConstrs((self.variables.t[i] + self.parameters.service_time + self.parameters.time[i, self.last] +
                      self.variables.h[i, self.last] <=
                      self.variables.td[self.last, k] + self.parameters.BigM * (1 - self.variables.x[i, self.last, k])
                      for i in p + d for k in b), name='end_time-leq')
        m.addConstrs((self.variables.t[i] + self.parameters.service_time + self.parameters.time[i, self.last] +
                      self.variables.h[i, self.last] >=
                      self.variables.td[self.last, k] + self.parameters.BigM * (self.variables.x[i, self.last, k] - 1)
                      for i in d for k in b), name='end_time-geq')
        m.addConstrs((self.variables.p_e[i] >= self.parameters.pickup_time[i] - 10 - self.variables.t[i] for i in
                      p + d), name='early-penalty')
        m.addConstrs((self.variables.p_l[i] >= -self.parameters.pickup_time[i] - 10 + self.variables.t[i] for i in
                      p + d), name='late-penalty')
        # ###Cite paper-15 for following tighter constraints
        m.addConstrs((self.variables.w[i] + self.parameters.load[j] <= self.variables.w[j] +
                      (1 - self.variables.x[i, j, k]) * self.parameters.BigM for i, j in self.parameters.edges for k in
                      b), name='load-balance')
        # m.addConstrs(self.variables.td[i,k] >= 480 for i in self.mapObject.depot() for k in b)
        m.write('TwoSP-Master.lp')

    def printsol(self, sub_model=None):
        if self.model.status == GRB.OPTIMAL:
            for k in range(self.bus):
                print('Routing for bus %g' % (k + 1))
                table = BeautifulTable()
                table.column_headers = self.head

                for i, j in self.parameters.edges:
                    if self.variables.x[i, j, k].Xn > 0.5 and i not in [0, self.last] and j not in [0, self.last]:
                        self.row[i, j, k] = [i, j, str(timedelta(minutes=self.variables.t[i].Xn)).rsplit(':', 1)[0],
                                             str(timedelta(minutes=self.variables.t[j].Xn)).rsplit(':', 1)[0]]
                        table.append_row(self.row[i, j, k])
                    elif self.variables.x[i, j, k].Xn > 0.5 and i == 0:
                        self.row[i, j, k] = [i, j, str(timedelta(minutes=self.variables.td[i, k].Xn)).rsplit(':', 1)[0],
                                             str(timedelta(minutes=self.variables.t[j].Xn)).rsplit(':', 1)[0]]
                        table.append_row(self.row[i, j, k])
                    elif self.variables.x[i, j, k].Xn > 0.5 and j == self.last:
                        self.row[i, j, k] = [i, j, str(timedelta(minutes=self.variables.t[i].Xn)).rsplit(':', 1)[0],
                                             str(timedelta(minutes=self.variables.td[j, k].Xn)).rsplit(':', 1)[0]]
                        table.append_row(self.row[i, j, k])
                print(table)
            if sub_model is not None:
                for s in sub_model.parameters.xi:
                    for k in range(sub_model.parameters.bus):
                        print('Scenario %g Routing for bus %g' % (s + 1, k + 1))
                        table = BeautifulTable()
                        table.column_headers = self.head

                        for i, j in sub_model.parameters.edges:
                            if sub_model.variables.xs[i, j, k, s].Xn > 0.5 and i not in [0, sub_model.last] \
                                    and j not in [0, sub_model.last]:
                                self.row[i, j, k, s] = [i, j,
                                                        str(timedelta(minutes=sub_model.variables.ts[i, s].Xn)).rsplit(
                                                            ':', 1)[0],
                                                        str(timedelta(minutes=sub_model.variables.ts[j, s].Xn)).rsplit(
                                                            ':', 1)[0]]
                                table.append_row(self.row[i, j, k, s])
                            elif sub_model.variables.xs[i, j, k, s].Xn > 0.5 and i == 0:
                                if j != self.last:
                                    self.row[i, j, k, s] = [i, j,
                                                            str(timedelta(
                                                                minutes=sub_model.variables.tds[i, k, s].Xn)).rsplit(
                                                                ':', 1)[
                                                                0],
                                                            str(timedelta(
                                                                minutes=sub_model.variables.ts[j, s].Xn)).rsplit(':',
                                                                                                                 1)[0]]
                                else:
                                    self.row[i, j, k, s] = [i, j,
                                                            str(timedelta(
                                                                minutes=sub_model.variables.tds[i, k, s].Xn)).rsplit(
                                                                ':', 1)[
                                                                0],
                                                            str(timedelta(
                                                                minutes=sub_model.variables.tds[j, k, s].Xn)).rsplit(
                                                                ':',
                                                                1)[0]]
                                table.append_row(self.row[i, j, k, s])
                            elif sub_model.variables.xs[i, j, k, s].Xn > 0.5 and j == sub_model.last:
                                if i != 0:
                                    self.row[i, j, k, s] = [i, j,
                                                            str(timedelta(
                                                                minutes=sub_model.variables.ts[i, s].Xn)).rsplit(':',
                                                                                                                 1)[0],
                                                            str(timedelta(
                                                                minutes=sub_model.variables.tds[j, k, s].Xn)).rsplit(
                                                                ':', 1)[
                                                                0]]
                                else:
                                    self.row[i, j, k, s] = [i, j,
                                                            str(timedelta(
                                                                minutes=sub_model.variables.tds[i, k, s].Xn)).rsplit(
                                                                ':',
                                                                1)[0],
                                                            str(timedelta(
                                                                minutes=sub_model.variables.tds[j, k, s].Xn)).rsplit(
                                                                ':', 1)[
                                                                0]]
                                table.append_row(self.row[i, j, k, s])
                        print(table)
            # sub_model.model.write('fole.sol')

    def result(self):
        self.results.holding = 0


class TwoStage:

    def __init__(self, Map, bus=1, probability=[0.8, 0.15, 0.03, 0.02], scenarios=5):
        self.MP = MasterProblem(Map, bus, probability)
        self.last = self.MP.last
        self.scenarios = scenarios
        self.variables = Structure()
        self.parameters = Structure()
        self.constraints = Structure()
        self.results = Structure()
        self.sim = Structure()
        self.build_model()

    def build_model(self):
        self.model = self.MP.model
        self._parameters()
        self._variables()
        self._constraints()
        self._setobjective()

    def _parameters(self):
        # self.parameters = self.MP.parameters
        self.parameters.BigM = self.MP.parameters.BigM
        self.parameters.service_time = self.MP.parameters.service_time
        self.parameters.capacity = self.MP.parameters.capacity
        self.parameters.distance = self.MP.parameters.distance
        self.parameters.time = self.MP.parameters.time
        self.parameters.load = self.MP.parameters.load
        self.parameters.pickup = self.MP.parameters.pickup
        self.parameters.dropoff = self.MP.parameters.dropoff
        self.parameters.pickup_time = self.MP.parameters.pickup_time
        self.parameters.nodes = self.MP.parameters.nodes
        self.parameters.rides = self.MP.parameters.rides
        self.parameters.edges = self.MP.parameters.edges
        self.bus = self.MP.bus
        self.parameters.xi = list(range(self.scenarios))
        self.parameters.wait_time = 7
        self._scenarioParamGen()

    def _scenarioParamGen(self):
        CC = self.MP.ClassProb
        S = self.scenarios
        seed = list(range(200, 200+S))
        n = self.parameters.rides
        Sims = self.parameters.xi
        show = lambda x, y, z, i, s: (x.update({(i, s): 1, (i + n, s): 1}),
                                      y.update({(i, s): 1, (i + n, s): 1}),
                                      z.update({(i, s): 0, (i + n, s): 0}))
        noshow = lambda x, y, z, i, s: (x.update({(i, s): 1, (i + n, s): 0}),
                                        y.update({(i, s): 0, (i + n, s): 0}),
                                        z.update({(i, s): 1, (i + n, s): 0}))
        sameday = lambda x, y, z, i, s: (x.update({(i, s): 0, (i + n, s): 0}),
                                         y.update({(i, s): 0, (i + n, s): 0}),
                                         z.update({(i, s): 0, (i + n, s): 0}))
        atthedoor = lambda x, y, z, i, s: (x.update({(i, s): 1, (i + n, s): 0}),
                                           y.update({(i, s): 0, (i + n, s): 0}),
                                           z.update({(i, s): 0, (i + n, s): 0}))
        self.sim.alpha = {}
        self.sim.beta = {}
        self.sim.gamma = {}

        for s in Sims:
            random.seed(seed[s])
            self.sim.alpha[0, s] = 1
            self.sim.alpha[self.last, s] = 1
            for i in self.parameters.pickup:
                j = random.random()
                if 0 <= j <= CC[0]:
                    show(self.sim.alpha, self.sim.beta, self.sim.gamma, i, s)
                elif CC[0] < j <= CC[0] + CC[1]:
                    sameday(self.sim.alpha, self.sim.beta, self.sim.gamma, i, s)
                elif CC[0] + CC[1] < j <= CC[0] + CC[1] + CC[2]:
                    atthedoor(self.sim.alpha, self.sim.beta, self.sim.gamma, i, s)
                else:
                    noshow(self.sim.alpha, self.sim.beta, self.sim.gamma, i, s)

    def _variables(self):
        m = self.model
        S = self.parameters.xi
        xi = self.scenarios
        p = self.parameters.pickup
        d = self.parameters.dropoff
        b = list(range(self.MP.bus))

        self.variables.xs = m.addVars(self.parameters.edges, b, S,
                                      vtype=GRB.BINARY, obj={(i, j, k, s): self.parameters.distance[i, j]/xi for i, j in
                                                             self.parameters.distance.keys()
                                                             for k in b for s in S}, name='x')
        self.variables.p_es = m.addVars(p + d, S, vtype=GRB.CONTINUOUS, lb=0, obj=1/xi, name='p_e')
        self.variables.p_ls = m.addVars(p + d, S, vtype=GRB.CONTINUOUS, lb=0, obj=1/xi, name='p_l')
        self.variables.ts = m.addVars(p + d, S, vtype=GRB.CONTINUOUS, lb=0, obj=0, name='t')
        self.variables.tds = m.addVars(self.MP.mapObject.depot(), b, S,
                                       vtype=GRB.CONTINUOUS, lb=0, obj=0.0001, name='t')
        self.variables.ws = m.addVars(self.parameters.nodes, S, lb=0, ub=self.parameters.capacity,
                                      vtype=GRB.CONTINUOUS, name='w')
        self.variables.hs = m.addVars(self.parameters.edges, S, vtype=GRB.CONTINUOUS, obj=1/xi, name='h')
        m.update()

    def _setobjective(self):
        m = self.model
        m.setObjective(
            # Master-problem
            quicksum(self.MP.variables.x[i, j, k] * self.parameters.distance[i, j]
                     + self.MP.variables.h[i, j]
                     # * self.MP.variables.w[i]
                     for i, j in self.parameters.edges for k in range(self.MP.bus)) +
            quicksum(self.MP.variables.p_e[i] + self.MP.variables.p_l[i] for i in self.parameters.pickup +
                     self.parameters.dropoff) +
            # Sub-problem
            (1 / self.scenarios) * (quicksum(self.parameters.distance[i, j] * (self.variables.xs[i, j, k, s] -
                                                                               self.MP.variables.x[i, j, k])
                                             + (self.variables.hs[i, j, s] - self.MP.variables.h[i, j])
                                             for i, j in self.parameters.edges
                                             for k in range(self.MP.bus) for s in self.parameters.xi) +
                                    quicksum((self.variables.p_es[i, s] - self.MP.variables.p_e[i])
                                             + (self.variables.p_ls[i, s] - self.MP.variables.p_l[i])
                                             for i in self.parameters.pickup + self.parameters.dropoff
                                             for s in self.parameters.xi))
        )

    def _constraints(self):
        m = self.model
        S = self.parameters.xi
        b = range(self.MP.bus)
        n = self.parameters.rides
        p = self.parameters.pickup
        d = self.parameters.dropoff
        al = self.sim.alpha
        bt = self.sim.beta
        ga = self.sim.gamma
        m.addConstrs((self.variables.xs.sum(i, '*', k, s) + self.variables.xs.sum('*', i, k, s) == 0
                      for i in p + d for k in b for s in S if al[i, s] == 0), name='Inactive-node')
        # m.addConstrs((-self.parameters.BigM * (1 - al[i, s] * al[j, s]) + self.MP.variables.x[i, j, k] <=
        #               self.variables.xs[i, j, k, s]
        #               for i, j in self.parameters.edges for k in b for s in S if i != 0),
        #              name='DnP-recourse')

        m.addConstrs(quicksum(self.variables.xs[0, j, k, s]*al[j, s] - self.MP.variables.x[0, j, k] for j in p + d)
                     + self.variables.xs[0, self.last, k, s] == 0 for k in b
                     for s in S)
        m.addConstrs((quicksum(self.variables.xs.sum(i, j, k, s) * al[j, s] - self.MP.variables.x.sum(i, j, k)
                               for j in self.parameters.nodes) == 0 for i in p +
                      d for k in b for s in S if al[i, s] == 1), name='flow-constraint-recourse_in')
        m.addConstrs((quicksum(self.MP.variables.x.sum(j, i, k) - self.variables.xs.sum(j, i, k, s) * al[j, s]
                               for j in self.parameters.nodes) == 0 for i in p +
                      d for k in b for s in S if al[i, s] == 1), name='flow-constraint-recourse_out')
        m.addConstrs((self.variables.ts[i, s] + self.parameters.time[i, i + n] <= self.variables.ts[i + n, s] for i in
                      p for s in S), name='min-TOA')
        m.addConstrs((self.variables.ts[i, s] + self.parameters.time[i, j] + bt[i, s] * self.parameters.service_time
                      + ga[i, s] * self.parameters.wait_time + self.variables.hs[i, j, s]
                      <= self.variables.ts[j, s] + self.parameters.BigM * (1 - self.variables.xs[i, j, k, s]) for i in
                      p + d for j in p + d for s in S
                      for k in b if i != j), name='precedence-leq')
        m.addConstrs((self.variables.ts[i, s] + self.parameters.time[i, j] + bt[i, s] * self.parameters.service_time
                      + ga[i, s] * self.parameters.wait_time + self.variables.hs[i, j, s]
                      >= self.variables.ts[j, s] + self.parameters.BigM * (self.variables.xs[i, j, k, s] - 1) for i in
                      p + d for j in p + d for s in S
                      for k in b if i != j), name='precedence-geq')
        m.addConstrs((self.variables.tds[0, k, s] + self.parameters.time[0, j] + self.variables.hs[0, j, s] <=
                      self.variables.ts[j, s] + self.parameters.BigM * (1 - self.variables.xs[0, j, k, s]) for j in
                      p + d
                      for k in b for s in S), name='start_time-leq')
        m.addConstrs((self.variables.tds[0, k, s] + self.parameters.time[0, j] + self.variables.hs[0, j, s] >=
                      self.variables.ts[j, s] + self.parameters.BigM * (self.variables.xs[0, j, k, s] - 1) for j in p
                      for k in b for s in S), name='start_time-geq')
        m.addConstrs(
            (self.variables.ts[i, s] + self.parameters.service_time + bt[i, s] * self.parameters.time[i, self.last]
             + ga[i, s] * self.parameters.wait_time + self.variables.hs[i, self.last, s] <=
             self.variables.tds[self.last, k, s] + self.parameters.BigM * (1 - self.variables.xs[i, self.last, k, s])
             for i in p + d for k in b for s in S), name='end_time-leq')
        m.addConstrs(
            (self.variables.ts[i, s] + self.parameters.service_time + bt[i, s] * self.parameters.time[i, self.last]
             + ga[i, s] * self.parameters.wait_time + self.variables.hs[i, self.last, s] >=
             self.variables.tds[self.last, k, s] + self.parameters.BigM * (self.variables.xs[i, self.last, k, s] - 1)
             for i in d for k in b for s in S), name='end_time-geq')
        m.addConstrs(
            (self.variables.p_es[i, s] >= self.parameters.pickup_time[i] - 20 - self.variables.ts[i, s] for i in
             p + d for s in S), name='early-penalty')
        m.addConstrs(
            (self.variables.p_ls[i, s] >= -self.parameters.pickup_time[i] - 20 + self.variables.ts[i, s] for i in
             p + d for s in S), name='late-penalty')

        m.write('TwoSP.lp')

    def printsol(self):
        self.MP.printsol(self)

    def optimize(self):
        def test(model, where):
            if where == GRB.callback.MIPSOL:
                if model.cbGet(GRB.Callback.MIPSOL_OBJ) > 1e4:
                    model.terminate()

        self.model.optimize()