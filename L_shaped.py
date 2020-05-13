import random
from gurobipy import *
from beautifultable import BeautifulTable
from _datetime import timedelta

class Structure:
    pass


class MasterProblem:

    def __init__(self, Map, bus=1, scenarios=5, probability=[0.8, 0.15, 0.03, 0.02], benders_gap=0.001, seed=200):
        self.model = Model()
        self.mapObject = Map
        self.seeder = seed
        self.maxiters = self.mapObject.N_riders*scenarios*scenarios
        self.ClassProb = probability
        self.last = self.mapObject.depot()[1]
        self.parameters = Structure()
        self.parameters.bus = bus
        self.variables = Structure()
        self.constraints = Structure()
        self.results = Structure()
        self.data = Structure()
        self.data.benders_gap = benders_gap
        self.scenarios = scenarios
        self.scenarioprob = {i: 1/scenarios for i in range(scenarios)}
        self.iters = 0
        # Instance attribute initialization
        self.submodel = None
        self.masterLP = None
        # Method instantiation
        self._Benders_init()
        self._build_model()
        self.__ClassProb__()

    def _build_model(self):
        self._parameters()
        self._variables()
        self._constraints()
        self.model.Params.lazyConstraints = 1
        self.model.Params.NodefileStart = 0.5
        self.model.Params.Presolve = 2
        self.model.Params.MIPFocus = 2
        self.model.params.Heuristics = 0.05
        self.head = ['Pick-up', 'Drop', 'Pickup time', 'Drop-off time']
        self.row = {}
        self.infeasol = []

    def optimize(self):
        self.model.Params.OutputFlag = 0
        self.model.optimize()
        self.setLowerBound()
        self._Benders_decomp()
        self.setMIP()
        self.solutions = []
        self.solutionsLP = []

        def OptCutFun(model, where):
            if where == GRB.callback.MIPSOL:
                    # and ((self.upperbounds[-1] - self.lowerbounds[-1]) >
                    #                              abs(self.data.benders_gap * self.lowerbounds[-1]) or self.iters == 0):
                x_sol = model.cbGetSolution(self.variables.x)
                h_sol = model.cbGetSolution(self.variables.h)
                pe_sol = model.cbGetSolution(self.variables.p_e)
                pl_sol = model.cbGetSolution(self.variables.p_l)
                sol = [x_sol, h_sol, pe_sol, pl_sol]
                th_sol = model.cbGetSolution(self.variables.th)
                if x_sol in self.solutions:
                    return True
                [self.submodel[s].fix_values(sol=sol) for s in range(self.scenarios)]
                if x_sol not in self.solutionsLP:
                    [self.submodel[s].relax() for s in range(self.scenarios)]
                    z_subLP = sum(self.scenarioprob[s] * self.submodel[s].relaxmod.ObjVal for s in range(self.scenarios))
                    self.solutionsLP.append(x_sol)
                    if z_subLP > th_sol:
                        self.add_sub_cuts(z_subLP, x_sol=x_sol)
                        self.LazySub += 1
                        objVal = model.cbGet(GRB.Callback.MIPSOL_OBJ)
                        objBst = model.cbGet(GRB.Callback.MIPSOL_OBJBST)
                        self.update_bounds(z_subLP, th_sol=th_sol, objVal=objVal, objBst=objBst)
                        # print('Sub-Gradient cut installed')
                    else:
                        # print('Sub-Gradient cut NOT installed')
                        pass

                try:
                    [self.submodel[s].optimize() for s in range(self.scenarios)]
                    z_sub = sum(self.scenarioprob[s] * self.submodel[s].model.ObjVal for s in range(self.scenarios))
                    self.solutions.append(x_sol)
                    if z_sub > th_sol:
                        self.LazyInt += 1
                        model.cbLazy((z_sub - self.subobjBst) *
                                     (quicksum(self.variables.x[i, j, k] for i, j, k in x_sol.keys() if
                                               x_sol[i, j, k] >= 0.5) -
                                      quicksum(self.variables.x[i, j, k] for i, j, k in x_sol.keys() if
                                               x_sol[i, j, k] < 0.5) -
                                      len(x_sol)) + z_sub <= self.variables.th)
                        objVal = model.cbGet(GRB.Callback.MIPSOL_OBJ)
                        objBst = model.cbGet(GRB.Callback.MIPSOL_OBJBST)
                        self.update_bounds(z_sub, th_sol=th_sol, objVal=objVal, objBst=objBst)
                        # print('Integrality cut installed')
                    else:
                        # print('Integrality cut NOT installed')
                        pass
                except AttributeError:
                    self.infeasol.append(sol)

                self.iters += 1

        self.model.Params.OutputFlag = 1
        self.model.optimize(OptCutFun)
        [self.submodel[s].fix_values() for s in range(self.scenarios)]
        [self.submodel[s].optimize() for s in range(self.scenarios)]
        z_sub = sum(self.scenarioprob[s] * self.submodel[s].model.ObjVal for s in range(self.scenarios))
        self.update_bounds(z_sub)
        print('Apprx = %f' % self.variables.th.X)
        print('ObjVal = %f' % z_sub)

    # def tabuHeuristic(self):
    #     # Construct initial solution
    #     self.tabu_init_()
    #     tabu = Tabu(self)
    #     best =
    #     localbest =

    def setLowerBound(self):
        seed = list(range(self.seeder, self.seeder + self.scenarios))
        self.submodel = {s: SubProblem(self, seed[s]) for s in range(self.scenarios)}
        [self.submodel[s].optimize() for s in range(self.scenarios)]
        # self.subobjBst = sum(self.scenarioprob[s] * self.submodel[s].model.ObjVal for s in range(self.scenarios))
        self.subobjBst = -GRB.INFINITY
        [self.submodel[s].Fixfirststage() for s in range(self.scenarios)]

    def _Benders_init(self):
        self.data.ub = GRB.INFINITY
        self.data.lb = -GRB.INFINITY
        z_sub = {}
        sens = {}
        self.constraints.cuts = {}
        self.cutlist = []
        self.upperbounds = []
        self.lowerbounds = []
        self.subobjBst = 0
        self.LazySub = 0
        self.LazyInt = 0

    def _Benders_decomp(self):
        [self.submodel[i].fix_values(self) for i in range(self.scenarios)]
        [self.submodel[i].relax() for i in range(self.scenarios)]
        z_sub = sum(self.scenarioprob[s] * self.submodel[s].relaxmod.ObjVal for s in range(self.scenarios))
        self.update_bounds(z_sub)
        print(self.variables.th.X,z_sub)
        while (abs(self.data.ub - self.data.lb) > abs(self.data.benders_gap * self.data.lb)) \
                and (len(self.cutlist) < self.maxiters):
            if z_sub > self.variables.th.X:
                self.add_sub_cuts(z_sub)
            self.model.optimize()
            [self.submodel[i].fix_values(self) for i in range(self.scenarios)]
            [self.submodel[i].relax() for i in range(self.scenarios)]
            z_sub = sum(self.scenarioprob[s] * self.submodel[s].relaxmod.ObjVal for s in range(self.scenarios))
            self.update_bounds(z_sub)
            print(self.variables.th.X, z_sub)
            print(self.data.ub,self.data.lb)

    def setMIP(self):
        for i,j,k in self.variables.x.keys():
            self.variables.x[i,j,k].vtype = 'B'
        for i in self.parameters.nodes:
            self.variables.r[i].vtype = 'B'

    def __ClassProb__(self):
        if len(self.ClassProb) != 4:
            raise TypeError("Expected array of length 4, got %g" % len(self.ClassProb))
        if sum(self.ClassProb) < 0.99999999999:
            raise ValueError("Expected sum of probabilities to be 1")

    def _parameters(self):
        self.parameters.BigM = 1e3
        self.parameters.service_time = 5
        self.parameters.capacity = 3
        self.parameters.ridetime = 60
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
        self.parameters.edges = {(i, j) for i, j in self.parameters.distance.keys() if i != self.last and j != 0}

    def _variables(self):
        m = self.model
        self.parameters.capacity = 3

        self.variables.x = m.addVars(self.parameters.edges, list(range(self.parameters.bus)),
                                     vtype=GRB.CONTINUOUS, lb=0, ub=1,
                                     obj={(i, j, k): self.parameters.distance[i, j] for i, j in
                                          self.parameters.distance.keys()
                                          for k in range(self.parameters.bus)}, name='x')
        self.variables.p_e = m.addVars(self.parameters.pickup + self.parameters.dropoff,
                                       vtype=GRB.CONTINUOUS, lb=0, obj=1, name='p_e')
        self.variables.p_l = m.addVars(self.parameters.pickup + self.parameters.dropoff,
                                       vtype=GRB.CONTINUOUS, lb=0, obj=1, name='p_l')
        self.variables.t = m.addVars(self.parameters.pickup + self.parameters.dropoff,
                                     vtype=GRB.CONTINUOUS, lb=0, obj=0, name='t')
        self.variables.td = m.addVars(self.mapObject.depot(), list(range(self.parameters.bus)),
                                      vtype=GRB.CONTINUOUS, lb=0, obj=0, name='t')
        self.variables.w = m.addVars(self.parameters.nodes, lb=0, ub=self.parameters.capacity, vtype=GRB.CONTINUOUS,
                                     name='w')
        self.variables.h = m.addVars(self.parameters.edges, vtype=GRB.CONTINUOUS, obj=1, name='h')
        self.variables.th = m.addVar(vtype=GRB.CONTINUOUS, lb=-1e4,
                                     ub=GRB.INFINITY, obj=1, name='value-func')
        self.variables.r = m.addVars(self.parameters.nodes, lb=0, obj=1, vtype=GRB.CONTINUOUS, name='D-Var')

        m.update()

    def _constraints(self):
        m = self.model
        b = range(self.parameters.bus)
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
        m.addConstrs((
            # (self.variables.r[i] == 1) >>
             (self.variables.t[i] + self.parameters.time[i, j] + self.parameters.service_time + self.variables.h[i, j]
             <= self.variables.t[j] + self.parameters.BigM * (2 - self.variables.x[i, j, k] - self.variables.r[i])) for i in
             p + d
             for j in p + d
             for k in b if i != j), name='hold-precedence-leq')
        m.addConstrs((
            # (self.variables.r[i] == 1) >>
             (self.variables.t[i] + self.parameters.time[i, j] + self.parameters.service_time + self.variables.h[i, j]
             >= self.variables.t[j] - self.parameters.BigM * (2 - self.variables.x[i, j, k] - self.variables.r[i])) for i in
             p + d
             for j in p + d
             for k in b if i != j), name='hold-precedence-geq')
        m.addConstrs((
            # (self.variables.r[i] == 0) >>
             (self.variables.t[i] + self.parameters.time[i, j] + self.parameters.service_time
              <= self.variables.t[j] + self.parameters.BigM * (1 - self.variables.x[i, j, k] + self.variables.r[i])) for i in
             p + d
             for j in p + d
             for k in b if i != j), name='precedence-leq')
        m.addConstrs((self.variables.td[0, k] + self.parameters.time[0, j] + self.variables.h[0, j] <=
                      self.variables.t[j] + self.parameters.BigM * (1 - self.variables.x[0, j, k]) for j in p + d
                      for k in b), name='start_time-leq')
        m.addConstrs((self.variables.td[0, k] + self.parameters.time[0, j] + self.variables.h[0, j] >=
                      self.variables.t[j] + self.parameters.BigM * (self.variables.x[0, j, k] - 1) for j in p + d
                      for k in b), name='start_time-geq')
        m.addConstrs((self.variables.t[i] + self.parameters.service_time + self.parameters.time[i, self.last] +
                      self.variables.h[i, self.last] <=
                      self.variables.td[self.last, k] + self.parameters.BigM * (1 - self.variables.x[i, self.last, k])
                      for i in p + d for k in b), name='end_time-leq')
        m.addConstrs((self.variables.t[i] + self.parameters.service_time + self.parameters.time[i, self.last] +
                      self.variables.h[i, self.last] >=
                      self.variables.td[self.last, k] + self.parameters.BigM * (self.variables.x[i, self.last, k] - 1)
                      for i in p + d for k in b), name='end_time-geq')
        m.addConstrs((self.variables.p_e[i] >= self.parameters.pickup_time[i] - 10 - self.variables.t[i] for i in
                      p + d), name='early-penalty')
        m.addConstrs((self.variables.p_l[i] >= -self.parameters.pickup_time[i] - 10 + self.variables.t[i] for i in
                      p + d), name='late-penalty')
        m.addConstrs((self.variables.w[i] + self.parameters.load[j] <= self.variables.w[j] +
                      (1 - self.variables.x[i, j, k]) * self.parameters.capacity for i, j in self.parameters.edges for k in
                      b), name='load-balance')
        m.addConstrs(self.variables.x[0, self.last, k] == 0 for k in b)
        m.addConstrs((self.variables.x[i, j, k] + self.variables.x[j, i, k] <= 1 for i in p+d for j in p+d for k in b
                     if i != j), name='forbidden')
        m.addConstrs(self.variables.w[i] <= self.parameters.capacity*self.variables.r[i] for i in self.parameters.nodes)
        m.write('L-shaped-Master.lp')

    def printsol(self, sub_model=None):
        if self.model.status == GRB.OPTIMAL:
            for k in range(self.parameters.bus):
                print('Routing for bus %g' % (k + 1))
                table = BeautifulTable()
                table.column_headers = self.head

                for i, j in self.parameters.edges:
                    if self.variables.x[i, j, k].X > 0.5 and i not in [0, self.last] and j not in [0, self.last]:
                        self.row[i, j, k] = [i, j, str(timedelta(minutes=self.variables.t[i].X)).rsplit(':', 1)[0],
                                             str(timedelta(minutes=self.variables.t[j].X)).rsplit(':', 1)[0]]
                        table.append_row(self.row[i, j, k])
                    elif self.variables.x[i, j, k].X > 0.5 and i == 0:
                        self.row[i, j, k] = [i, j, str(timedelta(minutes=self.variables.td[i, k].X)).rsplit(':', 1)[0],
                                             str(timedelta(minutes=self.variables.t[j].X)).rsplit(':', 1)[0]]
                        table.append_row(self.row[i, j, k])
                    elif self.variables.x[i, j, k].X > 0.5 and j == self.last:
                        self.row[i, j, k] = [i, j, str(timedelta(minutes=self.variables.t[i].X)).rsplit(':', 1)[0],
                                             str(timedelta(minutes=self.variables.td[j, k].X)).rsplit(':', 1)[0]]
                        table.append_row(self.row[i, j, k])
                print(table)
            if sub_model is not None:
                for s in sub_model.keys():
                    for k in range(sub_model[s].parameters.bus):
                        print('Scenario %g Routing for bus %g' % (s + 1, k + 1))
                        table = BeautifulTable()
                        table.column_headers = self.head

                        for i, j in sub_model[s].parameters.edges:
                            if sub_model[s].variables.xs[i, j, k].X > 0.5 and i not in [0, sub_model[s].last] \
                                    and j not in [0, sub_model[s].last]:
                                self.row[i, j, k] = [i, j,
                                                        str(timedelta(minutes=sub_model[s].variables.ts[i].X)).rsplit(
                                                            ':', 1)[0],
                                                        str(timedelta(minutes=sub_model[s].variables.ts[j].X)).rsplit(
                                                            ':', 1)[0]]
                                table.append_row(self.row[i, j, k])
                            elif sub_model[s].variables.xs[i, j, k].X > 0.5 and i == 0:
                                if j != self.last:
                                    self.row[i, j, k] = [i, j,
                                                            str(timedelta(
                                                                minutes=sub_model[s].variables.tds[i, k].X)).rsplit(
                                                                ':', 1)[
                                                                0],
                                                            str(timedelta(
                                                                minutes=sub_model[s].variables.ts[j].X)).rsplit(':',
                                                                                                                 1)[0]]
                                else:
                                    self.row[i, j, k] = [i, j,
                                                            str(timedelta(
                                                                minutes=sub_model[s].variables.tds[i, k].X)).rsplit(
                                                                ':', 1)[
                                                                0],
                                                            str(timedelta(
                                                                minutes=sub_model[s].variables.tds[j, k].X)).rsplit(
                                                                ':',
                                                                1)[0]]
                                table.append_row(self.row[i, j, k])
                            elif sub_model[s].variables.xs[i, j, k].X > 0.5 and j == sub_model[s].last:
                                if i != 0:
                                    self.row[i, j, k] = [i, j,
                                                            str(timedelta(
                                                                minutes=sub_model[s].variables.ts[i].X)).rsplit(':',
                                                                                                                 1)[0],
                                                            str(timedelta(
                                                                minutes=sub_model[s].variables.tds[j, k].X)).rsplit(
                                                                ':', 1)[
                                                                0]]
                                else:
                                    self.row[i, j, k] = [i, j,
                                                            str(timedelta(
                                                                minutes=sub_model[s].variables.tds[i, k].X)).rsplit(
                                                                ':',
                                                                1)[0],
                                                            str(timedelta(
                                                                minutes=sub_model[s].variables.tds[j, k].X)).rsplit(
                                                                ':', 1)[
                                                                0]]
                                table.append_row(self.row[i, j, k])
                        print(table)

    def add_sub_cuts(self, z_sub, x_sol=None):
        m = self.model

        # z_sub = sum(self.scenarioprob[s] * self.submodel[s].relaxmod.ObjVal for s in range(self.scenarios))
        sens = {(i, j, k): sum(self.submodel[s].relaxmod.getConstrByName('fix_x[{},{},{}]'.format(i,j,k)).pi
                               * self.scenarioprob[s] for s in range(self.scenarios))
                for i, j, k in self.variables.x.keys()}
        cut = len(self.cutlist)
        self.cutlist.append(cut)
        if x_sol is None:
            self.constraints.cuts[cut] = m.addConstr(z_sub + quicksum(sens[i, j, k] * self.variables.x[i, j, k]
                                                                      for i, j, k in self.variables.x.keys())
                                                     - sum(sens[i, j, k] * self.variables.x[i, j, k].X
                                                           for i, j, k in self.variables.x.keys())
                                                     <= self.variables.th, name='Cut-Const[{}]'.format(cut))
        else:
            m.cbLazy(z_sub + quicksum(sens[i, j, k] * self.variables.x[i, j, k]
                                      for i, j, k in self.variables.x.keys())
                     - sum(sens[i, j, k] * x_sol[i, j, k]
                           for i, j, k in self.variables.x.keys()) <= self.variables.th)

    def update_bounds(self, z_sub, th_sol=None, objVal=None, objBst = None):
        theta = self.variables.th.X if th_sol is None else th_sol
        z_master = self.model.ObjVal if objVal is None else objVal
        self.data.ub = z_master - theta + z_sub
        # The current best solution is the known lower bound to the model
        try:
            self.data.lb = self.model.ObjBound if objBst is None else objBst
        except GurobiError:
            self.data.lb = self.model.ObjVal if objVal is None else objVal
        self.upperbounds.append(self.data.ub)
        self.lowerbounds.append(self.data.lb)


class SubProblem:

    def __init__(self, MP, seed):
        self.MP = MP
        self.last = self.MP.last
        self.seed = seed
        self.variables = Structure()
        self.parameters = Structure()
        self.results = Structure()
        self.sim = Structure()
        self.parameters = self.MP.parameters
        self.model = None
        self.relaxmod = None
        self.build_model()

    def optimize(self):
        self.model.optimize()

    def build_model(self, model=None):
        self.model = Model() if model is None else model
        self._parameters()
        self._scenarioParamGen()
        self._variables()
        self._constraints()
        self.model.Params.OutputFlag = 0
        self._setobjective()

    def _parameters(self):
        self.parameters.wait_time = 7

    def relax(self):
        self.relaxmod.optimize()

    def _scenarioParamGen(self):
        random.seed(self.seed)
        CC = self.MP.ClassProb
        n = self.parameters.rides
        show = lambda x, y, z, i: (x.update({i: 1, (i + n): 1}),
                                      y.update({i: 1, (i + n): 1}),
                                      z.update({i: 0, (i + n): 0}))
        noshow = lambda x, y, z, i: (x.update({i: 1, (i + n): 0}),
                                        y.update({i: 0, (i + n): 0}),
                                        z.update({i: 1, (i + n): 0}))
        sameday = lambda x, y, z, i: (x.update({i: 0, (i + n): 0}),
                                         y.update({i: 0, (i + n): 0}),
                                         z.update({i: 0, (i + n): 0}))
        atthedoor = lambda x, y, z, i: (x.update({i: 1, (i + n): 0}),
                                           y.update({i: 0, (i + n): 0}),
                                           z.update({i: 0, (i + n): 0}))
        self.sim.alpha = {}
        self.sim.beta = {}
        self.sim.gamma = {}

        self.sim.alpha[0] = 1
        self.sim.alpha[self.last] = 1
        for i in self.parameters.pickup:
            j = random.random()
            if 0 <= j <= CC[0]:
                show(self.sim.alpha, self.sim.beta, self.sim.gamma, i)
            elif CC[0] < j <= CC[0] + CC[1]:
                sameday(self.sim.alpha, self.sim.beta, self.sim.gamma, i)
            elif CC[0] + CC[1] < j <= CC[0] + CC[1] + CC[2]:
                atthedoor(self.sim.alpha, self.sim.beta, self.sim.gamma, i)
            else:
                noshow(self.sim.alpha, self.sim.beta, self.sim.gamma, i)

    def _variables(self):
        m = self.model
        p = self.parameters.pickup
        d = self.parameters.dropoff
        b = list(range(self.MP.parameters.bus))

        self.variables.xs = m.addVars(self.parameters.edges, b,
                                      vtype=GRB.BINARY, obj={(i, j, k): self.parameters.distance[i, j] for i, j in
                                                             self.parameters.distance.keys()
                                                             for k in b}, name='x')
        self.variables.p_es = m.addVars(p + d, vtype=GRB.CONTINUOUS, lb=0, obj=1, name='p_e')
        self.variables.p_ls = m.addVars(p + d, vtype=GRB.CONTINUOUS, lb=0, obj=1, name='p_l')
        self.variables.ts = m.addVars(p + d, vtype=GRB.CONTINUOUS, lb=0, obj=0, name='t')
        self.variables.tds = m.addVars(self.MP.mapObject.depot(), b,
                                       vtype=GRB.CONTINUOUS, lb=0, obj=0.0001, name='t')
        self.variables.ws = m.addVars(self.parameters.nodes, lb=0, ub=self.parameters.capacity*10,
                                      vtype=GRB.CONTINUOUS, name='w')
        self.variables.hs = m.addVars(self.parameters.edges, vtype=GRB.CONTINUOUS, obj=1, name='h')
        self.variables.rs = m.addVars(self.parameters.nodes, vtype=GRB.BINARY, obj=1, name='r')
        self.variables.fix_x = m.addVars(self.parameters.edges, b, vtype=GRB.BINARY, name='fix_x')
        self.variables.fix_h = m.addVars(self.parameters.edges, vtype=GRB.CONTINUOUS, name='fix_h')
        self.variables.fix_p_e = m.addVars(p + d, vtype=GRB.CONTINUOUS, name='fix_p_e')
        self.variables.fix_p_l = m.addVars(p + d, vtype=GRB.CONTINUOUS, name='fix_p_l')
        m.update()

    def _setobjective(self):
        m = self.model
        m.setObjective(
            (quicksum(self.parameters.distance[i, j] * (self.variables.xs[i, j, k] - self.variables.fix_x[i, j, k]) +
                      (self.variables.hs[i, j] - self.variables.fix_h[i, j])
                      for i, j in self.parameters.edges for k in range(self.MP.parameters.bus))
             + quicksum((self.variables.p_es[i] - self.variables.fix_p_e[i])
                        + (self.variables.p_ls[i] - self.variables.fix_p_l[i])
                        for i in self.parameters.pickup + self.parameters.dropoff))
        )

    def _constraints(self):
        m = self.model
        b = list(range(self.MP.parameters.bus))
        n = self.parameters.rides
        p = self.parameters.pickup
        d = self.parameters.dropoff
        al = self.sim.alpha
        bt = self.sim.beta
        ga = self.sim.gamma
        m.addConstrs((self.variables.xs.sum(i, '*', k) + self.variables.xs.sum('*', i, k) == 0
                      for i in p + d for k in b if al[i] <= 0), name='Inactive-node')
        m.addConstrs((self.variables.fix_x[i, j, k] <= self.variables.xs[i, j, k]
                      for i, j in self.parameters.edges for k in b if al[i] == 1 and al[j] == 1),
                     name='DnP-recourse')
        m.addConstrs(quicksum(self.variables.xs[0, j, k]*al[j] for j in p + d)
                     + self.variables.xs[0, self.last, k] >= 1 for k in b)
        m.addConstrs((quicksum(al[j]*(self.variables.xs[i,j,k]) for j in self.parameters.nodes if i!=j and j!=0) -
                      quicksum(al[j]*(self.variables.xs[j,i,k]) for j in self.parameters.nodes if i!=j and j!=self.last)
                      >= 0 for i in p+d
                      for k in b if al[i] == 1), name='flow_constraint')
        m.addConstrs((self.variables.ts[i] + self.parameters.time[i, j] + bt[i] * self.parameters.service_time
                      + ga[i] * self.parameters.wait_time + self.variables.hs[i, j]
                      <= self.variables.ts[j] - self.parameters.BigM *
                       (self.variables.rs[i] + self.variables.xs[i, j, k] - 2) for i in
                      p + d for j in p + d
                      for k in b if i != j and al[i] == 1 and al[j] == 1), name='hold-precedence-leq')
        m.addConstrs((self.variables.ts[i] + self.parameters.time[i, j] + bt[i] * self.parameters.service_time
                      + ga[i] * self.parameters.wait_time + self.variables.hs[i, j]
                      >= self.variables.ts[j] + self.parameters.BigM *
                      (self.variables.rs[i] + self.variables.xs[i, j, k] - 2) for i in
                      p + d for j in p + d
                      for k in b if i != j and al[i] == 1 and al[j] == 1), name='hold-precedence-geq')
        m.addConstrs(((self.variables.ts[i] + self.parameters.time[i, j] + bt[i] * self.parameters.service_time
                       + ga[i] * self.parameters.wait_time
                       <= self.variables.ts[j] - self.parameters.BigM * (self.variables.xs[i, j, k] - 1)) for i in
                      p + d for j in p + d
                      for k in b if i != j and al[i] == 1 and al[j] == 1), name='precedence-leq')
        m.addConstrs((self.variables.tds[0, k] + self.parameters.time[0, j] + self.variables.hs[0, j] <=
                      self.variables.ts[j] + self.parameters.BigM * (1 - self.variables.xs[0, j, k]) for j in p + d
                      for k in b if al[j] == 1), name='start_time-leq')
        m.addConstrs((self.variables.tds[0, k] + self.parameters.time[0, j] + self.variables.hs[0, j] >=
                      self.variables.ts[j] + self.parameters.BigM * (self.variables.xs[0, j, k] - 1) for j in p + d
                      for k in b if al[j] == 1), name='start_time-geq')
        m.addConstrs(
            (self.variables.ts[i] + self.parameters.service_time + bt[i] * self.parameters.time[i, self.last]
             + ga[i] * self.parameters.wait_time + self.variables.hs[i, self.last] <=
             self.variables.tds[self.last, k] + self.parameters.BigM * (1 - self.variables.xs[i, self.last, k])
             for i in p + d for k in b if al[i] == 1), name='end_time-leq')
        m.addConstrs(
            (self.variables.ts[i] + self.parameters.service_time + bt[i] * self.parameters.time[i, self.last]
             + ga[i] * self.parameters.wait_time + self.variables.hs[i, self.last] >=
             self.variables.tds[self.last, k] + self.parameters.BigM * (self.variables.xs[i, self.last, k] - 1)
             for i in p + d for k in b if al[i] == 1), name='end_time-geq')
        m.addConstrs(
            (self.variables.p_es[i] >= self.parameters.pickup_time[i] - 10 - self.variables.ts[i] for i in
             p + d), name='early-penalty')
        m.addConstrs(
            (self.variables.p_ls[i] >= -self.parameters.pickup_time[i] - 10 + self.variables.ts[i] for i in
             p + d), name='late-penalty')
        m.addConstrs((self.variables.ws[i] + self.parameters.load[j] <= self.variables.ws[j] +
                      (1 - self.variables.xs[i, j, k]) * self.parameters.BigM for i, j in self.parameters.edges for k in
                      b if al[i] == 1 and al[j] == 1), name='load-balance')
        m.addConstrs(self.variables.ws[i] <= self.parameters.capacity*10*self.variables.rs[i] for i in self.parameters.nodes)
        m.write('L-shaped-Sub.lp')

    def Fixfirststage(self):
        m = self.model
        self.fixedconst_x = m.addConstrs(
            (self.variables.fix_x[i, j, k] == 0 for i, j, k in self.variables.fix_x.keys()), name='fix_x'
        )
        self.fixedconst_h = m.addConstrs(
            (self.variables.fix_h[i, j] == 0 for i, j in self.variables.fix_h.keys()), name='fix_h'
        )
        self.fixedconst_p_e = m.addConstrs(
            (self.variables.fix_p_e[i] == 0 for i in self.variables.fix_p_e.keys()), name='fix_p_e'
        )
        self.fixedconst_p_l = m.addConstrs(
            (self.variables.fix_p_l[i] == 0 for i in self.variables.fix_p_l.keys()), name='fix_p_l'
        )
        m.update()
        self.relaxmod = self.model.relax()

    def fix_values(self, MP=None, sol = None):
        if MP is None:
            m = self.MP
        else:
            m = MP
        if sol is None:
            for i, j, k in self.fixedconst_x.keys():
                self.relaxmod.getConstrByName('fix_x[{},{},{}]'.format(i, j, k)).rhs = \
                    round(m.variables.x[i, j, k].X, 4)
                self.fixedconst_x[i, j, k].rhs = \
                    round(m.variables.x[i, j, k].X)
            for i, j in self.fixedconst_h.keys():
                self.relaxmod.getConstrByName('fix_h[{},{}]'.format(i, j)).rhs = \
                    round(m.variables.h[i, j].X, 4)
                self.fixedconst_h[i, j].rhs = \
                    round(m.variables.h[i, j].X)
            for i in self.fixedconst_p_e.keys():
                self.relaxmod.getConstrByName('fix_p_e[{}]'.format(i)).rhs = \
                    round(m.variables.p_e[i].X, 4)
                self.fixedconst_p_e[i].rhs = \
                    round(m.variables.p_e[i].X)
            for i in self.fixedconst_p_l.keys():
                self.relaxmod.getConstrByName('fix_p_l[{}]'.format(i)).rhs = \
                    round(m.variables.p_l[i].X, 4)
                self.fixedconst_p_l[i].rhs = \
                    round(m.variables.p_l[i].X)

        else:
            for i, j, k in self.fixedconst_x.keys():
                # Update first-stage vars for LP sub-problem
                self.relaxmod.getConstrByName('fix_x[{},{},{}]'.format(i, j, k)).rhs = \
                    round(sol[0][i, j, k], 4)
                # Update first-stage vars for MIP sub-problem
                self.fixedconst_x[i, j, k].rhs = \
                    round(sol[0][i, j, k])
            for i, j in self.fixedconst_h.keys():
                self.relaxmod.getConstrByName('fix_h[{},{}]'.format(i, j)).rhs = \
                    round(sol[1][i, j], 4)
                self.fixedconst_h[i, j].rhs = \
                    round(sol[1][i, j])
            for i in self.fixedconst_p_e.keys():
                self.relaxmod.getConstrByName('fix_p_e[{}]'.format(i)).rhs = \
                    round(sol[2][i], 4)
                self.fixedconst_p_e[i].rhs = \
                    round(sol[2][i])
            for i in self.fixedconst_p_l.keys():
                self.relaxmod.getConstrByName('fix_p_l[{}]'.format(i)).rhs = \
                    round(sol[3][i], 4)
                self.fixedconst_p_l[i].rhs = \
                    round(sol[3][i])
        self.model.update()
        self.relaxmod.update()
