from src.Map import Map
from src.Tabu import Tabu
from src.TwoStage import TwoStage
from src.L_shaped import MasterProblem as mp
from Tests.Jobs import job
import time, sys


# **Modelling Parameters**

MIPGap = 0.001
TimeLimit = 18000
probability = [0.7, 0.1, 0.15, 0.05]
# **Defining the Map**


@job
def darp(mappy, bus, scenarios):
    t1 = time.time()
    mod = mp(mappy, bus=bus, scenarios=scenarios, probability=probability)
    mod.initialize()
    mod.setMIP()
    mod.model.params.OutputFlag = 0
    mod.variables.th.obj = 0
    # if aux:
    #     mod.setLowerBound()
    #     for s in range(mod.scenarios):
    #         mod.submodel[s].model.params.MIPGap = MIPGap
    # #             mod.submodel[s].model.params.OutputFlag = 1
    # else:
    mod.model.params.TimeLimit = TimeLimit
    mod.model.optimize()
    t1 = time.time() - t1
    return {'Obj': mod.model.ObjVal, 'Gap': mod.model.MIPGap, 'Time': t1}


@job
def twoStage(mappy, bus, scenarios, model=None):
    t2 = time.time()
    mod = TwoStage(mappy, bus=bus, scenarios=scenarios, probability=probability)
    mod.model.params.TimeLimit = TimeLimit
    if model is not None:
        for i, j, k in mod.MP.variables.x.keys():
            mod.MP.variables.x[i, j, k].start = model.variables.x[i, j, k].X
    mod.model.params.OutputFlag = 0
    mod.optimize()

    master = sum(mod.MP.variables.x[i, j, k].X * mod.parameters.distance[i, j]
                 + mod.MP.variables.h[i, j].X
                 # * mod.MP.variables.w[i]
                 for i, j in mod.parameters.edges for k in range(mod.MP.bus)) + \
             sum(mod.MP.variables.p_l[i].X for i in mod.parameters.pickup)

    savings = (sum((1 / mod.scenarios) * (mod.parameters.distance[i, j] * (mod.variables.xs[i, j, k, s].X -
                                                                               mod.MP.variables.x[i, j, k].X)
                                            + (mod.variables.hs[i, j, s].X - mod.MP.variables.h[i, j].X))
                   for i, j in mod.parameters.edges
                   for k in range(mod.MP.bus) for s in mod.parameters.xi) +
               sum((1 / mod.scenarios) * (mod.variables.p_ls[i, s].X - mod.MP.variables.p_l[i].X)
                   for i in mod.parameters.pickup for s in mod.parameters.xi))

    t2 = time.time() - t2
    return {'Obj': mod.model.ObjVal, 'Gap': mod.model.MIPGap, 'Time': t2, 'Savings': savings}


@job
def lShaped(mappy, bus, scenarios):
    t3 = time.time()
    mod = mp(mappy, bus=bus, scenarios=scenarios, probability=probability)
    mod.initialize()
    mod.model.params.TimeLimit = TimeLimit
    mod.model.params.MIPGap = MIPGap
    mod.model.params.OutputFlag = 0
    #     if mod.model.runtime < TimeLimit:
    #         mod.model.params.MIPGap
    mod.optimize(skip_init=False)
    savings = sum((1 / scenarios) * mod.submodel[i].model.ObjVal for i in mod.submodel.keys())
    t3 = time.time() - t3
    return {'Obj': mod.model.ObjVal, 'Gap': mod.model.MIPGap, 'Time': t3, 'Savings': savings}


@job
def tabu(mappy, bus, scenarios):
    t4 = time.time()
    tabu.TimeLimit = TimeLimit
    mod = Tabu(mappy, bus, scenarios, probability, tabu_iterations=800, tabu_status=mappy.N_riders + 2,
               rtoptm=5, subset=20, tsp=True)
    mod.tabuheuristic()
    t4 = time.time() - t4
    return {'Obj': mod.best[-1], 'Gap': None, 'Time': t4, 'Savings': mod.best[-3]}


@job
def no_lShaped(mappy, bus, scenarios):
    t3 = time.time()
    mod = mp(mappy, bus=bus, scenarios=scenarios, probability=probability)
    mod.initialize()
    mod.model.params.OutputFlag = 0
    mod.model.params.TimeLimit = TimeLimit
    mod.optimize(skip_init=True)
    savings = sum((1 / scenarios) * mod.submodel[i].model.ObjVal for i in mod.submodel.keys())
    t3 = time.time() - t3
    return {'Obj': mod.model.ObjVal, 'Gap': mod.model.MIPGap, 'Time': t3, 'Savings': savings}


