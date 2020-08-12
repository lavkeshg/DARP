from Map import Map
from Tabu import Tabu
from TwoStage import TwoStage
from L_shaped import MasterProblem as mp

rides = 10
bus = 2
scenarios = 10
MIPGap = 0.001
TimeLimit = 18000
probability = [0.7, 0.1, 0.15, 0.05]
mappy = Map(rides, seed=200)

lshaped = mp(mappy, bus=bus, scenarios=scenarios, probability=probability)
tabu = Tabu(lshaped, tabu_iterations=800, tabu_status=7, init_pool=200, rtoptm=5, subset=20, tsp=True, MIP=False)
tabu.tabuheuristic()
print(tabu.best)
# tabu.al = {s: tabu.model.submodel[s].sim.alpha for s in tabu.model.submodel.keys()}
# sol = tabu.initialSolution()
# print(sol)