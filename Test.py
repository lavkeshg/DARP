from src.Map import Map
from src.Tabu import Tabu
from src.L_shaped import MasterProblem as mp

rides = 5
bus = 2
scenarios = 10
MIPGap = 0.001
TimeLimit = 18000
probability = [0.7, 0.1, 0.15, 0.05]
mappy = Map(rides, seed=200)

tabu = Tabu(mappy, bus, scenarios, probability, tabu_iterations=800, tabu_status=8, init_pool=20,
            rtoptm=5, subset=20, tsp=True, MIP=False)
tabu.tabuheuristic()
print(tabu.best)
# tabu.al = {s: tabu.model.submodel[s].sim.alpha for s in tabu.model.submodel.keys()}
# sol = tabu.initialSolution()
# print(sol)