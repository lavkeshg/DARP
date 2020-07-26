from Map import Map
from Tabu import Tabu
from TwoStage import TwoStage
from L_shaped import MasterProblem as mp

rides = 5
bus = 2
scenarios = 200
MIPGap = 0.001
TimeLimit = 18000
probability = [0.7, 0.1, 0.15, 0.05]
mappy = Map(rides, seed=200)

lshaped = mp(mappy, bus=bus, scenarios=scenarios, probability=probability)
tabu = Tabu(lshaped, tabu_iterations=800, tabu_status=4,init_pool=1000, rtoptm=5, subset=20, tsp=True)
tabu.tabuheuristic()
print(tabu.best)