from Map import Map
from Tabu import Tabu
from TwoStage import TwoStage
from L_shaped import MasterProblem as mp

rides = 5
bus = 2
scenarios = 20
MIPGap = 0.001
TimeLimit = 18000
probability = [0.7, 0.1, 0.15, 0.05]
mappy = Map(rides)

lshaped = mp(mappy, bus=bus, scenarios=scenarios, probability=probability)
tabu = Tabu(lshaped,tabu_iterations=200, tabu_status=10, subset=10, tsp=True)
tabu.tabuheuristic()