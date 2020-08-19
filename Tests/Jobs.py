from src.Map import Map
import csv


def job(func):
    scenarios = [10, 20, 30]
    tests = [(3, 2), (4, 2), (5, 2), (6, 2), (7, 2), (9, 3), (10, 3), (11, 3), (13, 4), (15, 4), (18, 5), (20, 5), (22, 6),
         (24, 6), (25, 7), (27, 8)]

    def wrapperFunction():
        with open('./Results/{}.csv'.format(func.__name__), 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(('Rides', 'Bus', 'Scenarios', 'Objective Value', 'Savings', 'Time', 'Gap'))
        for scenario in scenarios:
            for rides, bus in tests:
                mappy = Map(rides, seed=200)
                r = func(mappy, bus, scenario)
                with open('./Results/{}.csv'.format(func.__name__), 'a', newline='') as csv_file:
                    csv_writer = csv.writer(csv_file)
                    csv_writer.writerow((rides, bus, scenario, r['Obj'], r.get('Savings', None), r['Time'], r.get('Gap',
                                                                                                                 None)))
        return True
    return wrapperFunction