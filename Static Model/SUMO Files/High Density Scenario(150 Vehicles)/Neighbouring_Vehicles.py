import os
import sys
import csv

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

sumoBinary = "/home/zen/sumo/bin/sumo-gui"
sumoCmd = [sumoBinary, "-c", "osm.sumocfg"]

Vehicles = []  # Empty List
i = 1
while i < 11:
    Vehicles.append("veh" + str(i))  # Appending first 10 vehicles which enter the simulation together to empty array
    i += 1

import traci
traci.start(sumoCmd)
step = 0
with open('HD_Neighbouring_Vehicles_Results.csv', mode='w', newline='') as Output:
    while step < 60:
        traci.simulationStep()
        step += 1

        for x in range(len(Vehicles)):
            VehI = x        
            VehJ = x + 1

            LeftFollowingNeighbours = list(traci.vehicle.getNeighbors(Vehicles[VehI],
                                                                      0b000))  # Returns ID of neighbour, distance in m from this neighbour


            RightFollowingNeighbours = list(traci.vehicle.getNeighbors(Vehicles[VehI],
                                                                       0b100))  # Returns ID of neighbour, distance in m from this neighbour


            LeftLeadingNeighbours = list(traci.vehicle.getNeighbors(Vehicles[VehI],
                                                                    0b010))  # Returns ID of neighbour, distance in m from this neighbour

            RightLeadingNeighbours = list(traci.vehicle.getNeighbors(Vehicles[VehI],
                                                                     0b110))  # Returns ID of neighbour, distance in m from this neighbour

            file_writer = csv.writer(Output)
            file_writer.writerow(
                [step, VehI, LeftFollowingNeighbours, RightFollowingNeighbours, LeftLeadingNeighbours, RightLeadingNeighbours])
