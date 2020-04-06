#!/usr/bin/env python2
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

VehiclesR042 = []  # 10 vehicles

b = 1
while b < 11:
    VehiclesR042.append("veh" + str(b))  # Appending first 10 vehicles which enter the simulation together to empty array
    b += 1

R042FuncSIR042 = 0
R042FuncSJR042 = 0
VehIR0042 = 0
VehJR042 = 0
SpeedIR0042 = 0
SpeedJR042 = 0
SpeedLeftFollowingNeighboursR042 = 0
SpeedRightFollowingNeighboursR042 = 0
SpeedLeftLeadingNeighboursR042 = 0
SpeedRightLeadingNeighboursR042 = 0
SJNR042 = 0
NextStepR042 = 0

# R042 Values
K1 = 0.623
A1 = 1.5053
B1 = 7.3634 * 10 ** -3
C1 = 7.8400 * 10 ** -7
D1 = 0
E1 = 0
F1 = 0
G1 = 0

import traci
traci.start(sumoCmd)
step = 0
with open('R042_SAS_Results.csv', mode='w', newline='') as Output:
    while step < 60:
        traci.simulationStep()
        step += 1

        # R042
        for x in range(len(VehiclesR042)):
            VehIR042 = x
            VehJR042 = x + 1
            if VehIR042 < 5:
                SpeedIR042 = 80
            else:
                SpeedIR042 = 70

            if VehJR042 < 5:
                SpeedJR042 = 80
            else:
                SpeedJR042 = 70

            R042FuncSIR042 = K1 * (
                (
                        A1 + B1 * SpeedIR042 + C1 * SpeedIR042 ** 2 + D1 * SpeedIR042 ** 3 + E1 * SpeedIR042 ** 4 + F1 * SpeedIR042 ** 5 +
                        G1 * SpeedIR042 ** 6) / SpeedIR042)

            R042FuncSJR042 = K1 * (
                (
                        A1 + B1 * SpeedJR042 + C1 * SpeedJR042 ** 2 + D1 * SpeedJR042 ** 3 + E1 * SpeedJR042 ** 4 + F1 * SpeedJR042 ** 5 +
                        G1 * SpeedJR042 ** 6) / SpeedJR042)

            LeftFollowingNeighboursR042 = traci.vehicle.getNeighbors(VehiclesR042[VehIR042],
                                                                      0b00)  # Returns ID of neighbour, distance in m from this neighbour
            if not LeftFollowingNeighboursR042:
                SpeedLeftFollowingNeighboursR042 = 0

            if LeftFollowingNeighboursR042:
                if ('veh1' or 'veh2' or 'veh3' or 'veh4' or 'veh5') in str (LeftFollowingNeighboursR042):
                    SpeedLeftFollowingNeighboursR042 = 80
                else:
                    SpeedLeftFollowingNeighboursR042 = 70

            RightFollowingNeighboursR042 = traci.vehicle.getNeighbors(VehiclesR042[VehIR042],
                                                                       0b100)  # Returns ID of neighbour, distance in m from this neighbour
            if not RightFollowingNeighboursR042:
                SpeedRightFollowingNeighboursR042 = 0

            if RightFollowingNeighboursR042:
                if ('veh1' or 'veh2' or 'veh3' or 'veh4' or 'veh5') in RightFollowingNeighboursR042:
                    SpeedRightFollowingNeighboursR042 = 80
                else:
                    SpeedRightFollowingNeighboursR042 = 70

            LeftLeadingNeighboursR042 = traci.vehicle.getNeighbors(VehiclesR042[VehIR042],
                                                                    0b010)  # Returns ID of neighbour, distance in m from this neighbour
            if not LeftLeadingNeighboursR042:
                SpeedLeftLeadingNeighboursR042 = 0

            if LeftLeadingNeighboursR042:
                if ('veh1' or 'veh2' or 'veh3' or 'veh4' or 'veh5') in LeftLeadingNeighboursR042:
                    SpeedLeftLeadingNeighboursR042 = 80
                else:
                    SpeedLeftLeadingNeighboursR042 = 70

            RightLeadingNeighboursR042 = traci.vehicle.getNeighbors(VehiclesR042[VehIR042], 0b110)  # Returns ID of neighbour, distance

            if not RightLeadingNeighboursR042:
                SpeedRightLeadingNeighboursR042 = 0

            if RightLeadingNeighboursR042:
                if ('veh1' or 'veh2' or 'veh3' or 'veh4' or 'veh5') in RightLeadingNeighboursR042:
                    SpeedRightLeadingNeighboursR042 = 80
                else:
                    SpeedRightLeadingNeighboursR042 = 70

            SJNR042 = SpeedLeftFollowingNeighboursR042 + SpeedRightFollowingNeighboursR042 + SpeedLeftLeadingNeighboursR042 + SpeedRightLeadingNeighboursR042

            QR0042 = 0.001 * (SJNR042 - SpeedIR042)

            NextStepR042 = SpeedIR042 + QR0042 - (0.01 * R042FuncSJR042)

            file_writer = csv.writer(Output)
            file_writer.writerow([step, VehIR042, SpeedIR042, LeftFollowingNeighboursR042,
                                  SpeedLeftFollowingNeighboursR042, RightFollowingNeighboursR042,
                                  SpeedRightFollowingNeighboursR042, LeftLeadingNeighboursR042,
                                  SpeedLeftLeadingNeighboursR042, RightLeadingNeighboursR042,
                                  SpeedRightLeadingNeighboursR042, NextStepR042])