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

VehiclesR001 = []  # 10 vehicles

a = 1
while a < 11:
    VehiclesR001.append("veh" + str(a))  # Appending first 10 vehicles which enter the simulation together to empty array
    a += 1

# R001 Values
K1 = 1.0
A1 = 519.82
B1 = 0.056485
C1 = 0.017898
D1 = 0
E1 = 0
F1 = 0
G1 = 0

#  Initialization of global variables
R001FiR001 = 0
VehIR001 = 0
VehJR001 = 0
SpeedIR001R001 = 0
SpeedJR001R001 = 0
SpeedLeftFollowingNeighboursR001 = 0
SpeedRightFollowingNeighboursR001 = 0
SpeedLeftLeadingNeighboursR001 = 0
SpeedRightLeadingNeighboursR001 = 0
SJNR001 = 0
NextStepR001 = 0
Newlist = []

# si = current vehicle
# sj = next vehicle
import traci

traci.start(sumoCmd)
step = 0
with open('R001_SAS_Results.csv', mode='w', newline='') as Output:
    while step < 60:
        traci.simulationStep()
        step += 1

        for x in range(len(VehiclesR001)):
            VehIR001 = x
            VehJR001 = x + 1
            if VehIR001 < 5:
                SpeedIR001 = 80
            else:
                SpeedIR001 = 70

            if VehJR001 < 5:
                SpeedJR001 = 80
            else:
                SpeedJR001 = 70

            R001FuncSIR001 = K1 * (
                (
                            A1 + B1 * SpeedIR001 + C1 * SpeedIR001 ** 2 + D1 * SpeedIR001 ** 3 + E1 * SpeedIR001 ** 4 + F1 * SpeedIR001 ** 5 +
                            G1 * SpeedIR001 ** 6) / SpeedIR001)

            R001FuncSJR001 = K1 * (
                (
                            A1 + B1 * SpeedJR001 + C1 * SpeedJR001 ** 2 + D1 * SpeedJR001 ** 3 + E1 * SpeedJR001 ** 4 + F1 * SpeedJR001 ** 5 +
                            G1 * SpeedJR001 ** 6) / SpeedJR001)

            LeftFollowingNeighboursR001 = traci.vehicle.getNeighbors(VehiclesR001[VehIR001],
                                                                      0b000)  # Returns ID of neighbour, distance in m from this neighbour
            if LeftFollowingNeighboursR001:
                if ('veh1' or 'veh2' or 'veh3' or 'veh4' or 'veh5') in str(LeftFollowingNeighboursR001):
                    SpeedLeftFollowingNeighboursR001 = 80
                else:
                    SpeedLeftFollowingNeighboursR001 = 70

            RightFollowingNeighboursR001 = traci.vehicle.getNeighbors(VehiclesR001[VehIR001],
                                                                       0b100)  # Returns ID of neighbour, distance in m from this neighbour
            if RightFollowingNeighboursR001:
                if ('veh1' or 'veh2' or 'veh3' or 'veh4' or 'veh5') in str(RightFollowingNeighboursR001):
                    SpeedRightFollowingNeighboursR001 = 80
                else:
                    SpeedRightFollowingNeighboursR001 = 70

            LeftLeadingNeighboursR001 = traci.vehicle.getNeighbors(VehiclesR001[VehIR001],
                                                                    0b010)  # Returns ID of neighbour, distance in m from this neighbour
            if LeftLeadingNeighboursR001:
                if ('veh1' or 'veh2' or 'veh3' or 'veh4' or 'veh5') in str(LeftLeadingNeighboursR001):
                    SpeedLeftLeadingNeighboursR001 = 80
                else:
                    SpeedLeftLeadingNeighboursR001 = 70

            RightLeadingNeighboursR001 = traci.vehicle.getNeighbors(VehiclesR001[VehIR001],
                                                                    0b110)  # Returns ID of neighbour, distance in m from this neighbour
            if RightLeadingNeighboursR001:
                if ('veh1' or 'veh2' or 'veh3' or 'veh4' or 'veh5') in str(RightLeadingNeighboursR001):
                    SpeedRightLeadingNeighboursR001 = 80
                else:
                    SpeedRightLeadingNeighboursR001 = 70

            SJNR001 = SpeedLeftFollowingNeighboursR001 + SpeedRightFollowingNeighboursR001 + SpeedLeftLeadingNeighboursR001 + SpeedRightLeadingNeighboursR001

            QR001 = 0.001 * (SJNR001 - SpeedIR001)

            NextStepR001 = SpeedIR001 + QR001 - (0.01 * R001FuncSJR001)

            file_writer = csv.writer(Output)
            file_writer.writerow([step, VehIR001, SpeedIR001, LeftFollowingNeighboursR001,
                                  SpeedLeftFollowingNeighboursR001, RightFollowingNeighboursR001,
                                  SpeedRightFollowingNeighboursR001, LeftLeadingNeighboursR001,
                                  SpeedLeftLeadingNeighboursR001, RightLeadingNeighboursR001,
                                  SpeedRightLeadingNeighboursR001, NextStepR001])
