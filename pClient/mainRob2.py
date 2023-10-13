import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import random

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):    
    def __init__(self, rob_name, rob_id, angles, host):
        # init system vars
        self.state = [[None] * 25 for _ in range(11)]

        # base class vars
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host) 

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("ConnectionÂ´ refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            #READ SENSOR 
            self.readSensors()
            # print measures array

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()


            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
                

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        #print Line Sensor values
        print("LineSensor", self.measures.lineSensor)


        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1,+0.1)
        elif self.measures.irSensor[left_id]> 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0,0.1)
        else:
            # matriz em que cada elemento é um objeto célula, simulado por uma lista pq sou preguiçoso xd e nao sei se é mais eficiente
            # tem 3 valores, se foi visitada, as coordenadas e uma lista de 8 valores que dizem se há caminho por pelo angulo iPI, sendo i o indice
            # exemplo para uma célula que foi visitada, nas coordenadas 13,4, com 2 caminhos descobertos:
            # [1, (13,4), [-1,-1,1,0,1,0,0,-1]]
            # se nao se souber nada sobre o ponto entao ele é 'None'
            # L =  5 - y
            # C = 12 + x 

            # check for paths

            # generic case adapt for the first one
            # TODO instead of '-1' must be 0 or 1, the values must already be known
            curr_x = 0
            curr_y = 0
            self.state[5-curr_y][12+curr_x] = [1, (curr_x,curr_y), [-1,-1,-1,-1,-1,-1,-1,-1]]
            
            # TODO adapt to the 8 surrounding cells:
            #                       x,y+1
            #            x-1,y+1             x+1,y+1
            #       x-1,y            x,y             x+1,y
            #            x-1,y-1             x+1,y-1
            #                       x,y-1
            # 


            for row in self.state:
                print(" ".join(str(cell) if cell is not None else "None" for cell in row))

            pass
        



class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()