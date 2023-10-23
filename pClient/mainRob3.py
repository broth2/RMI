import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import random
motion_angle = None
target_cell = None



CELLROWS=7
CELLCOLS=14


class MyRob(CRobLinkAngs):    
    start_x = None
    start_y = None
    new_x = 2
    new_y  = 0
    stack = []
    evaluation_area = False
    orientation_to_angle = {
    "E": 0,
    "NE": 45,
    "N": 90,
    "NW": 135,
    "W": 180,
    "SW": -135,
    "S": -90,
    "SE": -45,
    }
    possiblePaths = []

    orientation_rules = {
        'N': ['W', 'NW', 'N', 'NE', 'E'],
        'NE': ['NW', 'N', 'NE', 'E', 'SE'],
        'E': ['N', 'NE', 'E', 'SE', 'S'],
        'SE': ['NE', 'E', 'SE', 'S', 'SW'],
        'S': ['E', 'SE', 'S', 'SW', 'W'],
        'SW': ['SE', 'S', 'SW', 'W', 'NW'],
        'W': ['S', 'SW', 'W', 'NW', 'N'],
        'NW': ['SW', 'W', 'NW', 'N', 'NE'],

    }

    coord_sum = {
                'N': (0,2),
                'NE': (2,2),
                'E': (2,0),
                'SE': (2,-2),
                'S': (0,-2),
                'SW': (-2,-2),
                'W': (-2,0),
                'NW': (-2,2)
     }

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

    
    def myGps(self,x,y):
        return (x-self.start_x, y-self.start_y)
    
    def moveTo(self,target_x, target_y):
        print("moving to", target_x, target_y)
        print("current position", self.x, self.y)
        delta = atan2(target_y - self.y, target_x - self.x)
        print ("\ndelta\n", delta)
        alfa = self.measures.compass
        beta = delta - alfa
        self.driveMotors(0.09 - 0.03 * beta, 0.09 + 0.03 * beta)

    def sensorPosition(self, x, y):
        sx = x + 0.438 * cos(self.measures.compass)
        sy = y + 0.438 * sin(self.measures.compass)
        return sx, sy
    
    def appendInfo(self, x, y, lineSensor):
        x,y = self.sensorPosition(x,y)
        # round to whole even number to avoid floating point errors
        print("current position", self.x, self.y)
        print("SENSOR POSISION", x, y)
        #x = int(x) if ((int(x) % 2) == 0) else int(x) + 1
        #y = int(y) if ((int(y) % 2) == 0) else int(y) + 1
        self.stack.append((x, y,self.measures.compass,lineSensor))

    def possibleOrientation(self, elem):

        curr_orientation = self.get_orientation(elem[2])
        print("curr_orientation", curr_orientation)
        possible_orientations = self.orientation_rules[curr_orientation]
        print("possible orientations", possible_orientations)
        return possible_orientations

    def lineEval(self, possible_orientations, values):
        if values[2:5] == ['1', '1', '1']:
            self.possiblePaths.append(possible_orientations[2])
        if values[0:2] == ['1', '1']:
            self.possiblePaths.append(possible_orientations[0])
        if values[5:7] == ['1', '1']:
            self.possiblePaths.append(possible_orientations[4])
        if values[0:2] == ['1', '0']:
            self.possiblePaths.append(possible_orientations[1])
        if values[5:7] == ['0', '1']:
            self.possiblePaths.append(possible_orientations[3])

            
    def sumCoord(self):
        coord_sum = self.coord_sum[self.possiblePaths[1]]
        print("chosen", self.possiblePaths[1])
        self.new_x = self.x + coord_sum[0]
        self.new_y = self.y + coord_sum[1]

    def rotate_to_orientation(self, possible_paths):
        self.adjusting = True

        #get current orientation
        curr_orientation = self.measures.compass
        possible_angles = [self.orientation_to_angle[po] for po in possible_paths]
        print("possible angles:", possible_angles)  
        #calculate difference between current orientation and possible_angles[0]


        diff = possible_angles[1] - curr_orientation
        print("diff:", diff)
        #if diff between -10 and 10 print "rotate slowly"
        if abs(diff) == 0 :
            # If the orientation is within the threshold, stop rotating
            self.driveMotors(0, 0)
            self.new_orientation = True
            self.adjusting = False
            return
        else:
            if self.adjusting:
                if abs(diff) > 180:
                    if diff >= 0:
                        if diff > 70:
                            self.driveMotors(-0.05, 0.05)
                            return
                        elif diff > 30:
                            self.driveMotors(-0.03, 0.03)
                            return
                        elif diff > 10:
                            self.driveMotors(-0.005, 0.005)
                            return
                    else:
                        if diff < -70:
                            self.driveMotors(0.05, -0.05)
                            return
                        elif diff < -30:
                            self.driveMotors(0.03, -0.03)
                            return
                        elif diff < -10:
                            self.driveMotors(0.005, -0.005)
                            return
                else:
                    if diff < 0:
                        if diff < -70:
                            self.driveMotors(0.05, -0.05)
                            return
                        elif diff < -30:
                            self.driveMotors(0.03, -0.03)
                            return
                        elif diff < -10:
                            self.driveMotors(0.005, -0.005)
                            return
                    else:
                        if diff > 70:
                            self.driveMotors(-0.05, 0.05)
                            return
                        elif diff > 30:
                            self.driveMotors(-0.03, 0.03)
                            return
                        elif diff > 10:
                            self.driveMotors(-0.005, 0.005)
                            return 




    def findNewCell(self):
        while self.stack:
            elem = self.stack.pop()
            if not all(x == '0' for x in elem[3]):
                print("elem", elem)
                #eval Elem 
                possible_orientations = self.possibleOrientation(elem)
                values = elem[3]
                self.lineEval(possible_orientations, values)
                print("possible paths", self.possiblePaths)
                #calculate new coords
                self.rotate_to_orientation(self.possiblePaths)
                
                self.sumCoord()
                #print("values", values)
                self.stack = []
                self.possiblePaths.clear()
                break  # Exit the loop when the condition is met
        

    def get_orientation(self, compass):

        #print("Compass:", compass)

        if -22.5 <= compass <= 22.5:
            return "E"
        elif 22.5 < compass <= 67.5:
            return "NE"
        elif 67.5 < compass <= 112.5:
            return "N"
        elif 112.5 < compass <= 157.5:
            return "NW"
        elif -67.5 <= compass < -22.5:
            return "SE"
        elif -157.5 <= compass < -112.5:
            return "SW"
        elif -112.5 < compass <= -67.5:
            return "S"
        else:
            return "W"


    #MOVE TO -> EVALUATE -> DEAL WITH MATRIX -> REpeata

    def run(self):
        if self.status != 0:
            print("ConnectionÂ´ refused or error")
            quit()
        
        self.readSensors()
        self.start_x = self.measures.x
        self.start_y = self.measures.y


        state = 'stop'
        stopped_state = 'run'
        

        while True:
    
            self.readSensors()
                #READ SENSOR 
                # if firstRun 
            self.x,self.y =self.myGps(self.measures.x, self.measures.y)


            if abs(self.x-self.new_x) <= 0.1 and abs(self.y-self.new_y) <= 0.1:
                #adjust until perfect coordinates
                self.driveMotors(0,0)
                print("stopped")
                self.findNewCell()
            else:
                self.moveTo(self.new_x, self.new_y)
                self.appendInfo(self.x,self.y,self.measures.lineSensor)



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