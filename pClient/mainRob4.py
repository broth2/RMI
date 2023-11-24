
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import math

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    out_l = 0
    out_r = 0
    rot_predict = 0        # TODO: must be init as None and averager or smth after first readings
    x_predict = 0
    y_predict = 0

    def __init__(self, rob_name, rob_id, angles, host):
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
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
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
                    self.setVisitingLed(True)
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
            self.go()

    def go(self):
        self.follow_line(self.measures.lineSensor)
        self.detect_intersection(self.measures.lineSensor)

    
    def driveMotorsExt(self,lpow,rpow):
        #apply self.drive motors
        self.driveMotors(lpow,rpow)
        
        #apply movement model
        # print("applying movement model")
        if lpow>0.15:
            lpow = 0.15
        if rpow>0.15:
            rpow = 0.15

        self.out_l = (lpow + self.out_l)/2
        self.out_r = (rpow + self.out_r)/2

        lin = (self.out_l + self.out_r) / 2
        self.x_predict = self.x_predict + lin*math.cos(self.rot_predict)
        self.y_predict = self.y_predict + lin*math.sin(self.rot_predict)
        self.rot_predict = self.rot_predict + self.out_r - self.out_l

        print("x:",round(self.x_predict,2)," y:", round(self.y_predict,2)," t:", round(math.degrees(self.rot_predict),1), self.measures.compass)

    def follow_line(self, arr):
        first_index = -1
        last_index = -1
        for i in range(len(arr)):
            if arr[i] == '1':
                first_index = i
                break
        for i in range(len(arr)-1, -1, -1):
            if arr[i] == '1':
                last_index = i
                break

        center = (first_index + last_index) / 2

        if first_index==-1 and last_index==-1:
            print("oh nao oh deus")

        error = center - 3

        if error == 0:
            self.driveMotorsExt(0.07, 0.07)
        else:
            correction = (0.04 * error)
            left_motor_speed = 0.02 + correction
            right_motor_speed = 0.02 - correction
            self.driveMotorsExt(left_motor_speed, right_motor_speed)


    def detect_intersection(self, lineSensor):
        if '1' in lineSensor[:2]+lineSensor[-2:]:
            print(f"found intersection at ({round(self.x_predict,2)}, {round(self.y_predict,2)})")
        elif '1' not in lineSensor:
            print(f"found intersection at ({round(self.x_predict,2)}, {round(self.y_predict,2)})")

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