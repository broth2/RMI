import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    last_sensor = None
    last_10_sensor = [0,0,0,0,0,0,0,0,0,0]
    m = 0
    curr = None
    prev = None
    curr_err = None
    prev_err = None
    
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
                
    def store_sensor_values(self, arr):
        self.last_10_sensor[self.m] = arr
        self.m += 1
        if self.m == 10:
            self.m = 0

    def get_history_eval(self, orientation):
        #print("Last 10 sensor values", self.last_10_sensor)
        left = 0
        right = 0
        for i in self.last_10_sensor:
            arr = i
            leftmost = arr[0:2]
            rightmost = arr[5:7]
            if leftmost == ['0','1'] or leftmost == ['1','1']:
                 left += 1
            if rightmost == ['1','0'] or rightmost == ['1','1']:
                right += 1

        if left < right and orientation=="right":
            self.driveMotors(0.1,0)
            #print("execut: 90 degree curve to the right")
        elif left > right and orientation=="left":
            self.driveMotors(0,0.1)
            #print("execut: 90 degree curve to the left")
        elif left==right:
            if orientation=="right":
                self.driveMotors(0.1,0)
                print("90 degree curve to the right without integral")
            elif orientation=="left":
                self.driveMotors(0,0.1)
                print("90 degree curve to the left without integral")
        else:
            if left<right:
                self.driveMotors(0.1,0)
                print("90 degree curve to the right without proportional")
            elif right<left:
                self.driveMotors(0,0.1)
                print("90 degree curve to the left without proportional")
            else:
                self.driveMotors(0.1,0.1)
                print("going forward without proportional")

    
    def detect90(self):
        if not self.curr:
            self.prev = []
            self.curr = self.measures.lineSensor
            return
        self.prev = self.curr
        self.curr = self.measures.lineSensor

        if self.prev==['0','0', '1','1','1', '1','1'] and self.curr== ['0','0','0','1','1','1','1']:
            print("\ndetect: 90 degree curve to the right")
            self.driveMotors(0,0)
            self.get_history_eval("right")

        if self.prev==['1','1', '1','1','1', '0','0'] and self.curr== ['1','1','1','1','0','0','0']:
            print("\nndetect: 90 degree curve to the left")
            self.driveMotors(0,0)
            self.get_history_eval("left")


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
            # Calculate error
            self.store_sensor_values(self.measures.lineSensor)
            error = self.calculate_error(self.measures.lineSensor)
            self.detect90()
            print("Error", error, "\n")

            # If the error is zero, follow the line
            if error == 0:
                self.driveMotors(0.1, 0.1)
                
            else:
                if error > 1 or error < -1:
                    #print("BIG ERROR")
                    self.driveMotors(0.0, 0.0)
                    self.driveMotors(0.0, 0.0)
                # Adjust robot's direction based on error
                correction = (0.12 * error)/3  # Adjust this factor as needed
                left_motor_speed = 0.03 + correction
                right_motor_speed = 0.03 - correction

                self.driveMotors(left_motor_speed, right_motor_speed)
        

    def calculate_error(self, arr):
        #in this function the arr will be a list of 7 elements, being the 2 first elements represent the leftmost sensor and the 2 last elements represent the rightmost sensor
        #and the 3 middle elements represent the front sensors
        #if the arr is like [0,0,1,1,1,0,0] the error will be 0, if the group of ones deviate from that position the error will be different from 0
        # if the group of ones are more towards the left sensor, the error will be negative, if the ones are more towards the right sensor, the error will be positive
        # if the group of ones are more towards the center, the error will be CLOSER to 0


        #first we need to find the first and last index of the group of ones
        first_index = -1
        last_index = -1
        for i in range(len(arr)):
            #print("i", arr[i])
            if arr[i] == '1':
                first_index = i
                #print("First index", first_index)
                break
        for i in range(len(arr)-1, -1, -1):
            if arr[i] == '1':
                last_index = i
                #print("Last index", last_index)
                break

        #now we need to find the center of the group of ones
        center = (first_index + last_index) / 2
        print("Center", center)

        if first_index==-1 and last_index==-1:
            #return ultimo movimento
            print("ALL ZERO CASE")
            print("prev error:", self.prev_err, "curr error:", self.curr_err)
            #self.driveMotors(0,0)
            #return -1
            #center = 1.5
            return self.curr_err
        
        #now we need to find the error
        error = center - 3

        if self.curr_err is None:
            self.curr_err = error
        self.prev_err = self.curr_err
        self.curr_err = error
        
        return error




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