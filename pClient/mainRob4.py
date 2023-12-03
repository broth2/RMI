
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
    intersect_directions = set()                #set that will hold for which path there is an intersection
    history = [''] * 20
    m = 0
    intersect = False
    stop = False


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
        #self.follow_line(self.measures.lineSensor)
 


        was_in_intersect = self.intersect 
        self.intersect = self.detect_intersection(self.measures.lineSensor)
        
        if self.intersect:
            self.driveMotorsExt(0.04,0.04)
            self.append_history(self.measures.lineSensor)
        else:
            self.follow_line(self.measures.lineSensor)
            #self.driveMotorsExt(0.07,0.07)

        if was_in_intersect and not self.intersect:
            print(f"END INTERSECTION")
            #analise    
            self.analyze()
            #clear self.history
            self.reset_history()
            self.intersect_directions.clear()
        


        print("CYCLE END \n\n")


    
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



        if '1' in lineSensor[0] or '1' in lineSensor[1] or '1' in lineSensor[5] or '1' in lineSensor[6]:
            #print(f"found intersection at ({round(self.x_predict,2)}, {round(self.y_predict,2)})")
            return True

        else:
            return False


    def analyze(self):
        print("\n ANALYZE \n")
        last_data_index =  0


        for index,entry in enumerate(self.history):
            if self.history[index] != '':
                  last_data_index = index  


        print(f"Last data index {last_data_index}\n")

        #get 3 first entries and last 2
        first_3_entries = self.history[0:4]
        last_3_entries = self.history[last_data_index-2:last_data_index+1]
        entry_after_last = self.measures.lineSensor

        print(f"entry after last {entry_after_last}\n")

        self.eval(first_3_entries,last_3_entries,entry_after_last)


    def eval(self, first_entries, last_entries, entry_after_last):

        detected_out_first_entry,detected_inside_first_entry,detected_middle_first_entry = False,False,False
        detected_out_second_entry,detected_inside_second_entry,detected_middle_second_entry = False,False,False

        detected_out_first_exit,detected_inside_first_exit,detected_middle_first_exit = False,False,False
        detected_out_second_exit,detected_inside_second_exit,detected_middle_second_exit = False,False,False 

        first_entry = first_entries[0]
        second_entry = first_entries[1]
        third_entry = first_entries[2]

        if first_entry == second_entry:
            second_entry = third_entry
            

        exit_first = last_entries[-2]
        exit_second = last_entries[-1]

        if exit_first == exit_second:
            exit_first = last_entries[-3]

        print(f"FIRST ENTRY: {first_entry}")
        print(f"SECOND ENTRY: {second_entry}")

        print(f"EXIT FIRST: {exit_first}")
        print(f"EXIT SECOND: {exit_second}")
        #check if first entry has 1's in the first or last position

        detected_out_first_entry, detected_inside_first_entry, detected_middle_first_entry = self.where_detected(first_entry)
        detected_out_second_entry, detected_inside_second_entry, detected_middle_second_entry = self.where_detected(second_entry)

        detected_out_first_exit, detected_inside_first_exit, detected_middle_first_exit = self.where_detected(exit_first)
        detected_out_second_exit, detected_inside_second_exit, detected_middle_second_exit = self.where_detected(exit_second)

        #self.print_function(detected_out_first_entry,detected_inside_first_entry,detected_middle_first_entry,detected_out_second_entry,detected_inside_second_entry,detected_middle_second_entry,True)
        #self.print_function(detected_out_first_exit,detected_inside_first_exit,detected_middle_first_exit,detected_out_second_exit,detected_inside_second_exit,detected_middle_second_exit,False)

        info_entry = [detected_out_first_entry,detected_inside_first_entry,detected_middle_first_entry,detected_out_second_entry,detected_inside_second_entry,detected_middle_second_entry]
        info_exit = [detected_out_first_exit,detected_inside_first_exit,detected_middle_first_exit,detected_out_second_exit,detected_inside_second_exit,detected_middle_second_exit]

        self.intersect_directions.add(self.handle_first(info_entry,entry_after_last))
        self.intersect_directions.add(self.handle_exit(info_exit))
        
        
        if entry_after_last[2:5] == [ '1', '1','1'] or entry_after_last[2:5] == ['0', '1', '1'] or entry_after_last[2:5] == ['1','1','0']:
            self.intersect_directions.add(self.get_facing_direction())


        print(f"INTERSECT DIRECTIONS: {self.intersect_directions}")

    
    def print_function(self, detected_out_first, detected_inside_first, detected_middle_first, detected_out_second, detected_inside_second, detected_middle_second,entry):
        if entry:
            print("\nENTRY\n")
        else:
            print("\nEXIT\n")

        print(f"DETECTED OUT FIRST: {detected_out_first}")
        print(f"DETECTED INSIDE FIRST: {detected_inside_first}")
        print(f"DETECTED MIDDLE FIRST: {detected_middle_first}\n")
    
        print(f"DETECTED OUT SECOND: {detected_out_second}")
        print(f"DETECTED INSIDE SECOND: {detected_inside_second}")
        print(f"DETECTED MIDDLE SECOND: {detected_middle_second}\n ")
            

    
    def handle_first(self, arr,entry_after_last):
        detected_out_first = arr[0]
        detected_inside_first = arr[1]
        detected_out_second = arr[3]
        detected_inside_second = arr[4]

        directions = set()
    
        
        if detected_out_first and not detected_inside_first and detected_inside_second and detected_out_second:
            direction = detected_out_first[1]
            if direction == 'left':
                directions.add(self.get_facing_direction(135))
            elif direction == 'right':
                directions.add(self.get_facing_direction(-135))
            elif direction == 'both':
                directions.add(self.get_facing_direction(-135))
                directions.add(self.get_facing_direction(135))


        if (not detected_inside_first and not detected_out_first and detected_inside_second) and detected_out_second or (detected_inside_first and detected_out_first and detected_inside_second and detected_out_second):
            direction = detected_out_second[1]
            if direction == 'left':
                directions.add(self.get_facing_direction(90))
            elif direction == 'right':
                directions.add(self.get_facing_direction(-90))
            elif direction == 'both':
                directions.add(self.get_facing_direction(-90))
                directions.add(self.get_facing_direction(90))


        return frozenset(directions)
    
    
    def handle_exit(self, arr):
        detected_out_first = arr[0]
        detected_inside_first = arr[1]
        detected_out_second = arr[3]
        detected_inside_second = arr[4]


        directions = set()

        if detected_out_second and not detected_inside_second:
            direction = detected_out_second[1]
            print(f"detected out second {detected_out_second}")
            if direction == 'left':
                directions.add(self.get_facing_direction(45))
            elif direction == 'right':
                directions.add(self.get_facing_direction(-45))
            elif direction == 'both':
                directions.add(self.get_facing_direction(-45))
                directions.add(self.get_facing_direction(45))

        if (not detected_inside_first and not detected_out_first and detected_inside_second) and detected_out_second or (detected_inside_first and detected_out_first and detected_inside_second and detected_out_second):
            direction = detected_out_first[1]
            if direction == 'left':
                directions.add(self.get_facing_direction(90))
            elif direction == 'right':
                directions.add(self.get_facing_direction(-90))
            elif direction == 'both':
                directions.add(self.get_facing_direction(-90))
                directions.add(self.get_facing_direction(90))



        return frozenset(directions)
    

    def     where_detected(self, arr):
        detected_out = False
        detected_inside = False
        detected_middle = False

        if arr[0] == '1' or arr[6] == '1':
            detected_out = True
            if arr[0] == '1':
                detected_out = (True, 'left')
            if arr[6] == '1':
                detected_out = (True, 'right')
            if arr[0] == '1' and arr[6] == '1':
                detected_out = (True, 'both')

        if arr[1] == '1' or arr[5] == '1':
            detected_inside = True
            if arr[1] == '1':
                detected_inside = (True, 'left')
            if arr[5] == '1':
                detected_inside = (True, 'right')
            if arr[1] == '1' and arr[5] == '1':
                detected_inside = (True, 'both')
        if all(element == '1' for element in arr[2:5]):
            detected_middle = True

        return detected_out, detected_inside, detected_middle   


    def get_facing_direction(self,sum = 0):

        compass = self.measures.compass + sum 

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

    

    def append_history(self, arr):
        print(f" M : {self.m}")
        self.history[self.m] = arr
        self.m += 1

        if self.m == 20:
            self.m = 0


    def handle_intersections(self):
        

        for i in range(len(self.history)):
            tuple_values = self.history[i]
            #get the index of the first entry with corner = True
            if len(tuple_values) >= 2 and tuple_values[1] is True:
                # CHECK UNTIL THE END of history if there is any tuple_values[1] == false
                print(f"I: {i} \n")
                for j in range(1, len(self.history)):
                    next_index = (i + j) % len(self.history)
                    next_entry = self.history[next_index]
                    if len(next_entry) >= 2 and next_entry[1] is False:
                        return True
                    pass
                return False


    def reset_history(self):
        self.history = ['']  * 20
        self.m = 0


    
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