import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import random

CELLROWS=7
CELLCOLS=14

class Cell():
    visited = False
    paths = [-1,-1,-1,-1,-1,-1,-1,-1]
    coords = None




class MyRob(CRobLinkAngs):    
    start_x = None
    start_y = None
    visited_orientation = {}
    orientation = None
    paths = [-1,-1,-1,-1,-1,-1,-1,-1]
    possible_orientations = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    has_new_coords = False
    new_x = 0
    new_y = 0
    new_orientation = False
    adjusting = False
    first_r = True
    distance = 0
    prev_distance = None
    arrived = True

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
        self.readSensors()
        print("Initial measures")
        print(self.measures.x, self.measures.y)
        self.start_x = self.measures.x
        self.start_y = self.measures.y
        self.visited_orientation[(0,0)] = {'visited': set()}

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
                #self.wander()
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
        
        north_id = 0
        north_east_id = 1
        east_id = 2
        south_east_id = 3
        south_id = 4
        south_west_id = 5
        west_id = 6
        north_west_id = 7
        

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
            # [1, (13,4), [-1,-1,1,0,1,0,0,-1]] - [N,NE,E,SE,S,SW,W,NW]
            # visited , coords, 8 values list que dizem se há caminho por cada angulo
            # se nao se souber nada sobre o ponto entao ele é 'None'
            # L =  5 - y
            # C = 12 + x 
            x,y =self.myGps(self.measures.x, self.measures.y)
            #print(x,y, self.measures.compass)
            #print("\nSELF.NEW COORDS", self.has_new_coords)
            if self.has_new_coords:
                #if abs(round(x,1)-self.new_x) <= 0.1 and abs(round(y,1)-self.new_y) <= 0.1:
                #if round(x)>=self.new_x and round(y)>=self.new_y:
                self.distance = ((x-self.new_x)**2 + (y-self.new_y)**2)**0.5
                print("distance", self.distance, "prev", self.prev_distance)
                if self.prev_distance is None: self.prev_distance = self.distance
                if self.distance > self.prev_distance:
                    self.arrived = True
                    print("arrived")
                else:
                    self.arrived = False
                self.prev_distance = self.distance
                if self.arrived:
                    self.driveMotors(0,0)
                    print("-",x, self.new_x, y, self.new_y)
                    print("stopped")
                    self.has_new_coords = False
                    self.distance = None
                    self.prev_distance = None
                else:
                    #self.driveMotors(0.04,0.04)
                    self.follow_line(self.measures.lineSensor)

            self.get_orientation()
            coord = (x,y)
            #visited_orientations_coord = self.get_visited_orientations(coord)
            #print("visited orientations:", visited_orientations_coord)
            #print(coord, "-", coord == (self.new_x, self.new_y))
            #if abs(round(x,1)-self.new_x) <= 0.1 and abs(round(y,1)-self.new_y) <= 0.1:
 
            if self.arrived:
                coord = (self.new_x, self.new_y)
                #---------ADDING ORIENTATION VISITED-----------
                #get list of keys coord from visited_orientation
                if coord not in self.visited_orientation.keys():
                    self.paths = [0,0,0,0,0,0,0,0]
                    self.visited_orientation[coord] = {'visited': set()}
                    self.first_r = True
                
                self.visited_orientation[coord]['visited'].add(self.orientation)
                #------------------------------------------
        
                #check length of visited_orientation[coord]['visited'] 
                #if length is 8, then all orientations have been visited
                #         
                if len(self.visited_orientation[coord]['visited']) != 8:
                    if self.first_r and len(self.visited_orientation[coord]['visited']) == 2:
                        for elem in self.visited_orientation[coord]['visited']:
                            if self.orientation != elem:
                                self.visited_orientation[coord]['visited'].remove(elem)
                                break
                        self.first_r = False
                    #---------CHECK IF THERE IS A PATH ------------
                    #check if there is a path in the current orientation using the line sensor
                    line_sensor = self.measures.lineSensor
                    #print("LineSensor", line_sensor)    
    
                    if line_sensor[2:5] == ['1','1','1'] or line_sensor[2:5] == ['0','1','1'] or line_sensor[2:5] == ['1','1','0']:
                        #print("Path in orientation", self.orientation)
                        #switch case to get the orientation
                        match self.orientation:
                            case "N":
                                self.paths[north_id] = 1
                            case "NE":
                                self.paths[north_east_id] = 1
                            case "E":
                                self.paths[east_id] = 1
                            case "SE":
                                self.paths[south_east_id] = 1
                            case "S":
                                self.paths[south_id] = 1
                            case "SW":
                                self.paths[south_west_id] = 1
                            case "W":
                                self.paths[west_id] = 1
                            case "NW":
                                self.paths[north_west_id] = 1
                            case _:
                                print("Orientation not found")

                    self.driveMotors(0.04,-0.04)


                #proceed to new coordinates
                else:
                    for i in range(len(self.paths)):
                        if self.paths[i] == -1:
                            self.paths[i] = 0
                    #print(self.paths)
                    #print("all orientations visited")


                    #print("visited orientations:", self.visited_orientation)
                    #print("X:",x, " Y:",y)
                    x = int(round(x)/2)
                    y = int(round(y)/2)
                    # generic case adapt for the first one
                    # TODO instead of '-1' must be 0 or 1, the values must already be known
                    #curr_x = 0
                    #curr_y = 0

                    ## run the following code only one time for each cell
                    if self.state[5-y][12+x] is None:
                        cell = Cell()
                        cell.visited = True
                        cell.coords = coord
                        cell.paths = self.paths
                        self.state[5-y][12+x] = cell
                    #print("state: visited: ", self.state[5-y][12+x].visited, "coords: ", self.state[5-y][12+x].coords, "paths: ", self.state[5-y][12+x].paths)
                    
                    # TODO adapt to the 8 surrounding cells:
                    #                       x,y+1
                    #            x-1,y+1             x+1,y+1
                    #            x-1,y      x,y      x+1,y
                    #            x-1,y-1             x+1,y-1
                    #                       x,y-1
                    # 

                    # Determine an unexplored path
                    if not self.has_new_coords:
                        possible_paths = [i for i, path in enumerate(self.state[5-y][12+x].paths) if path == 1]
                        # use possible path indexs to get the orientation
                        print("possible orientations:", [self.possible_orientations[i] for i in possible_paths])
                        #rotate to one of the  possible orientations
                        self.rotate_to_orientation([self.possible_orientations[i] for i in possible_paths])
                    # dict to make the sum of the orientation and the coordinates
                    if self.new_orientation:
                        self.new_orientation = False
                        coord_sum = {'N': (0,2), 'NE': (2,2), 'E': (2,0), 'SE': (2,-2), 'S': (0,-2), 'SW': (-2,-2), 'W': (-2,0), 'NW': (-2,2)}
                        #print(coord_sum)
                        # get the sum of the orientation and the actual coordinates in a tuple
                        coord_sum = coord_sum[self.orientation]
                        #print(coord_sum)
                        # get the new coordinates
                        self.new_x = round(self.state[5-y][12+x].coords[0] + coord_sum[0])
                        self.new_y = round(self.state[5-y][12+x].coords[1] + coord_sum[1])
                        self.has_new_coords = True
                        print("puta belha")
                        print(self.state[5-y][12+x].coords[0],self.state[5-y][12+x].coords[1],"---",coord_sum[0],coord_sum[1])
                        print("new_x:", self.new_x, "new_y:", self.new_y)
                        #while self.x and self.y are not the new coordinates self.driveMotors(0.01,0.01)



                        # prestar atenção à direcao previa para eliminar essa das possiveis, por ex, se veio de este, nao pode ir para oeste
                        #for row in self.state:
                        #    print(" ".join(str(cell) if cell is not None else "None" for cell in row))

                #pass

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
            self.driveMotors(0.08, 0.08)
        else:
            if error > 1 or error < -1:
                self.driveMotors(0.0, 0.0)
                self.driveMotors(0.0, 0.0)
            correction = (0.05 * error)
            left_motor_speed = 0.02 + correction
            right_motor_speed = 0.02 - correction
            self.driveMotors(left_motor_speed, right_motor_speed)
        
        return


    def rotate_to_orientation(self, possible_paths):
        self.adjusting = True
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
        #get current orientation
        curr_orientation = self.measures.compass
        possible_angles = [orientation_to_angle[po] for po in possible_paths]
        print("possible angles:", possible_angles)  
        #calculate difference between current orientation and possible_angles[0]
        diff = possible_angles[0] - curr_orientation
        #print("diff:", diff)
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


    def myGps(self,x,y):
        #print mygps with 2 decimal places
        #print("myX:",round(x-self.start_x,2), " myY:",round(y-self.start_y,2),"\n")
        #print("newX:",self.new_x, " newY:",self.new_y,"\n")
        return (x-self.start_x, y-self.start_y)
            
    def get_orientation(self):

        compass = self.measures.compass
        #print("Compass:", compass)
        
        if -22.5 <= compass <= 22.5:
            self.orientation = "E"
        elif 22.5 < compass <= 67.5:
            self.orientation = "NE"
        elif 67.5 < compass <= 112.5:
            self.orientation = "N"
        elif 112.5 < compass <= 157.5:
            self.orientation = "NW"
        elif -67.5 <= compass < -22.5:
            self.orientation = "SE"
        elif -157.5 <= compass < -112.5:
            self.orientation = "SW"
        elif -112.5 < compass <= -67.5:
            self.orientation = "S"
        else:
            self.orientation = "W"
        




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