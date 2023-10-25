import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import random
from collections import deque

CELLROWS=7
CELLCOLS=14

class Cell():
    def __init__(self):
        self.visited = False
        self.paths = [-1,-1,-1,-1,-1,-1,-1,-1]
        self.coords = None




class MyRob(CRobLinkAngs):    
    start_x = None
    start_y = None
    visited_orientation = {}
    orientation = None
    paths = [-1,-1,-1,-1,-1,-1,-1,-1]
    possible_orientations = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    coord_sum = {'N': (0,2), 'NE': (2,2), 'E': (2,0), 'SE': (2,-2), 'S': (0,-2), 'SW': (-2,-2), 'W': (-2,0), 'NW': (-2,2)}
    has_new_coords = True
    new_x = 2
    new_y = 0
    new_orientation = False
    departure_orientation = None
    adjusting = False
    first_r = True
    distance = 0
    prev_distance = None
    arrived = True
    stack = []
    orientation_rules = {
        'N': ['W', 'NW', 'N', 'NE', 'E'],
        'NE': ['NW', 'N', 'NE', 'SE', 'E'],
        'E': ['N', 'NE', 'E', 'SE', 'S'],
        'SE': ['NE', 'E', 'SE', 'S', 'SW'],
        'S': ['E', 'SE', 'S', 'SW', 'W'],
        'SW': ['SE', 'S', 'SW', 'W', 'NW'],
        'W': ['S', 'SW', 'W', 'NW', 'N'],
        'NW': ['SW', 'W', 'NW', 'N', 'NE'],

    }
    
    possiblePaths = []

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
            
            # L =  5 - y
            # C = 12 + x 
            x,y =self.myGps(self.measures.x, self.measures.y)
            #print(x,y, self.measures.compass)
            #print("\nSELF.NEW COORDS", self.has_new_coords)
            if self.has_new_coords:
                #if abs(round(x,1)-self.new_x) <= 0.1 and abs(round(y,1)-self.new_y) <= 0.1:
                #if round(x)>=self.new_x and round(y)>=self.new_y:
                self.distance = ((x-self.new_x)**2 + (y-self.new_y)**2)**0.5
                print("distance", self.distance)
                
                if abs(x-self.new_x) <= 0.1 and abs(y-self.new_y) <= 0.1:
                    #adjust until perfect coordinates
                    self.driveMotors(0,0)
                    print("stopped")
                    self.arrived = True
                else:
                    self.arrived = False
                self.prev_distance = self.distance
                if self.arrived:
                    self.driveMotors(0,0)
                    print("(",x,"-",y,")", "-","(",self.new_x,"-", self.new_y,")")
                    self.has_new_coords = False
                    self.distance = None
                    print("stack", self.stack)
                    self.prev_distance = None
                    self.get_orientation()
                    self.departure_orientation = self.orientation
                else:
                    self.driveMotors(0.05,0.05)
                    print("A MEXER ME")
                    
                    #self.moveTo(self.new_x, self.new_y,x,y)
                    self.appendInfo(x,y,self.measures.lineSensor)
                    #self.follow_line(self.measures.lineSensor)

            self.get_orientation()
            coord = (x,y)
            #visited_orientations_coord = self.get_visited_orientations(coord)
            #print("visited orientations:", visited_orientations_coord)
            #print(coord, "-", coord == (self.new_x, self.new_y))
            #if abs(round(x,1)-self.new_x) <= 0.1 and abs(round(y,1)-self.new_y) <= 0.1:
            print("aqui else after move")
            if self.arrived:
                print("arrive")
                coord = (self.new_x, self.new_y)
                while self.stack:
                    elem = self.stack.pop()
                    if not all(x == '0' for x in elem[3]):
                        print("elem", elem)
                        #eval Elem 
                        possible_orientations = self.possibleOrientation(elem)
                        values = elem[3]
                        self.lineEval(possible_orientations, values)
                        print("possible paths", self.possiblePaths)

                        self.paths = self.find_indices(self.possiblePaths)
                        print("self.paths", self.paths)
                        self.stack = []
                        self.possiblePaths.clear()

                                                
                for i in range(len(self.paths)):
                    if self.paths[i] == -1:
                        self.paths[i] = 0

                x = int(round(x)/2)
                y = int(round(y)/2)

                # fill state matriz with info
                if self.state[5-y][12+x] is None:
                    cell = Cell()
                    cell.visited = True
                    cell.coords = coord
                    cell.paths = self.paths
                    self.state[5-y][12+x] = cell
                    self.create_neighbours(cell)
                else:
                    if not self.state[5-y][12+x].visited:
                        self.state[5-y][12+x].visited = True
                        self.state[5-y][12+x].paths = self.paths
                        self.create_neighbours(self.state[5-y][12+x])
                        # TODO something when cell has been visited?

                # Determine an unexplored path
                if not self.has_new_coords:
                    
                    possible_paths = [i for i, path in enumerate(self.state[5-y][12+x].paths) if path == 1]
                    print("possible paths33333:", possible_paths)
                    # use possible path indexs to get the orientation
                    print("possible orientations:", [self.possible_orientations[i] for i in possible_paths])
                    destination = self.choose_path(self.departure_orientation, self.state[5-y][12+x])
                    print("destination", destination)
                    self.rotate_to_orientation(destination)

                if self.new_orientation:
                    self.new_orientation = False
                    specific_coord_sum = self.coord_sum[self.orientation]
                    self.new_x = round(self.state[5-y][12+x].coords[0] + specific_coord_sum[0])
                    self.new_y = round(self.state[5-y][12+x].coords[1] + specific_coord_sum[1])
                    self.has_new_coords = True
                    self.shit()
                    print("puta belha")
                    print("new_x:", self.new_x, "new_y:", self.new_y)


    def find_indices(self,possible_paths):
        to_index = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        indices = [0] * len(to_index)
        
        for path in possible_paths:
            if path in to_index:
                index = to_index.index(path)
                indices[index] = 1

        
        return indices




    def possibleOrientation(self, elem):

        curr_orientation = self.get_compass(elem[2])
        print("curr_orientation", curr_orientation)
        possible_orientations = self.orientation_rules[curr_orientation]
        print("possible orientations", possible_orientations)
        return possible_orientations

    def shit(self):
        for line in self.state:
                for column in line:
                        if column is None:
                                print(0, end=" ")
                        else:
                                print(1, end=" ")
                print()
    
    def sensorPosition(self, x, y):
        sx = x + 0.438 * cos(self.measures.compass)
        sy = y + 0.438 * sin(self.measures.compass)
        return sx, sy

    def appendInfo(self, x, y, lineSensor):
        x,y = self.sensorPosition(x,y)
        # round to whole even number to avoid floating point errors

        print("SENSOR POSISION", x, y)
        #x = int(x) if ((int(x) % 2) == 0) else int(x) + 1
        #y = int(y) if ((int(y) % 2) == 0) else int(y) + 1
        self.stack.append((x, y,self.measures.compass,lineSensor))

    def get_antipodal(self, coord):
        # transforms a cardinal point like "N" or "SW" to its antipodal 180 degrees away
        index = self.possible_orientations.index(coord)
        return self.possible_orientations[(index + 4) % 8]
    
    
    def create_neighbours(self, cell):
        # fills state with information on surrounding cells
        print("creating neighbours for", cell.coords)
        indices_of_paths = [i for i, x in enumerate(cell.paths) if x == 1]
        orientation_of_paths = [self.possible_orientations[i] for i in indices_of_paths]
        destination_of_paths = [(self.coord_sum[i][0] + cell.coords[0], self.coord_sum[i][1] + cell.coords[1])for i in orientation_of_paths]
        antipodal_of_paths = [self.get_antipodal(crdnt) for crdnt in orientation_of_paths]
        antipodal_index_of_paths = [self.possible_orientations.index(i) for i in antipodal_of_paths]
        for i in range(len(destination_of_paths)):
            if self.state[5-int(destination_of_paths[i][1]/2)][12+int(destination_of_paths[i][0]/2)] is None:
                cell = Cell()
                cell.coords = destination_of_paths[i]
                cell.paths[antipodal_index_of_paths[i]] = 1
                self.state[5-int(destination_of_paths[i][1]/2)][12+int(destination_of_paths[i][0]/2)] = cell
            else:
                # if self.state[5-int(destination_of_paths[i][1]/2)][12+int(destination_of_paths[i][0]/2)].visited == True:
                #     return
                if self.state[5-int(destination_of_paths[i][1]/2)][12+int(destination_of_paths[i][0]/2)].paths[antipodal_index_of_paths[i]] == -1:
                    self.state[5-int(destination_of_paths[i][1]/2)][12+int(destination_of_paths[i][0]/2)].paths[antipodal_index_of_paths[i]] = 1
                elif self.state[5-int(destination_of_paths[i][1]/2)][12+int(destination_of_paths[i][0]/2)].paths[antipodal_index_of_paths[i]] == 0:
                    print("no changes due to info error on", self.state[5-int(destination_of_paths[i][1]/2)][12+int(destination_of_paths[i][0]/2)].coords, "path", antipodal_of_paths[i])
                if self.state[5-int(destination_of_paths[i][1]/2)][12+int(destination_of_paths[i][0]/2)].coords != destination_of_paths[i]:
                    print("info error, entry is", self.state[5-int(destination_of_paths[i][1]/2)][12+int(destination_of_paths[i][0]/2)].coords, "but expected it to be", destination_of_paths[i])

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

    def choose_path(self, departure_ornt, cell):
        indices_of_paths = [i for i, x in enumerate(cell.paths) if x == 1]                       # 1, 0, 1, 1
        orientation_of_paths = [self.possible_orientations[i] for i in indices_of_paths]    # N, S, E, W
        print("orientation_of_paths", orientation_of_paths)
        """ if departure_ornt is not None:
            print("entrei")
            print("departure_ornt",departure_ornt)
            return_path = self.get_antipodal(departure_ornt)
            print("return_path", return_path)
            orientation_of_paths.remove(return_path)
            if not orientation_of_paths:
                return return_path """
        destination_of_paths = [(self.coord_sum[i][0] + cell.coords[0], self.coord_sum[i][1] + cell.coords[1])for i in orientation_of_paths]
        for i in range(len(destination_of_paths)):
            if not self.state[5-int(destination_of_paths[i][1]/2)][12+int(destination_of_paths[i][0]/2)].visited:
                if -1 not in self.state[5-int(destination_of_paths[i][1]/2)][12+int(destination_of_paths[i][0]/2)].paths:
                    continue
                return orientation_of_paths[i]
        # TODO percorrer a matriz state toda, encontrar o ponto nao visitado mais proximo e retorna-lo, ja que nao é nenhum dos 8 envolventes
        self.closest_cell(cell)     # TODO this should return an orientation
        return #final proper print state

    
    def closest_cell(self, curr_cell):
        visited_cells = set()
        visited_cells.add(curr_cell)
        queue = deque([(curr_cell,0,[])])

        while queue:
            actual, moves, parents = queue.popleft()
            if not actual.visited:
                return parents[0]   #should be an orientation
            
            neighbours = self.get_neighbours(actual)

            for neighbour in neighbours:
                if not neighbour.visited:
                    return parents[0]   #should be an orientation
                if neighbour not in visited_cells:
                    parents.append[neighbour]
                    queue.append((neighbour, moves+1, parents))
                    visited_cells.add(neighbour)
        return -1 # no more unvisited nodes

    def get_neighbours(self, cell):
        neighbors = []
        
        indices_of_paths = [i for i, x in enumerate(cell.paths) if x == 1]
        orientation_of_paths = [self.possible_orientations[i] for i in indices_of_paths]
        destination_of_paths = [(self.coord_sum[i][0] + cell.coords[0], self.coord_sum[i][1] + cell.coords[1])for i in orientation_of_paths]
        for new_x, new_y in destination_of_paths:
            neighbors.append(self.state[5-int(new_y/2)][12+int(new_x/2)])

        return neighbors

    def rotate_to_orientation(self, destination):
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
        curr_angle = self.measures.compass
        destination_angle = orientation_to_angle[destination]
        #print("possible angles:", possible_angles)  
        #calculate difference between current orientation and possible_angles[0]
        diff = destination_angle - curr_angle
        #print("diff:", diff)
        #if diff between -10 and 10 print "rotate slowly"
        print("diff:", diff)
        if abs(diff) ==0 :
            # If the orientation is within the threshold, stop rotating
            self.driveMotors(0, 0)
            self.new_orientation = True
            self.adjusting = False
            print("new orientation")
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
                        print(3)
                        if diff > 70:
                            print(4)
                            self.driveMotors(-0.05, 0.05)
                            return
                        elif diff > 30:
                            print(5)
                            self.driveMotors(-0.03, 0.03)
                            return
                        elif diff > 10:
                            print(6)
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
    
    def get_compass(self, compass):

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
        if values == ['0', '0', '0', '1', '1', '0', '0'] or values == ['0', '0', '1', '1', '0', '0', '0']:
            self.possiblePaths.append(possible_orientations[2])



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