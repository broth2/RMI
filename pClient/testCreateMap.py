import pickle

class Cell():
    def __init__(self):
        self.visited = False
        self.paths = [-1,-1,-1,-1,-1,-1,-1,-1]
        self.coords = None



# Read the byte stream from a file or receive it from the other program
with open('pClient/serialized_object.pkl', 'rb') as file:
    serialized_object = file.read()

# Deserialize the object from the byte stream
state = pickle.loads(serialized_object)


o = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]

for line in state:
    for column in line:
        if column is None:
            continue
        print(column.paths)


# Use the deserialized object as needed

mapa = ""
for line in state:
    for column in line:
        if column is None:
            mapa = mapa + "  "
            continue
        if column.coords == (0,0):
            mapa = mapa + "I"
        else:
            mapa = mapa + " "
        if column.paths[2]==1:
            mapa = mapa + "-"
        else:
            mapa = mapa + " "
    mapa = mapa[:-1]
    if line is state[-1]:
        break
    mapa = mapa + "\n"
    prev_cell_none = False
    for column in line:
        if column is not None:
            if column.coords == (6,-4):
                flag = True
            if prev_cell_none and column.paths[5]==1:
                mapa = mapa + "/"
                continue
            if not prev_cell_none and column.paths[5]==1:
                mapa = mapa + "/"
            if prev_cell_none and column.paths[5]==0:
                mapa = mapa + " "
            if column.paths[4]==1:
                mapa = mapa + "|"
            else:

                mapa = mapa + " "
                space_added = True
            if column.paths[3]==1:
                mapa = mapa + "\\"
            else:
                if not space_added:
                    mapa = mapa + " "
            
            prev_cell_none = False
        else:
            if prev_cell_none:
                mapa = mapa + "  "
            else:
                mapa = mapa + " "
            prev_cell_none = True

    mapa = mapa + "\n"

    with open('map.txt', 'w') as file:
        print(mapa, file=file)
        #print(mapa)