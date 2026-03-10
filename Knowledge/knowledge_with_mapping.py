from controller import Robot, DistanceSensor, Motor, Camera, Gyro, Accelerometer, Speaker, LightSensor
from typing import Literal
from math import pi, pow, isnan, sin, cos, floor
from random import getrandbits

RIGHT_17 = 0
RIGHT_46 = 1
RIGHT_90 = 2
RIGHT_150 = 3
LEFT_150 = 4
LEFT_90 = 5
LEFT_46 = 6
LEFT_17 = 7

###############
# AIDANS SHIT
###############
NORTH = "NORTH"
SOUTH = "SOUTH"
EAST = "EAST"
WEST = "WEST"


class GridData:
    def __init__(self, row=None, col=None):
        self.row = row  # int
        self.col = col  # int
        self.north = True
        self.east = True
        self.south = True
        self.west = True
        self.type = None  # GridData

    def __str__(self):
        output = f"{(self.row, self.col)} Walls: \nN = {self.north}\nE = {self.east}\nS = {self.south}\nW = {self.west}"
        return output

    def walls_to_bits(self):
        output = ["0", "0", "0", "0"]
        if self.north:
            output[0] = "1"

        if self.east:
            output[1] = "1"

        if self.south:
            output[2] = "1"

        if self.west:
            output[3] = "1"
        return "".join(output)


class Maze:
    def __init__(self):
        self.grid = []  # grid for maze is organized by columns/rows
        self.root = GridData(0, 0)
        self.root.type = "START"
        self.current = self.root
        self.graph = None

        # store all grid data in a dictionary
        # grids are stored by their key as a tuple (row, col)
        self.cache = {}

        # keep track of grid bounds for printing out the matrix
        self.max_col = 0
        self.min_col = 0
        self.max_row = 0
        self.min_row = 0

    def _add_neighbor(self, direction):
        # new grid object to add information to
        new_grid = GridData()
        match direction:
            case "NORTH":
                row_update = self.current.row + 1
                if row_update > self.max_row:
                    self.max_row = row_update

                new_grid.row = row_update
                new_grid.col = self.current.col
                new_grid.south = False

            case "SOUTH":
                row_update = self.current.row - 1
                if row_update < self.min_row:
                    self.min_row = row_update

                new_grid.row = row_update
                new_grid.col = self.current.col
                new_grid.north = False

            case "EAST":
                col_update = self.current.col + 1
                if col_update > self.max_col:
                    self.max_col = col_update

                new_grid.col = col_update
                new_grid.row = self.current.row
                new_grid.west = False

            case "WEST":
                col_update = self.current.col - 1
                if col_update < self.min_col:
                    self.min_col = col_update

                new_grid.col = col_update
                new_grid.row = self.current.row
                new_grid.east = False

        # add new grid to the structure, then update the current grid
        self.current.next = new_grid
        self.cache[(self.current.row, self.current.col)] = self.current
        self.current = new_grid

    def add_node(self, direction):
        cur_row = self.current.row
        cur_col = self.current.col
        match direction:
            case "NORTH":
                self.current.north = False
                if self.cache.get((cur_row + 1, cur_col), False):
                    # handle if a grid is found
                    self.cache[cur_row + 1, cur_col].south = False
                    self.current = self.cache[cur_row + 1, cur_col]
                else:
                    self._add_neighbor(direction)

            case "SOUTH":
                self.current.south = False
                if self.cache.get((cur_row - 1, cur_col), False):
                    # handle if a grid is found
                    self.cache[cur_row - 1, cur_col].north = False
                    self.current = self.cache[cur_row - 1, cur_col]
                else:
                    self._add_neighbor(direction)

            case "EAST":
                self.current.east = False
                if self.cache.get((cur_row, cur_col + 1), False):
                    # handle if a grid is found
                    self.cache[cur_row, cur_col + 1].west = False
                    self.current = self.cache[cur_row, cur_col + 1]
                else:
                    self._add_neighbor(direction)

            case "WEST":
                self.current.west = False
                if self.cache.get((cur_row, cur_col - 1), False):
                    # handle if a grid is found
                    self.cache[cur_row, cur_col - 1].east = False
                    self.current = self.cache[cur_row, cur_col - 1]
                else:
                    self._add_neighbor(direction)

    def print_out(self):
        pass

    def print_moves(self):
        for each in self.cache.values():
            print(each)

    def print_maze(self):
        """
        lowk this shi chatgpt generated i got lazy
        since its mostly for sake of visuals/troubleshooting so i figured it doesnt matter
        sike i modified it to fit the class & save to a file
        """
        with open('maze.txt', 'w') as f:
            f.write('')

        grid = self.graph
        rows = len(grid)
        cols = len(grid[0])

        # Create drawing canvas
        height = rows * 2 + 1
        width = cols * 4 + 1
        canvas = [[" " for _ in range(width)] for _ in range(height)]

        # Place corner markers
        for r in range(0, height, 2):
            for c in range(0, width, 4):
                canvas[r][c] = "+"

        for r in range(rows):
            for c in range(cols):
                cell = grid[r][c]

                # If EMTY, assume no internal walls unless specified elsewhere
                if cell == "EMTY":
                    continue

                north, east, south, west = cell

                base_r = r * 2
                base_c = c * 4

                # North wall
                if north == "1":
                    for i in range(1, 4):
                        canvas[base_r][base_c + i] = "-"

                # South wall
                if south == "1":
                    for i in range(1, 4):
                        canvas[base_r + 2][base_c + i] = "-"

                # West wall
                if west == "1":
                    canvas[base_r + 1][base_c] = "|"

                # East wall
                if east == "1":
                    canvas[base_r + 1][base_c + 4] = "|"

        for row in canvas:
            output = "".join(row)
            with open('maze.txt', 'a') as f:
                f.write(output + "\n")
            print(output)
    def build_graph(self):
        # handle graphs with non-zero starting location
        graph_width = (self.max_col - self.min_col) + 1
        graph_height = (self.max_row - self.min_row) + 1

        # empty 2d list to hold encoded wall data
        self.graph = [["EMTY" for _ in range(graph_width)] for _ in range(graph_height)]

        for grid in self.cache.items():
            coordinate = grid[0]
            grid_row = coordinate[0]
            grid_col = coordinate[1]
            data = grid[1]
            
            # insert the encoded wall count into the graph at correct index
            self.graph[grid_row - self.min_row][grid_col - self.min_col] = data.walls_to_bits()
        self.graph.reverse()
    def print_graph(self):

        for row in range(len(self.graph)):
            row_out = ""

            for col in range(len(self.graph[row])):
                row_out += f"{self.graph[row][col]}\t"

            print(f"{row_out}")

    def fetch_tile(self, row, col):
        return self.cache[(row, col)]
        
#####################
# END AIDANS SHIT
#####################

TILE_SIZE = 0.5
MAX_ROTATION_SPEED = 6.28
MAX_TRUE_SPEED = 0.25

COULD_SEE_TILE_CUTOFF = 0.105
BETWEEN_CUTOFF = 0.17

START_OFFSET_X = TILE_SIZE / 2
START_OFFSET_Y = TILE_SIZE / 2

def main():
    # -----------------------------
    # Robot & Time Step
    # -----------------------------
    robot = Robot()
    for i in range(robot.getNumberOfDevices()):
        print(f"{i}: {robot.getDeviceByIndex(i).getName()} ({robot.getDeviceByIndex(i)})")

    # Get the basic time step of the current world
    timestep = int(robot.getBasicTimeStep())

    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')

    if not isinstance(leftMotor, Motor) or not isinstance(rightMotor, Motor):
        return

    # Set motors to velocity control mode
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))

    # Set a constant forward speed
    FORWARD_SPEED = 6.28
    leftMotor.setVelocity(FORWARD_SPEED)
    rightMotor.setVelocity(FORWARD_SPEED)
    
    

    # -----------------------------
    # Maze object
    # -----------------------------
    maze = Maze()
    last_tile = (0, 0)

    # -----------------------------
    # Camera
    # -----------------------------
    camera = robot.getDevice('camera')

    if not isinstance(camera, Camera):
        return

    camera.setFov(1.5)

    camera.enable(timestep)

    accelerometer = robot.getDevice('accelerometer')

    if not isinstance(accelerometer, Accelerometer):
        return

    accelerometer.lookup_table.clear()

    accelerometer.enable(timestep)

    gyro = robot.getDevice('gyro')

    if not isinstance(gyro, Gyro):
        return

    gyro.enable(timestep)

    groundSensors: list[DistanceSensor] = []

    for i in range(3):
        sensor = robot.getDevice(f"gs{i}")
        if not isinstance(sensor, DistanceSensor):
            continue

        sensor.enable(timestep)

        groundSensors.append(sensor)

        print(sensor.getLookupTable())

    distanceSensors: list[DistanceSensor] = []

    for i in range(8):
        sensor = robot.getDevice(f"ps{i}")
        if not isinstance(sensor, DistanceSensor):
            continue

        sensor.enable(timestep)

        distanceSensors.append(sensor)

        print(sensor.getLookupTable())

    lightSensors: list[LightSensor] = []

    for i in range(8):
        sensor = robot.getDevice(f"ls{i}")
        if not isinstance(sensor, LightSensor):
            continue

        sensor.enable(timestep)

        lightSensors.append(sensor)

        print(sensor.getLookupTable())


    imu = IMU(leftMotor, rightMotor, camera, accelerometer, gyro, groundSensors, distanceSensors)

    turnTarget = 0

    bufferTurn = -1

    wasBetweenFB = False

    # -----------------------------
    # Main Control Loop
    # -----------------------------
    while robot.step(timestep) != -1:
        print("----------==========----------")
        imu.step(timestep)
        
        cur_row = imu.getTileY()
        cur_col = imu.getTileX()
        cur_rot = round(imu.getRotation())
        if (cur_row, cur_col) != last_tile:
            # get NSEW direction
            direction = "dadgum nothin"
            if (cur_rot == 0):
                direction = "NORTH"
            elif (cur_rot == -90):
                direction = "EAST"
            elif (abs(cur_rot) == 180):
                direction = "SOUTH"
            elif (cur_rot == 90):
                direction = "WEST"
            
            maze.add_node(direction)
            last_tile = (cur_row, cur_col)
            maze.build_graph()
            maze.print_maze()
        print(f"{maze.cache}")
        print(f"{imu.getX():0.3f}, {imu.getY():0.3f} @ {cur_rot}")
        print(f"{imu.getIntraTileX()}, {imu.getIntraTileY()} in tile {(cur_col, cur_row)}")
        print(f"Between: {imu.isBeteenTiles()}")
        print(f"Between F/B: {imu.isBetweenTilesForwardsBack()}")
        print(f"Between L/R: {imu.isBetweenTilesLeftRight()}")
        print(f"Front: {imu.seesWallForward()}")
        print(f"Right: {imu.seesWallRight()}")
        print(f"Back: {imu.seesWallBack()}")
        print(f"Left: {imu.seesWallLeft()}")
        print(f"Could Right: {imu.couldSeeTileRight()}")
        print(f"Could Left: {imu.couldSeeTileLeft()}")

        if bufferTurn >= 0:
            if bufferTurn == 0:
                # Turn randomly
                turnTarget += 90 if bool(getrandbits(1)) else -90

                if turnTarget < -180:
                    turnTarget += 360

                if turnTarget > 180:
                    turnTarget -= 360
            
            bufferTurn -= 1

        angle = imu.getRotation()

        diff = getAngleDiff(turnTarget, angle)

        if wasBetweenFB and not imu.isBetweenTilesForwardsBack() and not imu.seesWallRight() and not imu.seesWallLeft():
            bufferTurn = 2

        if imu.seesWallForward() and abs(diff) < 2 and bufferTurn < 0:
            bufferTurn = 1

        if abs(diff) > 0.1:
            s = 2 * diff / 90
            if abs(s) > 1:
                s /= abs(s)

            leftMotor.setVelocity(-FORWARD_SPEED * s)
            rightMotor.setVelocity(s * FORWARD_SPEED)
        else:
            leftMotor.setVelocity(FORWARD_SPEED)
            rightMotor.setVelocity(FORWARD_SPEED)

        wasBetweenFB = imu.isBetweenTilesForwardsBack()

def getAngleDiff(base: float, angle: float) -> float:
    diff = base - angle

    if abs(diff) <= 180:
        return diff
    
    if base < angle:
        return getAngleDiff(base + 360, angle)
    else:
        return getAngleDiff(base, angle + 360)

def reverseLookup(sensor: DistanceSensor) -> float:
    table = sensor.getLookupTable()

    value = sensor.getValue()

    if len(table) < 6:
        return value
    
    lastRow = table[0:3]

    finalRowIndex = 3 * (len(table) // 3 - 1)

    for i in range(3, len(table), 3):
        thisRow = table[i:i+3]

        if value > max(lastRow[1], thisRow[1]):
            lastRow = thisRow
            continue

        if value < min(lastRow[1], thisRow[1]):
            lastRow = thisRow
            continue

        if i == finalRowIndex:
            threeSTDEV = 3 * thisRow[1] * thisRow[2]
            if abs(thisRow[1] - value) <= threeSTDEV:
                return -1

        slope = (lastRow[0] - thisRow[0]) / (lastRow[1] - thisRow[1])

        intercept = lastRow[0] - lastRow[1] * slope

        return slope * value + intercept
    
    return -1


    
    def facingEndWall(self):
        return self.facingGreenWall
    
    def getMovementVector(self, seconds: float):
        WHEEL_DIAMETER = 2 * pi * 0.00205

        lVel = self.leftMotor.getVelocity()
        rVel = self.rightMotor.getVelocity()

        if lVel == rVel:
            return (0, lVel * seconds * WHEEL_DIAMETER)
        
        if lVel < 0 != rVel < 0:
            return (0, 0)

        x = 0.0052
        a = max(lVel, rVel)
        b = min(lVel, rVel)
        r = b * x / (a - b)

        theta = a / (r + x)

        dForward = (r + x / 2) * cos(theta)
        dSide = (r + x / 2) * sin(theta)

        if rVel > lVel:
            dSide = -dSide

        dForward *= WHEEL_DIAMETER * seconds
        dSide *= WHEEL_DIAMETER * seconds

        return (dSide, dForward)
    

def getDirection(angle: float) -> Literal["NORTH", "EAST", "SOUTH", "WEST"]:
        if angle < -135:
            return "NORTH"
        elif angle < -45:
            return "EAST"
        elif angle < 45:
            return "SOUTH"
        elif angle < 135:
            return "WEST"
        
        return "SOUTH"


        self.gyro = GyroWrapper(gyro)

        self.gyro.setMode("degrees")

        self.wallStatus = [False, False, False, False, False, False]

        self.facingGreenWall = False
        self.facingRedWall = False
        
        whiteVotes = 0

        for sensor in self.groundSensors:
            if sensor.getValue() > IMU.WHITE_TILE_CUTOFF:
                whiteVotes += 1
        
        self.tileColor = "white" if whiteVotes >= 2 else "brown"

        self.x = 0
        self.y = 0

    def step(self, timeStep: int):
        seconds = timeStep / 1000
        self.accel.step(timeStep)
        self.gyro.step(timeStep)

        if self.leftMotor.getVelocity() == -self.rightMotor.getVelocity():
            self.accel._xVelocity = 0
            self.accel._yVelocity = 0
            self.accel._zVelocity = 0

        whiteVotes = 0

        for sensor in self.groundSensors:
            if sensor.getValue() > IMU.WHITE_TILE_CUTOFF:
                whiteVotes += 1

        self.wallStatus[0] = reverseLookup(self.distanceSensors[LEFT_17]) > 0 and reverseLookup(self.distanceSensors[RIGHT_17]) > 0
        self.wallStatus[1] = reverseLookup(self.distanceSensors[RIGHT_90]) > 0
        self.wallStatus[2] = reverseLookup(self.distanceSensors[LEFT_150]) > 0 and reverseLookup(self.distanceSensors[RIGHT_150]) > 0
        self.wallStatus[3] = reverseLookup(self.distanceSensors[LEFT_90]) > 0
        self.wallStatus[4] = reverseLookup(self.distanceSensors[RIGHT_46]) > 0
        self.wallStatus[5] = reverseLookup(self.distanceSensors[LEFT_46]) > 0

        newTileColor = "white" if whiteVotes >= 2 else "brown"
        
        if newTileColor != self.tileColor:
            self.tileColor = newTileColor


        angle = self.gyro.getZ() * GyroWrapper.DEGREES_TO_RADS

        WHEEL_CIRCUMFERENCE = 2 * pi * 0.00205

        distance = MAX_TRUE_SPEED * seconds * (self.leftMotor.getVelocity() + self.rightMotor.getVelocity()) / 2 / MAX_ROTATION_SPEED

        self.y += distance * cos(angle)
        self.x += distance * sin(-angle)

        self.facingRedWall = False
        self.facingGreenWall = False

        cameraImage = self.camera.getImageArray()
        r, g, b = cameraImage[IMU.CAMERA_PIXEL_X][IMU.CAMERA_PIXEL_Y]

        if r > 200 and g < 50 and b < 50:
            redHeight = 0
            for i in range(0, self.camera.getHeight()):
                r, g, b = cameraImage[IMU.CAMERA_PIXEL_X][i]
                if r > 200 and g < 50 and b < 50:
                    redHeight += 1
            
            if redHeight >= 30:
                self.facingRedWall = True
    
        if g > 200 and r < 50 and b < 50:
            greenHeight = 0
            for i in range(0, self.camera.getHeight()):
                r, g, b = cameraImage[IMU.CAMERA_PIXEL_X][i]
                if g > 200 and r < 50 and b < 50:
                    greenHeight += 1
            
            if greenHeight >= 30:
                self.facingGreenWall = True
            

    def getRotation(self) -> float:
        return self.gyro.getZ()
    
    def getFacingDirection(self) -> Literal["n", "e", "s", "w"]:
        angle = self.getRotation()

        if angle < -135:
            return "s"
        elif angle < -45:
            return "e"
        elif angle < 45:
            return "n"
        elif angle < 135:
            return "w"
        
        return "s"
    
    def getX(self):
        return self.x

    def getY(self):
        return self.y
    
    def getTileX(self) -> int:
        return floor((self.x + START_OFFSET_X) / TILE_SIZE)
    
    def getTileY(self) -> int:
        return floor((self.y + START_OFFSET_Y) / TILE_SIZE)
    
    def getIntraTileX(self) -> float:
        """
        Gets the x coordinate of the robot within its current tile
        """
        return self.x - TILE_SIZE * self.getTileX()
    
    def getIntraTileY(self) -> float:
        """
        Gets the y coordinate of the robot within its current tile
        """
        return self.y - TILE_SIZE * self.getTileY()
    
    def isBetweenTilesLeftRight(self) -> bool:
        direction = self.getFacingDirection()
        return abs(self.getIntraTileX()) > BETWEEN_CUTOFF if (direction == "n") or (direction == "s") else abs(self.getIntraTileY()) > BETWEEN_CUTOFF
    
    def isBetweenTilesForwardsBack(self) -> bool:
        direction = self.getFacingDirection()
        return abs(self.getIntraTileY()) > BETWEEN_CUTOFF if (direction == "n") or (direction == "s") else abs(self.getIntraTileX()) > BETWEEN_CUTOFF
    
    def isBeteenTiles(self) -> bool:
        return abs(self.getIntraTileX()) > BETWEEN_CUTOFF or abs(self.getIntraTileY()) > BETWEEN_CUTOFF
    
    def couldSeeTileRight(self) -> bool:
        direction = self.getFacingDirection()

        match direction:
            case "n":
                return self.getIntraTileX() > COULD_SEE_TILE_CUTOFF
            case "e":
                return self.getIntraTileY() < -COULD_SEE_TILE_CUTOFF
            case "s":
                return self.getIntraTileX() < -COULD_SEE_TILE_CUTOFF
            case "w":
                return self.getIntraTileY() > COULD_SEE_TILE_CUTOFF
            
    def couldSeeTileLeft(self) -> bool:
        direction = self.getFacingDirection()

        match direction:
            case "s":
                return self.getIntraTileX() > COULD_SEE_TILE_CUTOFF
            case "w":
                return self.getIntraTileY() < -COULD_SEE_TILE_CUTOFF
            case "n":
                return self.getIntraTileX() < -COULD_SEE_TILE_CUTOFF
            case "e":
                return self.getIntraTileY() > COULD_SEE_TILE_CUTOFF
    
    def seesWallForward(self):
        return self.wallStatus[0]
    
    def seesWallRight(self):
        return self.wallStatus[1]
    
    def seesWallBack(self):
        return self.wallStatus[2]
    
    def seesWallLeft(self):
        return self.wallStatus[3]
    
    def seesWallFrontRight(self):
        return self.wallStatus[4]
    
    def seesWallFrontLeft(self):
        return self.wallStatus[5]
    
    def getTileColor(self):
        return self.tileColor
    
    def facingStartWall(self):
        return self.facingRedWall
    
    def facingEndWall(self):
        return self.facingGreenWall
    
    def getMovementVector(self, seconds: float):
        WHEEL_DIAMETER = 2 * pi * 0.00205

        lVel = self.leftMotor.getVelocity()
        rVel = self.rightMotor.getVelocity()

        if lVel == rVel:
            return (0, lVel * seconds * WHEEL_DIAMETER)
        
        if lVel < 0 != rVel < 0:
            return (0, 0)

        x = 0.0052
        a = max(lVel, rVel)
        b = min(lVel, rVel)
        r = b * x / (a - b)

        theta = a / (r + x)

        dForward = (r + x / 2) * cos(theta)
        dSide = (r + x / 2) * sin(theta)

        if rVel > lVel:
            dSide = -dSide

        dForward *= WHEEL_DIAMETER * seconds
        dSide *= WHEEL_DIAMETER * seconds

        return (dSide, dForward)
    

def getDirection(angle: float) -> Literal["NORTH", "EAST", "SOUTH", "WEST"]:
        if angle < -135:
            return "NORTH"
        elif angle < -45:
            return "EAST"
        elif angle < 45:
            return "SOUTH"
        elif angle < 135:
            return "WEST"
        
        return "SOUTH"



class GyroWrapper:
    RADS_TO_DEGREES = 180 / pi
    DEGREES_TO_RADS = pi / 180

    # WeBots scales the sensor values by this for some reason
    SENSOR_OBFUSCATION_RATIO = 13.315805 / 100000

    def __init__(self, gyro: Gyro):
        self.gyro = gyro
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self.mode = "radians"

    def setMode(self, mode: Literal["degrees", "radians"]):
        self.mode = mode

    def step(self, timeStep: int):
        secondsElapsed = timeStep / 1000

        deltas = self.gyro.getValues()

        deltas = [delta * GyroWrapper.SENSOR_OBFUSCATION_RATIO for delta in deltas]

        self._x += deltas[0] * secondsElapsed
        self._y += deltas[1] * secondsElapsed
        self._z += deltas[2] * secondsElapsed

        if self._x > pi:
            self._x -= 2 * pi

        if self._y > pi:
            self._y -= 2 * pi

        if self._z > pi:
            self._z -= 2 * pi

        if self._x < -pi:
            self._x += 2 * pi

        if self._y < -pi:
            self._y += 2 * pi

        if self._z < -pi:
            self._z += 2 * pi

    def getX(self):
        return self._x if self.mode == "radians" else self._x * GyroWrapper.RADS_TO_DEGREES
    
    def getY(self):
        return self._y if self.mode == "radians" else self._y * GyroWrapper.RADS_TO_DEGREES
    
    def getZ(self):
        return self._z if self.mode == "radians" else self._z * GyroWrapper.RADS_TO_DEGREES
    
class AccelerometerWrapper:
    def __init__(self, accelerometer: Accelerometer):
        self.accelerometer = accelerometer
        self._xVelocity = 0.0
        self._yVelocity = 0.0
        self._zVelocity = 0.0
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0

    def step(self, timeStep: int):
        secondsElapsed = timeStep / 1000
        secondsSquared = secondsElapsed ** 2

        accelerations = self.accelerometer.getValues()

        accelerations[2] -=  9.80665

        self._x += self._xVelocity * secondsElapsed + 0.5 * accelerations[0] * secondsSquared
        self._y += self._yVelocity * secondsElapsed + 0.5 * accelerations[1] * secondsSquared
        self._z += self._zVelocity * secondsElapsed + 0.5 * accelerations[2] * secondsSquared

        self._xVelocity += accelerations[0] * secondsElapsed
        self._yVelocity += accelerations[1] * secondsElapsed
        self._zVelocity += accelerations[2] * secondsElapsed

    def getX(self):
        return self._x
    
    def getY(self):
        return self._y
    
    def getZ(self):
        return self._z

main()