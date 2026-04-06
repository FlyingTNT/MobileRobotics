from controller import Robot, Lidar, Motor, DistanceSensor, Camera, LidarPoint
from math import sin, cos, tan, isinf, isnan, sqrt, atan2, pi
from typing import Literal
import traceback
import sys
import math
import random

TIME_STEP = 64
MAX_SPEED = 10

robot = Robot()



# ------------------ CAMERA ------------------
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
cam_width = camera.getWidth()
cam_height = camera.getHeight()

# ------------------ LIDAR ------------------
lidar = robot.getDevice('LDS-01')
lidar.enable(TIME_STEP)
lidar.enablePointCloud()
lidar.setFrequency(25.0)

lidarMotor1 = robot.getDevice("LDS-01_main_motor")
lidarMotor2 = robot.getDevice("LDS-01_secondary_motor")
lidar1Velocity = lidarMotor1.getMaxVelocity()
lidar2Velocity = lidarMotor2.getMaxVelocity()
lidarMotor1.setPosition(float("infinity"))
lidarMotor2.setPosition(float("infinity"))

lidar_resolution = lidar.getHorizontalResolution()
lidar_fov = lidar.getFov()
# ------------------ WHEELS ------------------
wheels = []
wheel_names = [
    'front_left_wheel',
    'front_right_wheel',
    'back_left_wheel',
    'back_right_wheel'
]
for name in wheel_names:
    w = robot.getDevice(name)
    w.setPosition(float('inf'))
    w.setVelocity(0.0)
    wheels.append(w)

def set_speed(left, right):
    wheels[0].setVelocity(left)   # front left
    wheels[2].setVelocity(left)   # back left
    wheels[1].setVelocity(right)  # front right
    wheels[3].setVelocity(right)  # back right
    
# ------------------ DISTANCE SENSORS ------------------
frontLeftDistance = robot.getDevice("ds_front_left")
frontRightDistance = robot.getDevice("ds_front_right")
backDistance = robot.getDevice("ds_back")
rightDistance = robot.getDevice("ds_right")
leftDistance = robot.getDevice("ds_left")

def get_ds(name):
    """
    Returns the distance reading (in meters) for a named sensor.
    
    Available sensors:
        'DS_FRONT_LEFT', 'DS_FRONT_RIGHT', 'DS_BACK', 'DS_RIGHT', 'DS_LEFT'
    
    Example usage:
        if get_ds('DS_FRONT_LEFT') < 0.2:
            print("Close to something on the front-left!")
    """
    return distance_sensors[name].getValue()
    
def clamp(x, lo, hi): #utility function that constrains a value between a minimum and maximum bound [use if wanted]
    return max(lo, min(hi, x))

def is_yellow(r, g, b):
    # Core yellow: high R, high G, low B
    if r > 140 and g > 140 and b < 100:
        return True
    # Bright/washed-out yellow at distance (lighting reduces saturation)
    if r > 160 and g > 160 and b < 130 and r > b + 50 and g > b + 50:
        return True
    # Darker yellow in shadows
    if r > 100 and g > 100 and b < 60 and r > b + 60 and g > b + 60:
        return True
    return False


CAMERA_WIDTH = 1024
CAMERA_FOV = 1.57
TRUE_BALL_WIDTH = 0.04 * 2

camera_upd = 0
updates = 0
def ColorCheck(color):
    '''
    color detector salvaged from my implementation of the first project
    '''
    global camera_upd
    global updates
    if (camera_upd != updates):
        camera_upd += 1
        return 0
    camera_upd = 0
    
    pixels = []

    try:
        # process current image of camera
        image = camera.getImage()
        # iterate through each pixel in the image
        target_row = 0
        for pixel in range(0, CAMERA_WIDTH):
            # extract pixel rgb vales, look for green
            green = camera.imageGetGreen(image, CAMERA_WIDTH, pixel, target_row)
            blue = camera.imageGetBlue(image, CAMERA_WIDTH, pixel, target_row)
            red = camera.imageGetRed(image, CAMERA_WIDTH, pixel, target_row)

            ###
            #print(f"{red}R {green}G {blue}B")
            ###
            match color:
                case "yellow":
                    if ((green > 120 and red > 120) and not (blue > 100)):
                        # add a yellow pixel to all current image green pixels
                        pixels.append(pixel)
                case "magenta":
                    if ((red > 150 and blue > 150 ) and not (green > 100)):
                        # add a yellow pixel to all current image green pixels
                        pixels.append(pixel)
                case "cyan":
                    if ((blue > 150 and green > 150) and not (red > 100)):
                        # add a yellow pixel to all current image green pixels
                        pixels.append(pixel)


        if len(pixels) > 0:
            # green_center = (sum(green_pixels)) // (len(green_pixels))  # find the pixel center of green in row
            width = max(pixels) - min(pixels)
            midpoint = round((max(pixels) + min(pixels))/2, 0)
            try:
                # return the distance from center (yes this should not be handled here but alas)
                pixels.clear()
                return (width, midpoint)

            except:
                # if this prints out you have bigger fish to fry 
                print("if you're seeing this blame aidan")
                return 0
        else:
            print("no colored pixels")

        pixels.clear()
        return 0

    except ValueError:
        print("No camera yet")
        return 0

def angle_offset(offset):
    """
    pixel 0 = -.785 RAD
    pixel 1024 = +0.785 RAD
    ->DEG= +/- 44.97719
    """
    cam_center = 1024/2
    pixel_offset_ratio = (cam_center+offset)/cam_center

    print(pixel_offset_ratio)
    rad = pixel_offset_ratio * 44.97719
    return rad - 90


def BDC(pixel_width):
    '''
    Ball Distance Calculator finds the distance from the robot to the ball based on
    some data that I analyzed. Uses a relation between time, displacement, and the
    width of the ball seen on the camera
    d(W) = (roughly) 78.5/W, where W=width of Yellow pixels on camera
    '''
    constant = 78.5/2
    #distance = 0.8*math.log(pixel_width/42)
    if pixel_width != 0:
        distance = constant/pixel_width
        return distance
    return 67676767676767

def go(speed: float, turn: float):
    leftSpeed = speed * (1 + turn)
    rightSpeed = speed * (1 - turn)
    if abs(leftSpeed) > 1:
        leftSpeed /= abs(leftSpeed)

    if abs(rightSpeed) > 1:
        rightSpeed /= abs(rightSpeed)

    frontLeftMotor.setVelocity(MAX_MOTOR_SPEED * leftSpeed)
    backLeftMotor.setVelocity(MAX_MOTOR_SPEED * leftSpeed)

    frontRightMotor.setVelocity(MAX_MOTOR_SPEED * rightSpeed)
    backRightMotor.setVelocity(MAX_MOTOR_SPEED * rightSpeed)

######################################################


def getTrueDistance(sensor: DistanceSensor) -> float:
    value = sensor.getValue()

    return -1 if value == 1000.0 else value / 10000


class IMU:
    def __init__(self, lidar: Lidar, camera: Camera):
        self.lidar = lidar
        self.camera = camera
        self.rotation = 0.0
        self.spins = 0
        self.x = 0.0
        self.y = 0.0
        self.canBeTrusted = False

    def step(self, timestep: int):
        points = [point for point in self.lidar.getLayerPointCloud(0) if not isinf(point.x)]
        discontinuities = []

        last = points[0]

        for i in range(1, len(points)):
            this = points[i]

            if abs(last.x - this.x + last.y - this.y) > 0.5:
                discontinuities.append(i)

            last = this

        if len(discontinuities) > 1:
            if discontinuities[1] - discontinuities[0] < 10:
                points = points[0:discontinuities[0]] + points[discontinuities[1]:]
            else:
                points = points[discontinuities[0]:discontinuities[1]]

        # print(discontinuities)

        walls = DouglasPeucker(points, 0, len(points) - 1, 0.05)

        walls[-1] = walls[0]

        x1 = walls[0].x - walls[1].x
        y1 = walls[0].y - walls[1].y
        x2 = walls[-1].x - walls[-2].x
        y2 = walls[-1].y - walls[-2].y

        if abs(x1 * y2 - x2 * y1) < 0.05:
            walls.pop(0)
            walls.pop(-1)

        # print(f"Walls: {[(point.x, point.y) for point in walls]}")

        maxLengthIndex = -1
        maxLength = -1

        for i in range(1, len(walls)):
            lastPoint = walls[i - 1]
            thisPoint = walls[i]

            distSquared = (lastPoint.x - thisPoint.x) ** 2 + (lastPoint.y - thisPoint.y) ** 2
            if distSquared > maxLength:
                maxLengthIndex = i
                maxLength = distSquared
        startPoint = walls[maxLengthIndex - 1]
        endPoint = walls[maxLengthIndex]

        area, cX, cY = analyzeShape(walls)

        badData = area < 2.2

        self.canBeTrusted = not badData

        if badData:
            return

        newRotation = (180 * atan2(startPoint.y - endPoint.y, startPoint.x - endPoint.x) / pi)

        if newRotation < 0:
            newRotation += 180

        if abs(getAngleDiff(newRotation, self.rotation)) > 160:
            if newRotation > self.rotation:
                self.spins -= 1
            else:
                self.spins += 1

        self.rotation = newRotation
        if self.rotation < -180:
            self.rotation += 360

        rotationRads = -(newRotation * pi / 180 + pi * self.spins)

        ccX, ccY = cX * cos(rotationRads) - cY * sin(rotationRads), cX * sin(rotationRads) + cY * cos(rotationRads)

        self.x = ccX
        self.y = ccY

    def getRotation(self):
        rot = 180 * self.spins + self.rotation

        while rot > 180:
            rot -= 360

        while rot < -180:
            rot += 360

        return -rot

    def getX(self):
        return self.x

    def getY(self):
        return self.y


def getAngleDiff(base: float, angle: float) -> float:
    diff = base - angle

    if abs(diff) <= 180:
        return diff

    if base < angle:
        return getAngleDiff(base + 360, angle)
    else:
        return getAngleDiff(base, angle + 360)


def analyzeShape(edges: list[LidarPoint]) -> tuple[float, float, float]:
    area = 0
    centroidX = 0
    centroidY = 0

    for i in range(len(edges)):
        lastPoint = edges[i - 1]
        thisPoint = edges[i]

        ctd = lastPoint.x * thisPoint.y - thisPoint.x * lastPoint.y
        area += ctd
        centroidX += (lastPoint.x + thisPoint.x) * ctd
        centroidY += (lastPoint.y + thisPoint.y) * ctd

    area /= 2
    centroidX /= 6 * area
    centroidY /= 6 * area

    return (abs(area), centroidX, centroidY)


def DouglasPeucker(points: list[LidarPoint], start: int, end: int, epsilon: float):
    # Copied from wikipedia pseudocode; slightly modified for performance
    maxDistance = 0
    maxIndex = -1

    a = (points[start].y - points[end].y)
    b = -(points[start].x - points[end].x)
    c = -a * points[start].x + -b * points[start].y

    for i in range(start + 1, end - 1):
        d = abs(a * points[i].x + b * points[i].y + c)
        if (d > maxDistance):
            maxIndex = i
            maxDistance = d

    outPoints = []

    if (maxDistance > epsilon * sqrt(a * a + b * b)):
        recOut1 = DouglasPeucker(points, start, maxIndex, epsilon)
        recOut2 = DouglasPeucker(points, maxIndex, end, epsilon)

        outPoints = recOut1 + recOut2[1:]
    else:
        outPoints = [points[start], points[end]]

    return outPoints
######################################################


def xy_average(x_list, y_list):
    x_avg = sum(x_list)/len(x_list)
    y_avg = sum(y_list)/len(y_list)
    return (x_avg, y_avg)


ENEMY_GOAL = "cyan"

MODE = "START"
imu = IMU(lidar, camera)
canon_time = 0
DECADE = 7
x_decade = [None] * DECADE
y_decade = [None] * DECADE
xy_count = 0
xy_avg = None

imu_errors = 0
# each tile is .25 meters long

### Autonomous period stuff ###
xy_data_ready = False
goal_camping = (-1.06, -0.08)
rotated_into_pos = False
at_target = False
forwards = False
error_count = 0

### Scoring stuff ###
last_ball_pos = 0
enemy_goal = (1.1, 0)


### rocket league insults ###
goal_msg = ["What a save!", "Nice block!", "Close one!", "Siiiick!", "Whoops...", "No problem.", "Great Clear!", "Thanks!"]

def calculate_transform(start, current_angle, dest):
    '''
    Input should be 2 tuples of (x, y)
    Returns a tuple in the structure:
    (Hypotenuse, degree_offset, angle_difference)
    '''
    x = start[0] - dest[0]
    y = start[1] - dest[1]
    hyp = sqrt(x**2 + y**2)
    deg = round((math.atan(y/x)) * 180/pi, 1)
    angle_diff = current_angle - deg

    return (hyp, deg, angle_diff)

print("glhf!")
# ------------------ MAIN LOOP ------------------
while robot.step(TIME_STEP) != -1:

    try:
        imu.step(TIME_STEP)
    
    except:
        print("dadgum imu broke")
    leftSpeed = 0.0
    rightSpeed = 0.0

    if xy_count < DECADE:
        x_decade[xy_count] = imu.getX()
        y_decade[xy_count] = imu.getY()
        xy_count += 1
        if xy_data_ready:
            xy_avg = xy_average(x_decade, y_decade)
    elif imu.canBeTrusted:
        xy_count = 1
        x_decade[0] = imu.getX()
        y_decade[0] = imu.getY()
        xy_avg = xy_average(x_decade, y_decade)
        xy_data_ready = True
    else:
        print("Bad imu data.")

    cur_rot = imu.getRotation()
    #print(cur_rot)
    #print(xy_avg)

    canon_time = round(robot.getTime(), 3)
    # print(canon_time)
    yellow = ColorCheck("yellow")
    if yellow != 0:
        #print(BDC(yellow[0]))
        # print(angle_offset(yellow[1]))
        pass
    def no_balls():
        global error_count
        if yellow == 0:
            error_count += 1
            if error_count > 10:
                MODE = "POSITION"
                error_count = 0
                print("Back to goal camping")
        elif error_count > 0:
            error_count -= 1


    print(f"XY_Average Position: {xy_avg}\n"
          f"Rotation: {cur_rot}\n"
          f"Mode: {MODE}")

    match MODE:
        case "START":
            # print(f"{round(imu.getX(), 2)}, {round(imu.getY(), 2)}")

            rotated_into_pos = False
            at_target = False
            forwards = False

            if xy_avg != None: MODE = "POSITION"
            print("Chat Log: Centering!")
        case "POSITION":
            # find displacement to own goal
            x = xy_avg[0] - goal_camping[0]
            y = xy_avg[1] - goal_camping[1]

            if abs(x) < 0.1 and abs(y) < 0.1:
                rotated_into_pos = True
                at_target = True
                

            print(sqrt(x**2 + y**2))
            # begin positioning in goal
            if xy_avg != goal_camping:
                # handle imu errors
                if not imu.canBeTrusted: imu_errors += 1
                if imu_errors > 10: MODE = "UHOH" # if the imu starts acting a fool just go after ball
                    
                print(f"IMU Trusted: {imu.canBeTrusted}\nIMU Errors: {imu_errors}")
                tar_rot = round((math.atan(y/x)) * 180/pi, 1)
                angle_diff = cur_rot - tar_rot
                print(f'Offset: {x}, {y}\nTar Rot: {tar_rot}\nAngle Diff: {angle_diff}')
                ## step one: rotate to the face the right position
                if not abs(angle_diff) < 5 and not rotated_into_pos:
                    if -angle_diff < 0:
                        rightSpeed = -8
                        leftSpeed = 8
                    else:
                        leftSpeed = -8
                        rightSpeed = 8
                ## step two: back tf up
                elif not sqrt(x**2 + y**2) < .01 and not at_target:
                    rotated_into_pos = True
                    rightSpeed = -10
                    leftSpeed = -10
                ## step three: rotate into position
                else:
                    if cur_rot < -2:
                        rightSpeed = +10
                        leftSpeed = -10
                    elif cur_rot > 2:
                        rightSpeed = -10
                        leftSpeed = +10
                    else:
                        MODE = "WAITING"
                    at_target = True
                    print("Chat Log: In position.")
                    canon_time = 0

        case "WAITING":
            # handle no ball
            try:
                ball_dist = BDC(yellow[0])
            except:
                pass # oh lord

            if yellow == 0:
                error_count += 1
                if error_count > 10:
                    MODE = "START"
                    error_count = 0
                    print("Chat Log: Defending.")
                        
            elif (ball_dist < .5) or (canon_time > 15):  # start going when the ball is close/time has passed
                # decrease error buildup
                if error_count > 0: error_count -= 1
                #print(angle_offset(yellow[1]))
                ## correct angle until ball is centered on this robot, then full send to steal ball and run (SBR reference)
                if yellow[1] - CAMERA_WIDTH/2 < -50:
                    if ball_dist < .3:
                        leftSpeed = -4
                    else:
                        leftSpeed = -10
                    rightSpeed = +10
                elif yellow[1] - CAMERA_WIDTH/2 > 50:
                    if ball_dist < .3:
                        rightSpeed = -4
                    else:
                        rightSpeed = -10
                    leftSpeed = +10
                else:
                    # print(f"Robot thinks its centered {yellow[1] - CAMERA_WIDTH/2}")
                    # print(xy_avg)
                    rightSpeed = 10
                    leftSpeed = 10
                    print("Chat Log: Great pass!")
                    # change modes one tile before halfway
                    if xy_avg[0] >= -0.25:
                        MODE="BALLCHASE" ## technically not a bad thing to go into uh oh mode here

        case "BALLCHASE":
            #### aidan here - this method was smart but im not smart enough to program it, hence why i scrapped it
            ## handle no ball (again)
            if yellow == 0:
                error_count += 1
                if error_count > 3:
                    MODE = "START"
                    error_count = 0
                    print("Chat Log: Defending.")
            elif xy_avg[0] < 1:
                error_count = 0
                if yellow[1] - CAMERA_WIDTH/2 > 75:
                    leftSpeed = +8
                    rightSpeed = -10
                elif yellow[1] - CAMERA_WIDTH / 2 < -75:
                    rightSpeed = +8
                    leftSpeed = -10
                else:
                    leftSpeed = +10
                    rightSpeed = +10
            else:
                try:
                    goal = ColorCheck(ENEMY_GOAL)
                    print(f"Yellow Pixels: {yellow}\n{ENEMY_GOAL}: {goal}")
                    if yellow[0] > 300 and goal[0] > 300:
                        rand = random.randint(0, len(goal_msg))
                        print(goal_msg[rand])
                except:
                    pass  # surely this code could NEVER be a bad idea
                error_count = 0


        case "UHOH":
            # a fairly more forgiving defense mode toggle
            print("Chat Log: going for goal")
            if yellow == 0:
                error_count += 1
                if error_count > 30:
                    MODE = "START"
                    error_count = 0
                    print("Chat Log: Defending.")
            ## this is for when the imu just outright breaks
            goal = ColorCheck(ENEMY_GOAL)
            if goal == 0:
                goal = ColorCheck("cyan")
            print(f"Yellow Pixels: {yellow}\n{ENEMY_GOAL}: {goal}")
            # make robot track ball
            if yellow == 0:
                leftSpeed = +10
                rightSpeed = -10
            elif yellow[1] - CAMERA_WIDTH/2 > 75:
                error_count = 0
                leftSpeed = +10
                rightSpeed = -8
            elif yellow[1] - CAMERA_WIDTH / 2 < -75:
                error_count = 0
                rightSpeed = +10
                leftSpeed = -8
            elif goal != 0 and abs(goal[1] - CAMERA_WIDTH/2)  < 200:
                leftSpeed = +10
                rightSpeed = +10
            try:
                if yellow[0] > 300 and goal[0] > 300:
                    rand = random.randint(0, len(goal_msg))
                    print(goal_msg[rand])
            except:
                pass # surely this code could NEVER be a bad idea

    print("\n "*2)
    set_speed(leftSpeed, rightSpeed)

