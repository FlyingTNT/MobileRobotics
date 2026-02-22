from controller import Robot, Motor, Camera, Gyro
from typing import Literal
from math import pi, pow, isnan, sin, cos
import time

# -----------------------------
# Robot & Time Step
# -----------------------------
robot = Robot()

# Get the basic time step of the current world
timestep = int(robot.getBasicTimeStep())

# -----------------------------
# Motors (Differential Drive)
# -----------------------------
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Set motors to velocity control mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Set a constant forward speed
FORWARD_SPEED = 6.28 # Max = 6.28
left_motor.setVelocity(FORWARD_SPEED)
right_motor.setVelocity(FORWARD_SPEED)

# Distance Sensors n shi
distance_sensors = []

for i in range(8):
    sensor = robot.getDevice(f'ps{i}')
    sensor.enable(timestep)
    distance_sensors.append(sensor)

# -----------------------------
# CAMERA DECLARATION
# -----------------------------
camera = robot.getDevice('camera')
camera.setFov(1.5)
camera.enable(timestep)

# CAMERA VARIABLES
camera_width = camera.getWidth()
cam_middle = camera_width//2

green_pixels = []

camera_upd = 0
updates = 1

# controls the robot 
OVERRIDE = False

# Gyro code
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
   
    def getZ(self):
        return self._z if self.mode == "radians" else self._z * GyroWrapper.RADS_TO_DEGREES



#### GYRO DECLARATION
gyro = robot.getDevice("gyro")
gyro.enable(timestep)

gyroWrap = GyroWrapper(gyro)
gyroWrap.setMode('degrees')


# OVERRIDE VARIABLES            
override_trigger = False
delay = 0 # in seconds
last_update = 0 #timestamp

forward_mode = False
forward_timestamp = 0 # timestamp
forward_duration = .3 # in seconds

# ----------------
# Override follow control when no green is detected
# ----------------
def override_mode():
    global OVERRIDE
    global override_trigger
    global last_update
    angle = gyroWrap.getZ()
    check_time = time.time()
    
    # make the robot go forwards after entering override to clear wall
    if check_time - forward_timestamp < forward_duration:
        print("forwards mode")
        left_motor.setVelocity(FORWARD_SPEED)
        right_motor.setVelocity(FORWARD_SPEED)
        return
        
    # delay to prevent excessive mode switching (honestly doesnt work)
    if check_time - last_update < delay:
        print("not enough time")
        return
    
    # prevent change of direction until green is found, then break out of override mode
    elif (override_trigger == True):
        if ColorCheck() != 0:
            OVERRIDE = False
            override_trigger = False
        
        return
        
    # choose left or right directions based on the angle of the bot
    elif angle < -10:
        print("Goin left")
        left_motor.setVelocity(-4)
        right_motor.setVelocity(2 )
        override_trigger = True
        
    elif angle > 10:
        print("Goin right")       
        left_motor.setVelocity(2)
        right_motor.setVelocity(-4)
        override_trigger = True
        
    # yeah this prevents the unexpected behavior, couldnt tell you how/why
    else:
        print("dadgum robot is busted")
        OVERRIDE = False
    
    # used to update delay, once again doesnt really work
    last_update = check_time



# ---------------------------
# process colors
# ---------------------------
def ColorCheck():
    # limit the updates (performance i guess)
    global camera_upd
    global OVERRIDE
    global forward_timestamp
    global forward_mode
    if (camera_upd != updates):
        camera_upd += 1
        return 0
    camera_upd = 0
    
    try:
        # process current image of camera
        image = camera.getImage()
        # iterate through each pixel in the image
        target_row = 19  
        for pixel in range(0, camera_width):            
            # extract pixel rgb vales, look for green
            green = camera.imageGetGreen(image, camera_width, pixel, target_row)
            blue = camera.imageGetBlue(image, camera_width, pixel, target_row)
            red = camera.imageGetRed(image, camera_width, pixel, target_row)
            
            if (green > 100 and not (blue > 50 or red > 50)):
                # add a green pixel to all current image green pixels
                green_pixels.append(pixel)

        if len(green_pixels) > 0:
            green_center = (sum(green_pixels))//(len(green_pixels)) # find the pixel center of green in row
            
            try:
                # return the distance from center (yes this should not be handled here but alas)
                green_pixels.clear()
                return (cam_middle - green_center)/10
                
            except:
                # if this prints out you have bigger fish to fry 
                print("if you're seeing this blame aidan")
                return 0
        else:
            # go into override mode if no green is detected
            OVERRIDE = True

        green_pixels.clear()
        return 0
        
    except ValueError:
        print("No camera yet")
        return 0
        
# Align robot to green
offset_factor = 2

def Align(offset):
    # -----
    # offset value changes the direction by subtracting 
    # the paramter from the max speed of the motor
    # -----    
    if OVERRIDE:
        # prevents override mode from arguing over the motors
        return
        
    else:
        offset *= offset_factor # mostly to make it turn faster
        if offset < -6.48: offset = -6.48
        elif offset > 6.48: offset = 6.48
        
        l_offset = FORWARD_SPEED
        r_offset = FORWARD_SPEED
        
        # calculate how much turn it should turn
        # deadass basically is "if left turn left" its that simple
        if offset > 0:
            l_offset -= offset
        else:
            r_offset += offset
    
    # set the speed of the direction the bot needs to go
    left_motor.setVelocity(l_offset)
    right_motor.setVelocity(r_offset)

    
# -----------------------------
# Main Control Loop
# -----------------------------
while robot.step(timestep) != -1:
    gyroWrap.step(timestep)
    ps_values = [sensor.getValue() for sensor in distance_sensors]
    
    # override behavior when no green is found, otherwise follow the center of green on the screens
    if not OVERRIDE:
        Align(ColorCheck())
        forward_timestamp = time.time()
    else:
        override_mode()
