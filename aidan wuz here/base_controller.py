
from controller import Robot
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
# Camera
# -----------------------------
camera = robot.getDevice('camera')
camera.enable(timestep)
camera_width = camera.getWidth()
cam_middle = camera_width//2

green_pixels = []

camera_upd = 0
updates = 1

# ---------------------------
# process colors
# ---------------------------
def ColorCheck():
    # limit the updates (performance i guess)
    global camera_upd
    if (camera_upd != updates):
        camera_upd += 1
        return 0
    camera_upd = 0
    
    try:
        # process current image of camera
        image = camera.getImage()
        # iterate through each pixel in the image
        for pixel in range(0, camera_width):
            # extract pixel rgb vales, look for green
            green = camera.imageGetGreen(image, camera_width, pixel, 0)
            blue = camera.imageGetBlue(image, camera_width, pixel, 0)
            red = camera.imageGetRed(image, camera_width, pixel, 0)
            
            if (green > 50 and not (blue > 50 or red > 50)):
                # add a green pixel to all current image green pixels
                green_pixels.append(pixel)

        if len(green_pixels) > 0:
            # find the middle of green pixels (if applicable)
            green_center = (sum(green_pixels))//(len(green_pixels))
            
            try:
                green_pixels.clear()
                return (cam_middle - green_center)/10
                
            except:
                print("if you're seeing this blame aidan")
                return 0
                
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
    offset *= offset_factor # mostly to make it turn faster
    l_offset = FORWARD_SPEED
    r_offset = FORWARD_SPEED
    
    # calculate how much turn it should turn
    if offset > 0:
        l_offset -= offset
    else:
        r_offset += offset
    
    # set the speed 
    left_motor.setVelocity(l_offset)
    right_motor.setVelocity(r_offset)

    
# -----------------------------
# Main Control Loop
# -----------------------------
while robot.step(timestep) != -1:
    ps_values = [sensor.getValue() for sensor in distance_sensors]
    Align(ColorCheck())

    pass
