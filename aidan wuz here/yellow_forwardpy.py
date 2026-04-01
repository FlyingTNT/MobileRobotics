from controller import Robot, Camera, Lidar
import math

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
ds_names = ['ds_front_left', 'ds_front_right', 'ds_back', 'ds_right', 'ds_left']
distance_sensors = {}
for name in ds_names:
    ds = robot.getDevice(name)
    ds.enable(TIME_STEP)
    distance_sensors[name] = ds

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

def get_lidar_distances():
    """
    Returns a flat list of distance readings (in meters) from the lidar.

    - Each value is the distance to the nearest obstacle in that direction.
    - Values equal to 'inf' mean nothing was detected in that direction.
    - The list goes left-to-right across the lidar's field of view.

    Example usage:
        distances = get_lidar_distances()
        front_distance = distances[len(distances) // 2]  # center ray
    """
    return list(lidar.getRangeImage())

def get_lidar_sector(distances, sector='front'):
    """
    Returns the minimum distance detected in a named sector of the lidar.

    Sectors divide the lidar's view into five equal zones:
        'left', 'front-left', 'front', 'front-right', 'right'

    Parameters:
        distances (list): output from get_lidar_distances()
        sector (str): one of the five sector names above

    Returns:
        float: closest distance (meters) in that sector, or inf if nothing detected

    Example usage:
        distances = get_lidar_distances()
        if get_lidar_sector(distances, 'front') < 0.3:
            print("Obstacle ahead!")
    """
    n = len(distances)
    sectors = {
        'left':        distances[0           : n // 5],
        'front-left':  distances[n // 5      : 2 * n // 5],
        'front':       distances[2 * n // 5  : 3 * n // 5],
        'front-right': distances[3 * n // 5  : 4 * n // 5],
        'right':       distances[4 * n // 5  :],
    }
    readings = sectors.get(sector, [])
    return min((d for d in readings if not math.isinf(d)), default=float('inf'))

camera_upd = 0
updates = 0
camera_width = 1024
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
        for pixel in range(0, camera_width):
            # extract pixel rgb vales, look for green
            green = camera.imageGetGreen(image, camera_width, pixel, target_row)
            blue = camera.imageGetBlue(image, camera_width, pixel, target_row)
            red = camera.imageGetRed(image, camera_width, pixel, target_row)

            ###
            #print(f"{red}R {green}G {blue}B")
            ###
            match color:
                case "yellow":
                    if ((green > 120 and red > 120) and not (blue > 100)):
                        # add a yellow pixel to all current image green pixels
                        pixels.append(pixel)
                case "magenta":
                    pass
                case "cyan":
                    pass


        if len(pixels) > 0:
            # green_center = (sum(green_pixels)) // (len(green_pixels))  # find the pixel center of green in row
            width = max(pixels) - min(pixels)
            midpoint = round((max(pixels) + min(pixels))/2, 0)
            try:
                # return the distance from center (yes this should not be handled here but alas)
                pixels.clear()
                return (width, midpoint)# impportant yellow value here

            except:
                # if this prints out you have bigger fish to fry 
                print("if you're seeing this blame aidan")
                return 0
        else:
            print("you broke some shit dumbass")

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
    return "no dadgum pixels"
canon_time = 0
# each tile is .25 meters long

# ------------------ MAIN LOOP ------------------
while robot.step(TIME_STEP) != -1:
    leftSpeed = 0.0
    rightSpeed = 0.0

    canon_time = round(robot.getTime(), 3)
    # print(canon_time)
    yellow = ColorCheck("yellow")
    if yellow != 0:
        print(BDC(yellow[0]))
        print(angle_offset(yellow[1]))


    '''
    with open('data.txt', 'a') as f:
        f.write(f"{canon_time}: {yellow}\n")
    '''

    # ================= LIDAR READING =================
    # Get all distance readings from the lidar this timestep
    distances = get_lidar_distances()

    # Example: read the closest obstacle distance straight ahead
    front_dist = get_lidar_sector(distances, 'front')

    # Example: stop if something is within 0.2 meters in front
    # if front_dist < 0.2:
    #     set_speed(0, 0)
    #     continue

    # ================= YELLOW DETECTION =================
    image = camera.getImage()
    yellow_x_sum = 0
    yellow_count = 0

    
    for y in range(cam_height):
        for x in range(cam_width):
            r = Camera.imageGetRed(image, cam_width, x, y)
            g = Camera.imageGetGreen(image, cam_width, x, y)
            b = Camera.imageGetBlue(image, cam_width, x, y)
            if is_yellow(r, g, b):
                yellow_x_sum += x
                yellow_count += 1

    if yellow_count > 1:
        # ---- CHARGE OPPONENT ----
        avg_x = yellow_x_sum / yellow_count
        if avg_x < cam_width * 0.4:
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = MAX_SPEED
        elif avg_x > cam_width * 0.6:
            leftSpeed  = MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
        else:
            leftSpeed  = MAX_SPEED
            rightSpeed = MAX_SPEED

    set_speed(leftSpeed, rightSpeed)