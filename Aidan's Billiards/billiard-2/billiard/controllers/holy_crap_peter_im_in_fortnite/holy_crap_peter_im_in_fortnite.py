from controller import Robot

############# COLORS ###
# use these to control what ball/goal the bot goes for
ball_color = "yellow"
hole_color = "green"
########################



TIME_STEP = 64
robot = Robot()

wheels = []
wheelsNames = ['wheelLeft', 'wheelRight']

door_left = robot.getDevice("doorLeft")
door_right = robot.getDevice("doorRight")

door_left.setPosition(float('inf'))
door_left.setVelocity(0.0)
door_right.setPosition(float('inf'))
door_right.setVelocity(0.0)

MAX_VEL = 20 # the mf max velocity

ds_1 = robot.getDevice("DS_1")
ds_2 = robot.getDevice("DS_2")
ds_1.enable(TIME_STEP)
ds_2.enable(TIME_STEP)

# ------------------ CAMERA ------------------
camera_une = robot.getDevice('camera')
camera_deux = robot.getDevice('camera_THEFUCKINGSEQUEL')
camera_une.enable(TIME_STEP)
camera_deux.enable(TIME_STEP)
CAMERA_WIDTH = camera_une.getWidth()
CAM_HEIGHT = camera_une.getHeight()
cam_middle = int(CAMERA_WIDTH/2)

# control updates per frame
camera_upd = 0
updates = 0


##################### Helpers
def ColorCheck(color, camera):
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
            # print(f"{red}R {green}G {blue}B")
            ###
            match color:
                case "yellow":
                    if ((green > 120 and red > 120) and not (blue > 75)):
                        # add a pixel to all current image pixels
                        pixels.append(pixel)
                case "magenta":
                    if ((red > 150 and blue > 150) and not (green > 100)):
                        # add a pixel to all current image pixels
                        pixels.append(pixel)
                case "cyan":
                    if ((blue > 150 and green > 150) and not (red > 75)):
                        # add a pixel to all current image pixels
                        pixels.append(pixel)
                case "blue":
                    if ((blue > 150) and (not (red > 75) and not (green > 75))):
                        # add a pixel to all current image pixels
                        pixels.append(pixel)
                case "red":
                    if ((red > 150) and (not (blue > 75) and not (green > 75))):
                        # add a pixel to all current image pixels
                        pixels.append(pixel)
                case "green":
                    if ((green > 150) and (not (red > 75) and not (blue > 75))):
                        # add a pixel to all current image pixels
                        pixels.append(pixel)

        if len(pixels) > 0:
            width = max(pixels) - min(pixels)
            midpoint = round((max(pixels) + min(pixels)) / 2, 0)
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



for i in range(2):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

def clamp(lSpeed, rSpeed):
    if lSpeed < 0:
        lSpeed = max(-MAX_VEL, lSpeed)
    else:
        lSpeed = min(MAX_VEL, lSpeed)

    if rSpeed < 0:
        rSpeed = max(-MAX_VEL, rSpeed)
    else:
        rSpeed = min(MAX_VEL, rSpeed)

    return (lSpeed, rSpeed)


def is_detected(ds, threshold=30):
    print(ds.getValue())
    return ds.getValue() < threshold


######################### END HELPERS

# behavior control
MODE = "SEARCH"

# sets the speed of turning to correct when following ball
speed_factor = 10
ball_clamping = False
canon_time = 0


while robot.step(TIME_STEP) != -1:

    print()
    leftSpeed = 10
    rightSpeed = 10

    ball = ColorCheck(ball_color, camera_une)
    hole = ColorCheck(hole_color, camera_deux)

    print(MODE)
    print(ball)
    print(hole)
    print("\n"*5)

    match MODE:
        # rotate towards the ball
        case "SEARCH":
            print("Search")
            if ball == 0:
                leftSpeed = -10
                rightSpeed = 10
            else:
                MODE = "CHASE"
        # full send that bih towards the ball
        case "CHASE":
            print("Chase")
            if ball == 0:
                MODE = "SEARCH"
                continue
            elif is_detected(ds_1, 900):
                MODE = "CLOSE"
                leftSpeed = 0
                rightSpeed = 0

                ball_clamping = True
                canon_time = round(robot.getTime(), 3)
                continue
            offset = (ball[1] - cam_middle)/speed_factor
            if offset > 1:
                # make wheels turn to adjust
                rightSpeed -= offset
            elif offset < 1:
                leftSpeed += offset
            else:
                pass
        case "CLOSE":
            # save the moment the bot starts grabbing ball, just to make sure it doesnt break
            if ball_clamping and round(robot.getTime(), 3) > canon_time + 5:
                ball_clamping = False
                canon_time = round(robot.getTime(), 3)
                MODE = "RESET"
                continue
            else:
                print("Close")
                leftSpeed = 0
                rightSpeed = 0
                if is_detected(ds_2, 1000):
                    MODE = "GOAL"
                    door_right.setVelocity(0)
                    door_left.setVelocity(0)
                door_left.setVelocity(-3)
                door_right.setVelocity(3)
        case "GOAL":
            if not is_detected(ds_1, 900):
                canon_time = round(robot.getTime(), 3)
                MODE = "RESET"
                continue
            # turn to find the goal
            if hole == 0:
                leftSpeed = -5
                rightSpeed = 5
            else:
                offset = (hole[1] - cam_middle)/speed_factor
                print(f"Goal offset: {offset}")
                if offset > 1:
                    # make wheels turn to adjust
                    rightSpeed -= offset
                elif offset < 1:
                    leftSpeed += offset
                else:
                    pass

        case "RESET":
            print("RESETTING")
            # set doors back to original position
            if canon_time + 3 > round(robot.getTime(), 3):
                door_left.setPosition(0)
                door_right.setPosition(0)
                leftSpeed = 0
                rightSpeed = 0
            # restore bot to search behavior
            else:
                door_left.setPosition(float('inf'))
                door_right.setPosition(float('inf'))
                door_right.setVelocity(0)
                door_left.setVelocity(0)
                MODE = "SEARCH"


    clamped_speed = clamp(leftSpeed, rightSpeed)

    wheels[0].setVelocity(clamped_speed[0])
    wheels[1].setVelocity(clamped_speed[1])
    
    #door_left.setVelocity(-2)
    #door_right.setVelocity(2)
