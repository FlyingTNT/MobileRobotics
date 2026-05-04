"""
Cyan
"""

from controller import Robot, Camera
import math

robot    = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
for m in (left_motor, right_motor):
    m.setPosition(float('inf'))
    m.setVelocity(0)

gps     = robot.getDevice("gps")
compass = robot.getDevice("compass")
camera  = robot.getDevice("camera")
gps.enable(timestep)
compass.enable(timestep)
camera.enable(timestep)

CAM_W = camera.getWidth()
CAM_H = camera.getHeight()

MAX_SPEED  = 6.0
FAST       = 6.0
APPROACH   = 2.0
TURN_SPEED = 3.5
SLOW_TURN  = 1.2
ANGLE_TOL  = 0.06
ARRIVE_DIST= 0.30

CYAN_H = (160, 200); CYAN_S = 100; CYAN_V = 100
BLUE_H = (210, 260); BLUE_S =  80; BLUE_V =  50

SCAN_WP1 = (2.0,  0.5)
SCAN_WP2 = (1.0,  1.5)

# Wall boundaries - if robot GPS goes past these, it's stuck on a wall
WALL_X_MIN, WALL_X_MAX = -6.3,  2.7
WALL_Y_MIN, WALL_Y_MAX = -3.0,  5.7

def set_speeds(l, r):
    left_motor.setVelocity( max(-MAX_SPEED, min(MAX_SPEED, l)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, r)))

def get_pos():
    v = gps.getValues(); return (v[0], v[1])

def get_heading():
    v = compass.getValues()
    return math.atan2(v[0], v[1]) - math.pi / 2

def dist(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def signed_angle_to(pos, hdg, target):
    d = math.atan2(target[1]-pos[1], target[0]-pos[0]) - hdg
    while d >  math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return d

def near_wall():
    p = get_pos()
    return (p[0] < WALL_X_MIN or p[0] > WALL_X_MAX or
            p[1] < WALL_Y_MIN or p[1] > WALL_Y_MAX)

def rgb_to_hsv(r, g, b):
    r,g,b = r/255.,g/255.,b/255.
    mx,mn = max(r,g,b),min(r,g,b); df=mx-mn
    if mx==0: return 0,0,0
    s=df/mx
    if df==0: h=0.
    elif mx==r: h=(60*((g-b)/df)+360)%360
    elif mx==g: h=(60*((b-r)/df)+120)%360
    else:       h=(60*((r-g)/df)+240)%360
    return h,s*255,mx*255

def scan_color(h_min, h_max, s_min, v_min):
    img = camera.getImage()
    if img is None: return None
    count=0; x_sum=0
    for py in range(0, CAM_H, 3):
        for px in range(0, CAM_W, 3):
            h,s,v = rgb_to_hsv(
                Camera.imageGetRed(img,   CAM_W, px, py),
                Camera.imageGetGreen(img, CAM_W, px, py),
                Camera.imageGetBlue(img,  CAM_W, px, py))
            if h_min<=h<=h_max and s>=s_min and v>=v_min:
                count+=1; x_sum+=px
    total=(CAM_W//3)*(CAM_H//3)
    if count==0: return None
    return (x_sum/count - CAM_W/2)/(CAM_W/2), count/total

def gps_drive_to(target, speed):
    pos=get_pos(); hdg=get_heading()
    if dist(pos,target)<ARRIVE_DIST:
        set_speeds(0,0); return True
    err=signed_angle_to(pos,hdg,target)
    if abs(err)>ANGLE_TOL:
        if err>0: set_speeds(-TURN_SPEED, TURN_SPEED)
        else:     set_speeds( TURN_SPEED,-TURN_SPEED)
    else:
        set_speeds(FAST-err*2.5, FAST+err*2.5)
    return False

GOTO_SCAN  = 0
FIND_CYAN  = 1
AIM_CYAN   = 2
APPROACH   = 3
BACK_UP    = 4
FIND_BLUE  = 5
AIM_BLUE   = 6
PUSH       = 7
WALL_ESCAPE= 8
DONE       = 9

state        = GOTO_SCAN
scan_step    = 0
aim_stable   = 0
stuck_counter= 0
timer        = 0
prev_pos     = None
attempt      = 0
prev_state   = GOTO_SCAN

def retry(reason):
    global state, scan_step, aim_stable, stuck_counter, timer, prev_pos, attempt
    print(f"RETRY #{attempt+1}: {reason}")
    attempt     += 1
    state        = GOTO_SCAN
    scan_step    = 0
    aim_stable   = 0
    stuck_counter= 0
    timer        = 0
    prev_pos     = None

print("Cyan robot v16 starting...")

while robot.step(timestep) != -1:
    pos=get_pos(); hdg=get_heading()

    if state == DONE:
        set_speeds(0,0); continue

    if state == WALL_ESCAPE:
        set_speeds(-4.0, -4.0)
        timer -= 1
        if timer <= 0:
            retry("escaped wall")
        continue

  
    if state == GOTO_SCAN:
        wp = SCAN_WP1 if scan_step==0 else SCAN_WP2
        if gps_drive_to(wp, FAST):
            if scan_step==0:
                scan_step=1
            else:
                print(f"At scan pos (attempt {attempt+1}). Searching...")
                state=FIND_CYAN
        continue

    if state == FIND_CYAN:
        result = scan_color(*CYAN_H, CYAN_S, CYAN_V)
        if result:
            aim_stable=0; state=AIM_CYAN
        else:
            set_speeds(-SLOW_TURN, SLOW_TURN)
        continue

   
    if state == AIM_CYAN:
        result = scan_color(*CYAN_H, CYAN_S, CYAN_V)
        if result is None:
            set_speeds(-SLOW_TURN, SLOW_TURN)
            state=FIND_CYAN; continue
        cx, fill = result
        if abs(cx) < 0.08:
            aim_stable+=1; set_speeds(0,0)
            if aim_stable>=8:
                print("Aimed at cyan — approaching...")
                state=APPROACH
                prev_pos=pos; stuck_counter=0
        else:
            aim_stable=0
            if cx>0: set_speeds( SLOW_TURN,-SLOW_TURN)
            else:    set_speeds(-SLOW_TURN, SLOW_TURN)
        continue

    if state == APPROACH:
        if near_wall():
            print("Hit wall during approach!")
            state=WALL_ESCAPE; timer=40; continue

        result = scan_color(*CYAN_H, CYAN_S, CYAN_V)
        if result is None:
            retry("lost cyan during approach (overshot)")
            continue

        cx, fill = result
        if abs(cx) > 0.12:
            if cx>0: set_speeds( SLOW_TURN*0.6,-SLOW_TURN*0.6)
            else:    set_speeds(-SLOW_TURN*0.6, SLOW_TURN*0.6)
        else:
            set_speeds(APPROACH, APPROACH)

        if fill > 0.18:
            print(f"Contact! fill={fill:.2f} — backing up...")
            state=BACK_UP; timer=90; continue

        if prev_pos is not None:
            if dist(pos, prev_pos) < 0.005:
                stuck_counter+=1
                if stuck_counter > 25:
                    print("Contact via GPS stuck — backing up...")
                    state=BACK_UP; timer=90; continue
            else:
                stuck_counter=0
        prev_pos=pos
        continue

    if state == BACK_UP:
        set_speeds(-3.5, -3.5)
        timer-=1
        if timer<=0:
            set_speeds(0,0)
            result = scan_color(*CYAN_H, CYAN_S, CYAN_V)
            if result is None:
                retry("cyan not visible after backup")
            else:
                print("Good position. Finding blue hole...")
                aim_stable=0; state=FIND_BLUE
        continue

    if state == FIND_BLUE:
        if near_wall():
            state=WALL_ESCAPE; timer=40; continue
        result = scan_color(*BLUE_H, BLUE_S, BLUE_V)
        if result:
            aim_stable=0; state=AIM_BLUE
            print("Blue hole found!")
        else:
            set_speeds(-SLOW_TURN, SLOW_TURN)
        continue

    if state == AIM_BLUE:
        if near_wall():
            state=WALL_ESCAPE; timer=40; continue
        result = scan_color(*BLUE_H, BLUE_S, BLUE_V)
        if result is None:
            state=FIND_BLUE; continue
        cx, fill = result
        if abs(cx) < 0.08:
            aim_stable+=1; set_speeds(0,0)
            if aim_stable>=8:
                cyan = scan_color(*CYAN_H, CYAN_S, CYAN_V)
                if cyan is None:
                    retry("cyan disappeared before push")
                else:
                    print("Locked on — PUSHING!")
                    timer=220; state=PUSH
        else:
            aim_stable=0
            if cx>0: set_speeds( SLOW_TURN,-SLOW_TURN)
            else:    set_speeds(-SLOW_TURN, SLOW_TURN)
        continue

 
    if state == PUSH:
        # Hit wall during push = overshot, retry
        if near_wall():
            print("Hit wall during push — retrying...")
            state=WALL_ESCAPE; timer=50; continue

        # Steer toward blue hole using camera the whole time
        blue = scan_color(*BLUE_H, BLUE_S, BLUE_V)
        if blue:
            correction = blue[0] * 2.0
            set_speeds(FAST-correction, FAST+correction)
        else:
            # Lost blue hole view (ball blocking it) — just drive straight
            set_speeds(FAST, FAST)

        timer-=1
        if timer<=0:
            print("Push complete!")
            set_speeds(0,0)
            state=DONE
        continue
