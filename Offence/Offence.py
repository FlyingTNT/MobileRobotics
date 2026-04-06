"""Kill controller."""

from controller import Robot, Lidar, Motor, DistanceSensor, Camera, LidarPoint
from math import sin, cos, tan, isinf, isnan, sqrt, atan2, pi
from typing import Literal
import traceback
import sys
from datetime import datetime

MAX_MOTOR_SPEED = 10.0
CAMERA_WIDTH = 4000
CAMERA_FOV = 0.785
TRUE_BALL_WIDTH = 0.04 * 2

def main():
    robot = Robot()

    timestep = int(robot.getBasicTimeStep())

    for i in range(robot.getNumberOfDevices()):
        print(f"{i}: {robot.getDeviceByIndex(i).getName()} ({robot.getDeviceByIndex(i)})")

    lidar = robot.getDevice("LDS-01")
    lidarMotor1 = robot.getDevice("LDS-01_main_motor")
    lidarMotor2 = robot.getDevice("LDS-01_secondary_motor")
    camera = robot.getDevice("camera")
    frontLeftDistance = robot.getDevice("ds_front_left")
    frontRightDistance = robot.getDevice("ds_front_right")
    backDistance = robot.getDevice("ds_back")
    rightDistance = robot.getDevice("ds_right")
    leftDistance = robot.getDevice("ds_left")
    frontLeftMotor = robot.getDevice("front_left_wheel")
    frontRightMotor = robot.getDevice("front_right_wheel")
    backLeftMotor = robot.getDevice("back_left_wheel")
    backRightMotor = robot.getDevice("back_right_wheel")

    if not isinstance(lidar, Lidar):
        return
    
    if not isinstance(lidarMotor1, Motor):
        return
    
    if not isinstance(lidarMotor2, Motor):
        return

    if not isinstance(camera, Camera):
        return
    
    if not isinstance(frontLeftDistance, DistanceSensor):
        return
    
    if not isinstance(frontRightDistance, DistanceSensor):
        return
    
    if not isinstance(backDistance, DistanceSensor):
        return
    
    if not isinstance(rightDistance, DistanceSensor):
        return
    
    if not isinstance(leftDistance, DistanceSensor):
        return
    
    if not isinstance(frontLeftMotor, Motor):
        return
    
    if not isinstance(frontRightMotor, Motor):
        return
    
    if not isinstance(backLeftMotor, Motor):
        return
    
    if not isinstance(backRightMotor, Motor):
        return

    lidar.enable(timestep)
    camera.enable(timestep)

    global CAMERA_FOV, CAMERA_WIDTH
    CAMERA_FOV = camera.getFov()
    CAMERA_WIDTH = camera.getWidth()

    lidar.enablePointCloud()

    lidar.setFrequency(25.0)

    print(lidar.getFrequency())
    print(lidar.getMaxFrequency())
    print(lidar.getFov())
    print(lidar.getNumberOfLayers())
    print(lidar.getNumberOfPoints())

    print(lidar.getVerticalFov())

    lidar1Velocity = lidarMotor1.getMaxVelocity()
    lidar2Velocity = lidarMotor2.getMaxVelocity()

    frontLeftDistance.enable(timestep)
    frontRightDistance.enable(timestep)
    backDistance.enable(timestep)
    leftDistance.enable(timestep)
    rightDistance.enable(timestep)

    print(frontLeftDistance.getLookupTable())
    print(frontRightDistance.getLookupTable())
    print(backDistance.getLookupTable())
    print(leftDistance.getLookupTable())
    print(rightDistance.getLookupTable())

    lidarMotor1.setPosition(float("infinity"))
    lidarMotor2.setPosition(float("infinity"))

    frontLeftMotor.setPosition(float("infinity"))
    frontRightMotor.setPosition(float("infinity"))
    backLeftMotor.setPosition(float("infinity"))
    backRightMotor.setPosition(float("infinity"))

    def go(speed: float, turn: float):
        #print(f"Going: {speed}, {turn}")

        if abs(turn) != 2:
            curSpeed = (frontLeftMotor.getVelocity() + frontRightMotor.getVelocity() + backLeftMotor.getVelocity() + backRightMotor.getVelocity()) / 4 / MAX_MOTOR_SPEED
            speed = (curSpeed + speed) / 2

        #print(f"Going: {speed}, {turn}")

        leftSpeed = speed * (1 + turn)
        rightSpeed = speed * (1 - turn)

        if (leftSpeed < 0) != (frontLeftMotor.getVelocity() < 0) and frontLeftMotor.getVelocity() != 0:
            leftSpeed = 0
            rightSpeed = 0

        if abs(leftSpeed) > 1:
            leftSpeed /= abs(leftSpeed)

        if abs(rightSpeed) > 1:
            rightSpeed /= abs(rightSpeed)

        frontLeftMotor.setVelocity(MAX_MOTOR_SPEED * leftSpeed)
        backLeftMotor.setVelocity(MAX_MOTOR_SPEED * leftSpeed)

        frontRightMotor.setVelocity(MAX_MOTOR_SPEED * rightSpeed)
        backRightMotor.setVelocity(MAX_MOTOR_SPEED * rightSpeed)

    imu = IMU(lidar, camera, frontLeftDistance, frontRightDistance)

    close = False

    steps: list[tuple[float, float]] = []
    step = -1

    def goTo(x: float, y: float, c: bool = True):
        nonlocal close
        xDiff = x - imu.x
        yDiff = y - imu.y

        # print(f"tgt : {x}, {y}")

        dist = sqrt(xDiff ** 2 + yDiff ** 2)

        if dist > 0.1:
            close = False
        elif dist < 0.05:
            close = True

        if close and c:
            if abs(imu.angleToBall) < 0.1:
                go(0, 0)
            elif not imu.canSeeBall:
                go(0.2, -2 if -imu.getRotation() < 0 else 2)
            else:
                go(0.2 * abs(imu.angleToBall / 45), -2 if imu.angleToBall < 0 else 2)
            return

        back = xDiff < 0 # imu.canSeeBall and abs(angleDiff) > 90

        if back:
            xDiff = -xDiff
            yDiff = -yDiff

        angleTo = -atan2(yDiff, xDiff) / pi * 180

        angleDiff = getAngleDiff(imu.getRotation(), angleTo)

        #print(f"The angle to is {angleTo} and we are at {imu.getRotation()} so we're going {angleDiff}")
        
        go((-dist / 0.25 if back else dist / 0.25) if c else (-1 if back else 1), (1 if back else -1) * angleDiff / 45)

    def followSteps():
        nonlocal step, steps, task
        if step < 0 or len(steps) == 0:
            return
        
        if step >= len(steps):
            step = -1
            steps = []
            task = "defend"
            return
        
        target = steps[step]
        dist = sqrt((target[0] - imu.x) ** 2 + (target[1] - imu.y) ** 2)

        print(f"Step {target}")

        if dist < 0.05:
            step += 1
            followSteps()

        goTo(target[0], target[1], False)

    def stabilize():
        frontLeftMotor.setVelocity(MAX_MOTOR_SPEED)
        backLeftMotor.setVelocity(-MAX_MOTOR_SPEED)
        frontRightMotor.setVelocity(MAX_MOTOR_SPEED)
        backRightMotor.setVelocity(-MAX_MOTOR_SPEED)

    deadlock = 0
    lastPosX = 0
    lastPosY = 0
    stabct = 0

    pushLeft = getNondeterministicRandom()

    def dribble(bias: float):
        go(1, imu.angleToBall / 22 + bias)


    def doTask(task: Literal["stabilize", "defend", "ddefend", "kill", "findBall", "pushBall", "pinBall", "kickoff"]):
        nonlocal deadlock, lastPosX, lastPosY, stabct

        if task != "stabilize":
            stabct = 0

        match task:
            case "stabilize":
                print("Stabilizing!")
                stabct = stabct + 1
                if stabct > 10:
                    go(-1, 0)
                else:
                    stabilize()
            case "defend" | "ddefend":
                print("Defending!")
                goTo(-1, 0 if task == "defend" else imu.y)
            case "findBall":
                print("Searching for the ball!")
                if not imu.knowsWhereBallIs():
                    go(1, -2 if imu.angleToBall < 0 else 2)
                else:
                    go(0.1, -2 if imu.angleToBall < 0 else 2)
            case "pushBall":
                if not imu.knowsWhereBallIs():
                    doTask("findBall")
                else:
                    print("Attacking the ball")
                    if not imu.canFeelBall:
                        dribble(0)
                    else:
                        dribble(imu.y / 0.5 * 0.1)
            case "kill":              
                if not imu.enemySpotted:
                    print(f"Finding enemy >:)")
                    go(1, -2)
                else:
                    print(f"Murdering enemy at angle {imu.angleToEnemy} ({imu.enemyX}, {imu.enemyY})")
                    go(1, imu.angleToEnemy / 22 + 0.1)
            case "kickoff":
                if step > -1:
                    followSteps()
                else:
                    task = "defend"

    task = "defend"
    loops = 0

    attacked = False

    while robot.step(timestep) != -1:
        try:
            imu.step(timestep)
        except Exception as e:
            traceback.print_exc(file=sys.stdout)
            print(f"Error: {e}")

        if (task == "kill" or task == "pushBall"):
            if (imu.getX() - lastPosX) ** 2 + (imu.getY() - lastPosY) ** 2 < 0.01 ** 2:
                deadlock += 1
            else:
                deadlock = 0

            lastPosX = imu.getX()
            lastPosY = imu.getY()

            if deadlock > 42:
                deadlock = 0
                continue
            
            if deadlock > 40:
                go(1, -2)
                print(f"Breaking deadlock")  
                continue

        if not imu.canBeTrusted:
            doTask("stabilize")

        if imu.knowsWhereBallIs() and imu.absoluteBallX > 0.1 and task != "ddefend" and loops > 10 and task != "kickoff":
            task = "defend"

        if (imu.knowsWhereBallIs() and imu.absoluteBallX < -0.05 and loops > 10) and task != "kickoff":
            task = "pushBall" if (imu.x < imu.absoluteBallX or imu.distanceToBall < 0.2) else "ddefend"
            
        if loops > 150 and not attacked and sqrt(imu.absoluteBallX ** 2 + imu.absoluteBallY ** 2) < 0.2:
            attacked = True
            task = "kickoff"
            if step < 0:
                steps = [(-1.05, 0), (-1.05, 0), (-0.8, 0.15), (-0.6, 0.2475), (-0.4, 0.3), (-0.2, 0.25), (0.05, -0.08)]
                if getNondeterministicRandom() and False:
                    steps = [(x, -y) for x, y in steps]
                step = 0
        
        if False and not imu.knowsWhereBallIs() and (task != "kill" and task != "ddefend" or (task == "ddefend" and imu.x < -0.95)):
            task = "findBall"

        doTask(task)
        loops += 1

def getTrueDistance(sensor: DistanceSensor) -> float:
    value = sensor.getValue()

    return -1 if value == 1000.0 else value / 10000

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

def isEnemy(r, g, b):
    main = False
    other = 0

    if r < 70:
        other = 1
    elif (r > 148 and r < 163) or (r > 215 and r < 230):
        main = True
    else:
        return False
    
    if g < 72:
        other += 1
    elif not main and ((g > 157 and g < 163) or (g > 218 and g < 240)):
        main = True
    else:
        return False
    
    if b < 70 and other == 1:
        return True
    elif main:
        return False
    
    return (b > 165 and b < 175) or (b > 222 and b < 235)

def angle_offset(offset):
    """
    pixel 0 = -.785 RAD
    pixel 1024 = +0.785 RAD
    ->DEG= +/- 44.97719
    """
    cam_center = CAMERA_WIDTH/2
    pixel_offset_ratio = (offset - cam_center)/(CAMERA_WIDTH)

    rad = pixel_offset_ratio * CAMERA_FOV
    return rad / pi * 180


def BDC(pixel_width: int):
    '''
    Ball Distance Calculator finds the distance from the robot to the ball based on
    some data that I analyzed. Uses a relation between time, displacement, and the
    width of the ball seen on the camera
    d(W) = (roughly) 78.5/W, where W=width of Yellow pixels on camera
    '''

    
    if pixel_width == 0:
        return -1

    angl = CAMERA_FOV * (pixel_width / CAMERA_WIDTH)
    d = (TRUE_BALL_WIDTH / 2) / tan(angl / 2) / 2 * 0.75
    

    constant = 78.5/2
    #distance = 0.8*math.log(pixel_width/42)
    if pixel_width != 0:
        distance = constant/pixel_width

        #print(f"{d} vs {distance} ({distance / d})")

        return d
    return -1

class IMU:
    def __init__(self, lidar: Lidar, camera: Camera, distanceL: DistanceSensor, distanceR: DistanceSensor):
        self.lidar = lidar
        self.camera = camera
        self.distanceL = distanceL
        self.distanceR = distanceR
        self.rotation = 0.0
        self.spins = 0
        self.x = 0.0
        self.y = 0.0
        self.canBeTrusted = False
        self.canSeeBall = False
        self.canFeelBall = False
        self.ballImagePosition = CAMERA_WIDTH / 2
        self.angleToBall = 0
        self.distanceToBall = 0.5
        self.relativeBallX = 0.5
        self.relativeBallY = 0
        self.absoluteBallX = 0
        self.absoluteBallY = 0
        self.ballVelocityX = 0
        self.ballVelocityY = 0
        self.prevBallPositions = [Point(0.0, 0.0)]
        self.tickCount = 0
        self.wasBallClose = -1000
        self.lastBallWidth = 0
        self.enemyX = 0.5
        self.enemyY = 0
        self.enemySpotted = False
        self.angleToEnemy = 0

    def step(self, timestep: int):
        self.tickCount += 1
        image = self.camera.getImage()
        yellow_x_sum = 0
        yellow_count = 0

        enemySum = 0
        enemyCount = 0

        cam_width = self.camera.getWidth()
        cam_height = self.camera.getHeight()

        isBallCutOff = False
        
        for y in range(cam_height):
            for x in range(cam_width):
                r = Camera.imageGetRed(image, cam_width, x, y)
                g = Camera.imageGetGreen(image, cam_width, x, y)
                b = Camera.imageGetBlue(image, cam_width, x, y)
                if is_yellow(r, g, b):
                    yellow_x_sum += x
                    yellow_count += 1

                    if x == 0 or x == (CAMERA_WIDTH - 1):
                        isBallCutOff = not isBallCutOff

                if isEnemy(r, g, b):
                    enemySum += x
                    enemyCount += 1

        self.canSeeBall = yellow_count > 0

        self.canFeelBall = self.tickCount - self.wasBallClose < 2 and (getTrueDistance(self.distanceL) > 0 or getTrueDistance(self.distanceR) > 0)

        if yellow_count > CAMERA_WIDTH / 3:
            self.wasBallClose = self.tickCount

        if self.canSeeBall and abs(self.lastBallWidth - yellow_count) < 0.05 * CAMERA_WIDTH:
            self.ballImagePosition = yellow_x_sum / yellow_count
            self.angleToBall = angle_offset(self.ballImagePosition)
            angleRads = self.angleToBall * pi / 180

            if not isBallCutOff:
                self.distanceToBall = BDC(yellow_count) * 2.5

            self.relativeBallX = self.distanceToBall * sin(angleRads)
            self.relativeBallY = self.distanceToBall * cos(angleRads)
        elif self.canFeelBall:
            self.wasBallClose = self.tickCount
            dl = getTrueDistance(self.distanceL)
            dr = getTrueDistance(self.distanceR)

            if dl > 0 and dr > 0:
                self.distanceToBall = (dl + dr) / 2
                self.angleToBall = ((dl - dr) / 0.1) * 10
            elif dl > 0:
                self.distanceToBall = dl
                self.angleToBall = 10
            else:
                self.distanceToBall = dr
                self.angleToBall = -10

            angleRads = self.angleToBall * pi / 180

            self.relativeBallX = self.distanceToBall * sin(angleRads)
            self.relativeBallY = self.distanceToBall * cos(angleRads)
        elif self.tickCount - self.wasBallClose < 5:
            self.prevBallPositions.clear()

        self.lastBallWidth = yellow_count

        self.enemySpotted = enemyCount > 0

        if self.enemySpotted:
            self.angleToEnemy = angle_offset(enemySum / enemyCount)

        # LIDAR ANALYSIS
        points = [point for point in self.lidar.getLayerPointCloud(0) if not isinf(point.x)]
        discontinuities = []

        #print([(p.x, p.y) for p in points])

        last = points[0]

        relativeEnemyX = 0
        relativeEnemyY = 0

        for i in range(1, len(points)):
            this = points[i]

            if ((last.x - this.x) ** 2) + ((last.y - this.y) ** 2) > (0.2 ** 2):
                discontinuities.append(i)
            
            last = this

        if len(discontinuities) > 1:
            pa = []
            pb = []

            dl = discontinuities[0]

            for d in discontinuities[1:]:
                if d - dl < 20:
                    pb += points[dl:d]
                else:
                    pa += points[dl:d]

                dl = d

            if discontinuities[0] - (dl - len(points)) < 20:
                pb += points[dl:]
                pb += points[0:discontinuities[0]]
            else:
                pa += points[dl:]
                pa += points[0:discontinuities[0]]
            
            points = pa

            if len(discontinuities) == 2 and len(pb) > 1:
                relativeEnemyX = sum([p.x for p in pb]) / len(pb)
                relativeEnemyY = sum([p.y for p in pb]) / len(pb)

        #print(discontinuities)
        #print([(p.x, p.y) for p in points])

        walls = [item[1] for item in DouglasPeucker(points, 0, len(points) - 1, 0.05)]

        if len(walls) < 4:
            self.canBeTrusted = False
            return

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

        badData = area < 2.2 or area > 2.6

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

        cosR = cos(rotationRads)
        sinR = sin(rotationRads)

        ccX, ccY = cX * cosR - cY * sinR, cX * sinR + cY * cosR

        self.x = ccX
        self.y = ccY

        if len(discontinuities) == 2:

            relativeEnemyX = (relativeEnemyX * cosR - relativeEnemyY * sinR)
            relativeEnemyY = (relativeEnemyX * sinR + relativeEnemyY * cosR)

            relativeEnemyX -= ccX
            relativeEnemyY -= ccY
            self.enemyX = relativeEnemyX
            self.enemyY = relativeEnemyY

        if (self.canSeeBall or self.canFeelBall):
            self.absoluteBallX = self.x + (self.relativeBallX * sinR + self.relativeBallY * cosR)
            self.absoluteBallY = self.y + (self.relativeBallX * cosR - self.relativeBallY * sinR)

            if self.distanceToBall > 1.5:
                return

            self.prevBallPositions.append(Point(self.absoluteBallX, self.absoluteBallY))

            if len(self.prevBallPositions) > 1:
                if len(self.prevBallPositions) > 7:
                    self.prevBallPositions.pop(0)

                ballMovement = DouglasPeucker(self.prevBallPositions, 0, len(self.prevBallPositions) - 1, 0.03)
                #print(f"BMVMT : {[item[1] for item in ballMovement]}")
                #print(f"BMVMT : {ballMovement}")

                movementVectorStart = ballMovement[-2]
                movementVectorEnd = ballMovement[-1]
                movementX = movementVectorEnd[1].x - movementVectorStart[1].x
                movementY = movementVectorEnd[1].y - movementVectorStart[1].y
                currentMovementAngle = atan2(movementY, movementX)
                currentMovementMagnitude = sqrt(movementX ** 2 + movementY ** 2)
                currentMovementMagnitude /= (timestep * (movementVectorEnd[0] - movementVectorStart[0])) / 1000

                self.ballVelocityX = currentMovementMagnitude * cos(currentMovementAngle)
                self.ballVelocityY = currentMovementMagnitude * sin(currentMovementAngle)

    
    def getRotation(self):
        rot = 180 * self.spins + self.rotation

        while rot > 180:
            rot -= 360

        while rot < -180:
            rot += 360
        
        return rot
    
    def getX(self):
        return self.x
    
    def getY(self):
        return self.y
    
    def isBallProbablyTooCloseToSee(self):
        return self.canFeelBall or self.tickCount - self.wasBallClose < 5

    def knowsWhereBallIs(self):
        return self.canSeeBall or self.isBallProbablyTooCloseToSee()

def getAngleDiff(base: float, angle: float) -> float:
    diff = base - angle

    if abs(diff) <= 180:
        return diff
    
    if base < angle:
        return getAngleDiff(base + 360, angle)
    else:
        return getAngleDiff(base, angle + 360)
    
class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return f"({self.x}, {self.y})"
    
    def __repr__(self) -> str:
        return str(self)
    
def analyzeShape(edges: list[LidarPoint | Point]) -> tuple[float, float, float]:
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

    if area == 0:
        return (0, 0, 0)

    centroidX /= 6 * area
    centroidY /= 6 * area

    return (abs(area), centroidX, centroidY)

def DouglasPeucker(points: list[LidarPoint] | list[Point], start: int, end: int, epsilon: float) -> list[tuple[int, LidarPoint | Point]]:
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

    if (maxDistance > epsilon * sqrt(a*a + b*b)):
        recOut1 = DouglasPeucker(points, start, maxIndex, epsilon)
        recOut2 = DouglasPeucker(points, maxIndex, end, epsilon)

        outPoints = recOut1 + recOut2[1:]
    else:
        outPoints = [(start, points[start]), (end, points[end])]

    return outPoints

def getNondeterministicRandom() -> bool:
    return datetime.now().second % 2 == 0

main()