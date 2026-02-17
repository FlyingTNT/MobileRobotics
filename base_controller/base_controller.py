"""
Basic e-puck controller:
- Drives forward continuously
- Enables camera
- Enables distance sensors
"""

from controller import Robot, DistanceSensor, Motor, Camera, Gyro, Accelerometer, PositionSensor
from typing import Literal
from math import pi, pow, isnan, sin, cos

AIM_MARGIN = 3
CORRECT_MARGIN = 4

CAREFUL_BONUS = 5

def main():

    # -----------------------------
    # Robot & Time Step
    # -----------------------------
    robot = Robot()

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

    accelWrapper = AccelerometerWrapper(accelerometer)

    gyro = robot.getDevice('gyro')

    if not isinstance(gyro, Gyro):
        return

    gyro.enable(timestep)

    gyroWrapper = GyroWrapper(gyro)

    gyroWrapper.setMode("degrees")

    for i in range(robot.getNumberOfDevices()):
        print(f"{i}: {robot.getDeviceByIndex(i).getName()} ({robot.getDeviceByIndex(i)})")

    print(f"Gyro Lookup: {gyro.getLookupTable()}")
    print(f"Accel Lookup: {accelerometer.getLookupTable()}")

    passthru = 0
    hasBeenPassthru = False

    lastPass = 0
    currentSteps = 0

    beCareful = False

    sawLeft = False
    sawRight = False

    # -----------------------------
    # Main Control Loop
    # -----------------------------
    while robot.step(timestep) != -1:
        gyroWrapper.step(timestep)
        accelWrapper.step(timestep)

        #print(f"X: {gyroWrapper.getX()}")
        #print(f"Y: {gyroWrapper.getY()}")
        #print(f"Z: {gyroWrapper.getZ()}")

        analysis = analyzeCam(camera, beCareful)

        if passthru == 0:
            sawLeft |= analysis.seesLeft
            sawRight |= analysis.seesRight

        if analysis.distance < 0 and not hasBeenPassthru:
            passthru = 8 if not beCareful else 12
            
        if analysis.distance < 0.1 and analysis.distance > 0:
            passthru = 8 if not beCareful else 12

        turn = 0.0

        if passthru > 0 or analysis.margin > CORRECT_MARGIN + (CAREFUL_BONUS if beCareful else 0):
            turn = gyroWrapper.getZ() / 90 if passthru == 0 else (8 - passthru) * gyroWrapper.getZ() / 90 / 8
        elif isnan(analysis.greenTarget):
            turn = 0.75 if sawLeft else -0.75
        else:
            turn = -analysis.greenTarget * 1.3
            hasBeenPassthru = False

        print(f"Target: {analysis.greenTarget}")
        print(f"Turn: {turn}")
        print(f"Dist: {analysis.distance * cos(gyroWrapper.getZ() * GyroWrapper.DEGREES_TO_RADS)}")
        print(f"Margin: {analysis.margin}")
        print(f"L/R: {analysis.seesLeft}/{analysis.seesRight}")
        print(f"sL/sR: {sawLeft}/{sawRight}")
        
        if turn < 0:
            rightMotor.setVelocity(FORWARD_SPEED)
            leftMotor.setVelocity(FORWARD_SPEED * (1 + turn))
        elif turn > 0:
            rightMotor.setVelocity(FORWARD_SPEED * (1 - turn))
            leftMotor.setVelocity(FORWARD_SPEED)
        else:
            rightMotor.setVelocity(FORWARD_SPEED)
            leftMotor.setVelocity(FORWARD_SPEED)
        
        if passthru > 0:
            passthru -= 1
            hasBeenPassthru = True

            if passthru == 0:
                sawLeft = False
                sawRight = False
                print(f"Pass took {currentSteps - lastPass}")
                lastPass = currentSteps
                beCareful = analysis.distance < 1.5 and analysis.distance > 0
                if beCareful:
                    print("CAREFUL CAREFUL CAREFUL")
                print(sawLeft)
        currentSteps += 1

def estimateDistance(frontLeftSensor: DistanceSensor, frontRightSensor: DistanceSensor, rotation: float) -> float:
    l = frontLeftSensor.getValue()
    r = frontRightSensor.getValue()

    print(f"{r} -> {reverseLookup(frontRightSensor)}")

    lBad = l < 0
    rBad = r < 0

    if lBad and rBad:
        return -1
    
    l *= sin(1.87 + rotation)
    r *= sin(1.27 + rotation)

    return l if rBad else r if lBad else (l + r) / 2 

def reverseLookup(sensor: DistanceSensor) -> float:
    table = sensor.getLookupTable()

    value = sensor.getValue()

    if len(table) < 6:
        return value
    
    lastRow = table[0:3]

    for i in range(3, len(table), 3):
        thisRow = table[i:i+3]

        if value > max(lastRow[1], thisRow[1]):
            lastRow = thisRow
            continue

        if value < min(lastRow[1], thisRow[1]):
            lastRow = thisRow
            continue

        slope = (lastRow[0] - thisRow[0]) / (lastRow[1] - thisRow[1])

        intercept = lastRow[0] - lastRow[1] * slope

        return slope * value + intercept
    
    return -1


class CameraAnalysis:
    def __init__(self):
        self.seesLeft = False
        self.seesRight = False
        self.greenTarget = 0.0
        self.greenCenter = 0.0
        self.margin = 0
        self.distance = 0.0

def analyzeCam(camera: Camera, careful: bool = False) -> CameraAnalysis:
    R = 0
    G = 1
    B = 2

    image = camera.getImage()

    redLow = 1000
    redHigh = 0
    redLeft = 1000
    redRight = 0

    greenLow = 1000
    greenHigh = 0
    greenLeft = 1000
    greenRight = 0

    whiteLow = 1000
    whiteHigh = 0
    whiteLeft = 1000
    whiteRight = 0

    rwhiteLow = 1000
    rwhiteHigh = 0
    rwhiteLeft = 1000
    rwhiteRight = 0

    for x in range(0, camera.getWidth()):
        for y in range(0, camera.getHeight()):
            r = camera.imageGetRed(image, camera.getWidth(), x, y)
            g = camera.imageGetGreen(image, camera.getWidth(), x, y)
            b = camera.imageGetBlue(image, camera.getWidth(), x, y)

            if r >= 180 and g >= 180 and b >= 180:
                if y < whiteLow:
                    whiteLow = y
                if y > whiteHigh:
                    whiteHigh = y
                if x < whiteLeft:
                    whiteLeft = x
                if x > whiteRight:
                    whiteRight = x
                continue

            if 70 < r and r < 75 and 75 < g and g < 82 and 95 < b and b < 104:
                if y < rwhiteLow:
                    rwhiteLow = y
                if y > rwhiteHigh:
                    rwhiteHigh = y
                if x < rwhiteLeft:
                    rwhiteLeft = x
                if x > rwhiteRight:
                    rwhiteRight = x
                continue

            if b > 50:
                continue

            #print(f"{x}, {y}: {r}, {g}, {b}")

            if r >= 150:
                if y < redLow:
                    redLow = y
                if y > redHigh:
                    redHigh = y
                if x < redLeft:
                    redLeft = x
                if x > redRight:
                    redRight = x
                continue

            if g >= 150:
                if y < greenLow:
                    greenLow = y
                if y > greenHigh:
                    greenHigh = y
                if x < greenLeft:
                    greenLeft = x
                    #print(f"{r}, {g}, {b}")
                if x > greenRight:
                    greenRight = x
                continue
    
    out = CameraAnalysis()

    print(whiteLeft)

    out.seesLeft = whiteLeft == 0
    out.seesRight = rwhiteRight == camera.getWidth() - 1

    topTop = max(redHigh, greenHigh)
    bottomBottom = min(redLow, greenLow)

    height = topTop - bottomBottom

    try:
        out.distance = 8.5638 * pow(height, -1.143)  # Got these numbers thru Excel
    except:
        out.distance = -1

    center = camera.getWidth() / 2

    #print(center)
    #print(greenLeft)
    #print(greenRight)

    out.greenCenter = greenLeft + (greenRight - greenLeft) / 2

    margin = AIM_MARGIN if AIM_MARGIN >= 0 else 1
    if careful:
        margin += CAREFUL_BONUS

    greenLeft += margin
    greenRight -= margin

    if greenLeft >= 999:
        out.greenTarget = float("nan")
        out.margin = -1000
    elif AIM_MARGIN < 0:
        out.greenTarget = (center - out.greenCenter) / camera.getWidth()
        out.margin = min(center - greenLeft + margin, greenRight - center + margin)
    elif greenLeft < center and center < greenRight:
        out.greenTarget = 0
        out.margin = min(center - greenLeft, greenRight - center) + margin
    elif center <= greenLeft:
        out.greenTarget = (center - greenLeft) / camera.getWidth()
        out.margin = center - greenLeft + margin
    elif center >= greenRight:
        out.greenTarget = (center - greenRight) / camera.getWidth()
        out.margin = greenRight - center + margin
    else:
        out.greenTarget = float("nan")
        out.margin = -1000

    out.greenCenter = greenLeft + (greenRight - greenLeft) / 2

    return out



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