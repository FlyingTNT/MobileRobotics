"""Awesome controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Lidar, Motor, DistanceSensor, Camera, LidarPoint
from math import sin, cos, isinf, sqrt, atan2, pi


MAX_MOTOR_SPEED = 10.0

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

    imu = IMU(lidar, camera)

    while robot.step(timestep) != -1:
        imu.step(timestep)

        print(f"Data is Good: {imu.canBeTrusted}")
        print(f"Position: {imu.getX()}, {imu.getY()}")
        print(f"Rotation: {imu.getRotation()}")
        print(f"Spins: {imu.spins}")
        print(f"Fl: {getTrueDistance(frontLeftDistance)}")
        print(f"Fr: {getTrueDistance(frontRightDistance)}")
        print(f"B : {getTrueDistance(backDistance)}")
        print(f"L : {getTrueDistance(leftDistance)}")
        print(f"R : {getTrueDistance(rightDistance)}")

        go(1, 0.5)

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

        print(f"Walls: {[(point.x, point.y) for point in walls]}")

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
        
        return rot
    
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

    if (maxDistance > epsilon * sqrt(a*a + b*b)):
        recOut1 = DouglasPeucker(points, start, maxIndex, epsilon)
        recOut2 = DouglasPeucker(points, maxIndex, end, epsilon)

        outPoints = recOut1 + recOut2[1:]
    else:
        outPoints = [points[start], points[end]]

    return outPoints



main()