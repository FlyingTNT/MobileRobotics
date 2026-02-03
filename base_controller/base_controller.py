"""
Basic e-puck controller:
- Drives forward continuously
- Enables camera
- Enables distance sensors
"""

from controller import Robot, DistanceSensor, Motor, Camera, Gyro, Accelerometer
from typing import Literal
from math import pi

def main():

    # -----------------------------
    # Robot & Time Step
    # -----------------------------
    robot = Robot()

    # Get the basic time step of the current world
    timestep = int(robot.getBasicTimeStep())

    distanceSensors: list[DistanceSensor] = []

    # -----------------------------
    # Motors (Differential Drive)
    # -----------------------------
    for i in range(8):
        sensor = robot.getDevice(f"ps{i}")

        if not isinstance(sensor, DistanceSensor):
            return

        distanceSensors.append(sensor)
        sensor.enable(timestep)

    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')

    if not isinstance(leftMotor, Motor) or not isinstance(rightMotor, Motor):
        return

    # Set motors to velocity control mode
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))

    # Set a constant forward speed
    FORWARD_SPEED = 6.28 # Max = 6.28
    leftMotor.setVelocity(FORWARD_SPEED)
    rightMotor.setVelocity(FORWARD_SPEED)

    # -----------------------------
    # Camera
    # -----------------------------
    camera = robot.getDevice('camera')

    if not isinstance(camera, Camera):
        return

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

    save = True

    # -----------------------------
    # Main Control Loop
    # -----------------------------
    while robot.step(timestep) != -1:
        """
        This loop runs once every timestep.
        The robot currently:
        - Keeps moving forward
        - Reads sensors (but doesn't react yet)
        """

        if save:
            save = False
            camera.saveImage("camera.png", 100)

        # Read distance sensor values
        ps_values = [sensor.getValue() for sensor in distanceSensors]

        gyroWrapper.step(timestep)
        accelWrapper.step(timestep)

        print(gyro.getValues())

        print(f"X: {gyroWrapper.getX()}")
        print(f"Y: {gyroWrapper.getY()}")
        print(f"Z: {gyroWrapper.getZ()}")

        print("Accel")
        print(f"X: {accelWrapper.getX()}")
        print(f"Y: {accelWrapper.getY()}")
        print(f"Z: {accelWrapper.getZ()}")

        # Example debug output (optional)
        # print(ps_values)

        # Motors already set → robot keeps moving forward
        pass

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