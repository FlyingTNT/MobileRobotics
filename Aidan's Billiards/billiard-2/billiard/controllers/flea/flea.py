"""Flea controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, TouchSensor, Camera

def main():
    robot = Robot()
    motorFR = robot.getDevice("motorFR")
    motorFL = robot.getDevice("motorFL")
    motorBR = robot.getDevice("motorBR")
    motorBL = robot.getDevice("motorBL")
    #evilMotor = robot.getDevice("evilMotor")
    #evilerMotor = robot.getDevice("evilerMotor")
    hatSensor = robot.getDevice("hat sensor")
    camera = robot.getDevice("camera")

    if not isinstance(motorFR, Motor):
        return
    
    if not isinstance(motorFL, Motor):
        return
    
    if not isinstance(motorBR, Motor):
        return
    
    if not isinstance(motorBL, Motor):
        return
    
    #if not isinstance(evilMotor, Motor):
    #    return
    
    #if not isinstance(evilerMotor, Motor):
    #    return
    
    if not isinstance(hatSensor, TouchSensor):
        return
    
    if not isinstance(camera, Camera):
        return

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    hatSensor.enable(timestep)
    camera.enable(timestep)

    motorFR.setPosition(float("infinity"))
    motorFL.setPosition(float("infinity"))
    motorBR.setPosition(float("infinity"))
    motorBL.setPosition(float("infinity"))
    motorFR.setVelocity(0)
    motorFL.setVelocity(0)
    motorBR.setVelocity(0)
    motorBL.setVelocity(0)

    WHEEL_VEL = 50
    EVIL_VEL = 500000

    ticks = 0

    def fly():
        # Increase power for balls that are far away (likely to be near a wall)
        mult = max(ticks / 350, 1)
        go(EVIL_VEL * mult, 0)
        #evilMotor.setPosition(float("infinity"))
        #evilerMotor.setPosition(float("infinity"))
        #evilMotor.setVelocity(EVIL_VEL)
        #evilerMotor.setVelocity(EVIL_VEL)

    def goR(speed: float, turn: bool):
        motorFR.setVelocity(-speed)
        motorBR.setVelocity(-speed)

    def goL(speed: float, turn: bool):
        motorFL.setVelocity(-speed)
        motorBL.setVelocity(-speed)

    def go(absSpeed: float, direction: float):
        l = min(1, 1 + direction)
        r = min(1, 1 - direction)

        goL(l * absSpeed, direction != 0)
        goR(r * absSpeed, direction != 0)
        
    lastTurn = -1
    touchCount = 0

    while robot.step(timestep) != -1:
        ticks += 1

        cam = analyzeCam(camera)
        turn = None if cam is None else cam[0]
        width = None if cam is None else cam[1]
        if turn is not None:
            turn = -turn
        
        
        if turn is not None:
            lastTurn = 1 if turn > 0 else -1

        if turn is None:
            go(50, 0.5 * lastTurn)
        else:
            go(WHEEL_VEL, turn)

        #print(turn)
        
        if hatSensor.getValue() > 0:
            touchCount += 1
        else:
            touchCount = 0
            
        #print(touchCount)

        if ticks > 10 and touchCount >= 1 and (touchCount >= 2 or (width is not None and width >= camera.getWidth() - 3)):
            fly()

def analyzeCam(camera: Camera) -> tuple[float, int] | None:
    image = camera.getImage()

    left = 1000
    right = 0

    for x in range(0, camera.getWidth()):
        r = camera.imageGetRed(image, camera.getWidth(), x, camera.getHeight() // 2 - 2)
        g = camera.imageGetGreen(image, camera.getWidth(), x, camera.getHeight() // 2 - 2)
        b = camera.imageGetBlue(image, camera.getWidth(), x, camera.getHeight() // 2 - 2)

        if r > 75 and r < 135 and g > 30 and g < 72 and b > 130 and b < 190:
            if x < left:
                left = x
            if x > right:
                right = x

    if left == 1000:
        return None
    
    return (((left + (right - left) / 2) - camera.getWidth() / 2) / (camera.getWidth() / 2), right - left)

main()