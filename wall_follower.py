from controller import Robot

TIME_STEP = 64
MAX_SPEED = 6.28

robot = Robot()

left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

camera = robot.getCamera("camera")
camera.enable(TIME_STEP)

radiusMod = 1
misses = 0

prox_sensors = []
for i in range(8):
    sensor = robot.getDistanceSensor(f"ps{i}")
    sensor.enable(TIME_STEP)
    prox_sensors.append(sensor)

while robot.step(TIME_STEP) != -1:
    image = camera.getImageArray()
    x = camera.getWidth() // 2
    y = camera.getHeight() // 2
    r, g, b = image[x][y]

    front_wall = prox_sensors[0].getValue() > 80

    if g > 200 and r < 80 and b < 80 and front_wall:
        left_motor.setVelocity(0.5)
        right_motor.setVelocity(0.5)
        break

    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    right_wall = prox_sensors[2].getValue() > 80
    right_corner = prox_sensors[1].getValue() > 80
    
    if misses > -1 and not right_wall:
        misses += 1
    else:
        misses = -1
        radiusMod = 1
    
    if misses % 10 == 0:
        radiusMod *= 0.9

    if front_wall:
        left_speed = -MAX_SPEED
        right_speed = MAX_SPEED
    else:
        if right_wall:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED
        else:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED / (8 * radiusMod)
        if right_corner:
            left_speed = MAX_SPEED / 8
            right_speed = MAX_SPEED

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
