from controller import Supervisor
import time

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

motor = robot.getDevice("propeller")
motor.setPosition(float('inf'))
motor.setVelocity(0.0)
time.sleep(1)
robot_node = robot.getSelf()

k = 1e-6

while robot.step(timestep) != -1:

    velocity = 500
    
    motor.setVelocity(velocity)

    thrust = k * velocity * velocity

    robot_node.addForce([0, 0, thrust], False)
    