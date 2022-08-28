import pybullet as p
import pybullet_data
import time
import math

# connect to physics simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# load the ground plane and spawn robot1. robot1 will look for robot2
plane = p.loadURDF("plane.urdf")
robot1 = p.loadURDF("robot2.urdf")

# spawn robot2 at position x=3, y=3, z=0
pos = [3,3,0]
robot2 = p.loadURDF("robot2.urdf", pos)
p.setGravity(0,0,-10)

# keep moving the robot
while True:
    # get position and orientation of robot1 and robot2
    pos_r1, orient_r1 = p.getBasePositionAndOrientation(robot1)
    pos_r2, orient_r2 = p.getBasePositionAndOrientation(robot2)

    # change orientation of robot1 from quaternion to euler and get the yaw value
    theta_r1 = p.getEulerFromQuaternion(orient_r1)[2]

    # get the Cartesian coordinate of the robot2
    x2 = pos_r2[0]
    y2 = pos_r2[1]

    # direction robot2 is facing
    angle_2 = math.atan2(x2, y2)

    # linear and angular velocity of robot 1
    lin_vel = 0
    ang_vel = 0

    # keep turning if robot1 is not facing robot2, else keep moving forward
    if (abs(angle_2 - theta_r1) > 0.2):
        lin_vel = 0
        ang_vel = 0.7
    else:
        lin_vel = 0.7
        ang_vel = 0

    # robot velocity of right and left wheel
    Vr = (2*lin_vel - ang_vel*0.3)/(2*0.04)
    Vl = (2*lin_vel + ang_vel*0.3)/(2*0.04)

    maxForce = 30

    # move the right wheel
    p.setJointMotorControl2(robot1,
                            jointIndex=1,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=Vr,
                            force=maxForce)

    # move the left wheel
    p.setJointMotorControl2(robot1,
                            jointIndex=2,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=Vl,
                            force=maxForce)

    # keep simulation running for 1./240 seconds
    p.stepSimulation()
    time.sleep(1./240)