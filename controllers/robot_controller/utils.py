import math

from vector import Vector

EPS = 0.00001
debug = True


def bound(value, low, high):
    return max(low, min(high, value))


def to_rad(deg):
    return (deg / 180) * math.pi


def to_deg(deg):
    return (deg * 180) / math.pi


def wrap360(degrees):
    if 0 <= degrees and degrees < 360:
        return degrees
    return (degrees % 360 + 360) % 360


def wrap180(degrees):
    if -180 < degrees and degrees <= 180:
        return degrees
    return (degrees + 540) % 360 - 180


def wrap90(degrees):
    if -90 <= degrees and degrees <= 90:
        return degrees
    return abs((degrees % 360 + 270) % 360 - 180) - 90


def init(robot, timestep):
    # right motor ---------------------------------------

    rightMotor = robot.getDevice("rightMotor")
    rightMotor.setPosition(float("inf"))

    # left motor ----------------------------------------

    leftMotor = robot.getDevice("leftMotor")
    leftMotor.setPosition(float("inf"))

    # Lidar----------------------------------------------

    lidar = robot.getDevice("Lidar")
    lidar.enable(timestep)

    # right Distance Sensor -----------------------------

    rightDisSensor = robot.getDevice("rightDistanceSensor")
    rightDisSensor.enable(timestep)

    # left Distance Sensor ------------------------------

    leftDisSensor = robot.getDevice("leftDistanceSensor")
    leftDisSensor.enable(timestep)

    # Lidar----------------------------------------------

    lidar = robot.getDevice("Lidar")
    lidar.enable(timestep)

    # GPS------------------------------------------------

    gps = robot.getDevice("GPS")
    gps.enable(timestep)

    # Receiver-------------------------------------------

    receiver = robot.getDevice("Receiver")
    receiver.enable(timestep)

    # compass--------------------------------------------

    compass = robot.getDevice("Compass")
    compass.enable(timestep)
    # ---------------------------------------------------

    # return all sensors
    return [rightMotor, leftMotor, lidar, rightDisSensor, leftDisSensor, gps, receiver, compass]


def get_geometry_values(sensors):
    lidar = sensors[2]
    rightDisSensor = sensors[3]
    leftDisSensor = sensors[4]
    gps = sensors[5]
    receiver = sensors[6]
    compass = sensors[7]

    # dequeue received messages
    tLoc = [0, 0]
    while receiver.getQueueLength() > 0:
        message = receiver.getData().decode("utf-8")
        tLoc = list(map(float, message.split(",")))
        receiver.nextPacket()
    targetLocation = Vector(tLoc[0], tLoc[2])

    # get robot angle and shift angle by 90
    compassValue = compass.getValues()
    angle = math.atan2(compassValue[0], compassValue[2]) + math.pi / 2

    # create vector from angle and size 1
    robotAngle = Vector.from_angle_size(angle, 1)

    # get robot location
    loc = gps.getValues()
    robotLocation = Vector(loc[0], loc[2])

    # get location to target vector
    robotToTargetVec = targetLocation - robotLocation

    # get angle to target vector
    robotToTargetAngle = robotAngle.angle_to(robotToTargetVec)

    # Distance to wall----------------------------------
    leftDisValue = leftDisSensor.getValue()
    rightDisValue = rightDisSensor.getValue()
    lidarValues = lidar.getRangeImage()

    return (
        robotLocation,
        robotAngle,
        targetLocation,
        robotToTargetVec,
        robotToTargetAngle,
        leftDisValue,
        rightDisValue,
        lidarValues,
    )


def debug_console(values):
    if not debug:
        return
    ####################################################
    # output console-------------------------------------
    print("robot: ", values[0], " angle: ", values[1].angle_deg())
    print(
        "target: ",
        values[2],
    )
    print("robot to target  : ", values[3], " angle: ", values[3].angle_deg(), " diff angle: ", values[4])
    print("L_Dis_Value :", values[5], "  R_Dis_Value : ", values[6])
    print("Lidar Sensor Values", values[7])
    print("-------------------------------------")

    ####################################################
