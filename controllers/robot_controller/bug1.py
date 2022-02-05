from controller import Robot
from controllers.robot_controller.utils import debug_console, init, get_geometry_values

robot = Robot()
timestep = 64

# init sensors
sensors = init(robot, timestep)
# extract motors
rightMotor = sensors[0]
leftMotor = sensors[1]


while robot.step(timestep) != -1:
    # get geometry values from sensors
    values = get_geometry_values(sensors)
    (
        robotLocation,
        robotAngle,
        targetLocation,
        robotToTargetVector,
        robotToTargetAngle,
        leftDisValue,
        rightDisValue,
        lidarValues,
    ) = values

    debug_console(values)
