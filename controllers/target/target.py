from controller import Robot


robot = Robot()
timestep = 64

gps = robot.getDevice("Gps")
gps.enable(timestep)


while robot.step(timestep) != -1:

    emitter = robot.getDevice("Emitter")
    location = gps.getValues()
    msg = "".join(str(c) + "," for c in location)
    msg = str.encode(msg, "utf-8")
    emitter.send(msg)
