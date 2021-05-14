from controller import Robot


robot = Robot()
timestep = 64

gps = robot.getDevice('Gps')
gps.enable(timestep)
   
    
while robot.step(timestep) != -1:
    
    emitter = robot.getDevice('Emitter')
    myLocation = gps.getValues()
    #print("---------------------------------")
    #print("target said : ", myLocation )
    msg = ''.join(str(c)+'@' for c in myLocation)
    msg = str.encode(msg)
    emitter.send(msg)