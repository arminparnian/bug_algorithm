from controller import Robot ,Compass, Motor, DistanceSensor ,Lidar, Receiver

import struct

import math

robot = Robot()

positive_infnity = float('inf')

timestep = 64

#right motor ---------------------------------------

rightMotor = robot.getDevice('rightMotor')
rightMotor.setPosition(float("inf"))

#left motor ----------------------------------------

leftMotor = robot.getDevice('leftMotor')
leftMotor.setPosition(float("inf"))

#Lidar----------------------------------------------

lidar = robot.getDevice('Lidar')
lidar.enable(timestep)

#right Distance Sensor -----------------------------

rightDisSensor = robot.getDevice('rightDistanceSensor')
rightDisSensor.enable(timestep)

#left Distance Sensor ------------------------------

leftDisSensor = robot.getDevice('leftDistanceSensor')
leftDisSensor.enable(timestep)

#Lidar----------------------------------------------

lidar = robot.getDevice('Lidar')
lidar.enable(timestep)

#GPS------------------------------------------------

gps = robot.getDevice('GPS')
gps.enable(timestep)

#Receiver-------------------------------------------

receiver = robot.getDevice('Receiver') 
receiver.enable(timestep)

#compass--------------------------------------------

compass = robot.getDevice('Compass') 
compass.enable(timestep)
#---------------------------------------------------

rightVelocity = 0
leftVelocity = 0

#---------------------------------------------------
myLocation = [0,0,0]
objLocation = [0,0,0]

while robot.step(timestep) != -1:

    #Distance to wall----------------------------------
    
    leftDisValue = leftDisSensor.getValue()
    rightDisValue = rightDisSensor.getValue()
    
    lidarValues = lidar.getRangeImage()
    ####################################################
    #data receive--------------------------------------
    
    lenQueue = receiver.getQueueLength()
    
    if lenQueue :
    
        msg = receiver.getData()
        msg = msg.decode()
        tmpList = msg.split('@')
        objLocation = [float(c) for c in tmpList[:-1]]
        #print('objLoction : ',  objLocation)
        receiver.nextPacket()
        
    ####################################################
    #get value from compass and calculat angle
    
    compassValue = compass.getValues()
    
    angle = math.atan2(compassValue[0] , compassValue[2])
    
    bearing = (angle - 1.5708) / math.pi * 180.0
  
    if bearing < 0.0 :
        bearing = bearing + 360.0
    
    bearing = 180 + bearing
    
    if bearing > 360.0 :
        bearing =  bearing - 360
        
    #calculat location and distance
    
    myLocation = gps.getValues()
        
    #print(myLocation)
     
    X_robot = myLocation[0]
    Y_robot = myLocation[2]
     
    X_target = objLocation[0]
    Y_target = objLocation[2]
    
    distance = math.sqrt(abs(X_robot-X_target)**2 + abs(Y_robot-Y_target)**2)
    
    dx = X_robot - X_target - 0.05 
    dy = Y_robot - Y_target 
    
    dt = math.atan2(dy,dx) / math.pi * 180.0
    if dt < 0.0 :
        dt = dt + 360
    
  
    diff_angle =  abs(bearing - dt)
    
    ####################################################
    #output console-------------------------------------
    print("Lidar Sensor Values",lidarValues)
    print("robot X : ",X_robot ," Y : ",Y_robot)
    print("target X : ",X_target ," Y : ",Y_target) 
    print("distance to target  : " , distance)
    print("L_Dis_Value :",leftDisValue,"  R_Dis_Value : ",rightDisValue)
    print("compass : ",compassValue)
    print("bearing :  ",bearing ," dt :" ,dt," diff_angle :  ",diff_angle)
    print("-------------------------------------") 
    
    ####################################################
    #                     MOVE  
    mode = 0
    
    for i in lidarValues:
        if i == positive_infnity:
            mode = 0
        else :
            mode = 1
            break

    if (leftDisValue < 900) and (rightDisValue > leftDisValue) and mode == 1: #turn right    
        rightVelocity = -1
        leftVelocity = 1
        mode = 1  
        
    elif (rightDisValue < 900) and (rightDisValue < leftDisValue) and mode == 1:    
        rightVelocity = 1
        leftVelocity = -1 
        mode = 1
    elif mode == 1 :
        rightVelocity = 1
        leftVelocity = 1
            

    if diff_angle > 1.0  and mode == 0:
        rightVelocity = -1
        leftVelocity = 1 
        mode == 0
    elif mode == 0:
        rightVelocity = 1
        leftVelocity = 1
        
        
    if distance < 0.3 :
        print("done !! ")
        rightVelocity = 0
        leftVelocity = 0
        mode = 2   
        
    #################################################### 
    rightMotor.setVelocity(rightVelocity)
    leftMotor.setVelocity(leftVelocity)
    
    

    
