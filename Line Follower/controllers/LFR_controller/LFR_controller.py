from controller import Robot, DistanceSensor, Motor

robot = Robot()

time = int(robot.getBasicTimeStep())

time_step = 64
base_speed = 6


# IR sensors
ds = []
ds_names = ['ds_right','ds_mid', 'ds_left']
for i in range(3):
    ds.append(robot.getDevice(ds_names[i]))
    ds[i].enable(time_step)
    
    
# motors
wheels = []
wheel_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheel_names[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)


#PID controller
last_error = intg = diff = prop = error = 0

kp = 1.6
ki = 0
kd = 0.3


def pathCorrection(error):
    
    global last_error, intg, diff, prop
    prop = error
    intg += error
    diff = error - last_error
    
    correction = (kp * prop) + (ki * intg) + (kd * diff)
    last_error = error
    
    return correction
    
    
#Speed correction    
def setSpeed(base_speed, correction):
    
    right_Speed = base_speed + correction
    left_Speed = base_speed - correction
    
  
    if right_Speed ==  base_speed or left_Speed == base_speed:
        wheels[0].setVelocity(left_Speed)
        wheels[1].setVelocity(right_Speed)
        wheels[2].setVelocity(left_Speed)
        wheels[3].setVelocity(right_Speed)
        
        
    if right_Speed < base_speed :
        wheels[0].setVelocity(left_Speed)
        wheels[1].setVelocity(0)
        wheels[2].setVelocity(left_Speed)
        wheels[3].setVelocity(0)
        
     
    if left_Speed < base_speed :
        wheels[0].setVelocity(0)
        wheels[1].setVelocity(right_Speed)
        wheels[2].setVelocity(0)
        wheels[3].setVelocity(right_Speed)
    
    

while robot.step(time_step) != -1:
    right_ir_val = ds[0].getValue()
    mid_ir_value = ds[1].getValue()
    left_ir_value = ds[2].getValue()
  
    if left_ir_value < 500 and right_ir_val < 500 and mid_ir_value >= 500:
        error = 0
        print("Forward -> Left {0}, Middle {1}, right {2}".format(left_ir_value,mid_ir_value,right_ir_val))

    elif left_ir_value < 500 and right_ir_val >= 500 and mid_ir_value >= 500:
        error = -1
        print("Right -> Left {0}, Middle {1}, right {2}".format(left_ir_value,mid_ir_value,right_ir_val))

    elif left_ir_value >= 500 and right_ir_val < 500 and mid_ir_value >= 500:
        error = 1
        print("Left -> Left {0}, Middle {1}, right {2}".format(left_ir_value,mid_ir_value,right_ir_val))

    elif left_ir_value >= 500 and right_ir_val < 500 and mid_ir_value < 500:
        error = 1
        print("Left -> Left {0}, Middle {1}, right {2}".format(left_ir_value,mid_ir_value,right_ir_val))

    elif left_ir_value < 500 and right_ir_val >= 500 and mid_ir_value < 500:
        error = -1
        print("Right -> Left {0}, Middle {1}, right {2}".format(left_ir_value,mid_ir_value,right_ir_val))

    setSpeed(base_speed, pathCorrection(error))
         
    pass