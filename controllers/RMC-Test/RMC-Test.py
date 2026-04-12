# RMC-Test controller
#        ____             __             ____  ______
#       / __ \_________ _/ /_____  _____/ __ \/ ____/
#      / / / / ___/ __ `/ //_/ _ \/ ___/ / / / / __  
#     / /_/ / /  / /_/ / ,< /  __/ /  / /_/ / /_/ /  
#    /_____/_/   \__,_/_/|_|\___/_/  /_____/\____/   
#       

from controller import Supervisor
import random

# create the Robot instance.
robot = Supervisor()

robot_node = robot.getSelf()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# speed motor range
MaxSpeed = 100  # rad/s
MinSpeed =  15  # rad/s

# Initialize speed motor
speed   =  0
rotate  = -1
forward =  0

# Initialize motors
L_motor = robot.getDevice('L_motor::gear')
L_motor.setPosition(float('inf'))
L_motor.setVelocity(speed)  # stop left motor

R_motor = robot.getDevice('R_motor::gear')
R_motor.setPosition(float('inf'))
R_motor.setVelocity(speed)  # stop right motor

# Initialize ground sensors
gs = []
gsv = [0, 0, 0, 0]  # ground sensor values
gsn = [0, 0, 0, 0]  # normaliced groun sensor values
gsc = [0, 0, 0, 0]  # ground sensors colors

threshold = 300

for i in range(4):
    gss = robot.getDevice('gs' + str(i))
    gss.enable(timestep)
    gs.append(gss)


# Initialize wheel sensors
right_s = robot.getDevice('rs')
right_s.enable(timestep)

left_s = robot.getDevice('ls')
left_s.enable(timestep)

# Initialize LED
led = robot.getDevice('led')
led.set(1)

# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep)

# Initialize main_display
display = robot.getDevice('display')
display.setFont('Lucida Console', 10, True)


L_Margin = 10
T_Margin = 10
P_Spacing = 25

# FSM
FWD = 0
ST1 = 1
BWD = 2
ST2 = 3

states_RL = ['LEFT', 'STOP', 'RIGHT', 'STOP']
states_FB = ['FWD', 'STOP', 'BWD', 'STOP']

# Times
T_FWD = 4
T_BWD = 4
T_STP = 2

# Initialize state
state = FWD

# Initial time
state_start_time = robot.getTime()
speed = random.uniform(MinSpeed, MaxSpeed)

# hour:minutes:seconds.thousandths
def hms(sec):
    h = int(sec // 3600)
    m = int(sec % 3600 // 60)
    s = int(sec % 3600 % 60)
    c = (sec - int(sec)) * 100
    tm = f'{h:02d}:{m:02d}:{s:02d}.{int(c):02d}'
    return tm

# Main loop:
while robot.step(timestep) != -1:
    current_time = robot.getTime()
    elapsed = current_time - state_start_time
    
    CoM = robot_node.getCenterOfMass()
    Pos = robot_node.getPosition()
    
    #strP = f'CoM [ x: {CoM[0]-Pos[0]:7.4f},  y: {CoM[1]-Pos[1]:7.4f},  z: {CoM[2]-Pos[2]:7.4f} ]'
    #robot.setLabel(0, strP, 0, 0.88, 0.06, 0x00FF00, 0, 'Lucida Console')

    strP = f'R sesor: {right_s.getValue():6.1f}    L sesor: {left_s.getValue():6.1f}'
    robot.setLabel(1, strP, 0, 0.92, 0.06, 0x00FF00, 0, 'Lucida Console')

    strP = ''
    if right_s.getValue() > threshold:
        strS = '|█ R'
    else:
        strS = '|_ R'
        
    for i in range(4):
        gsv[i] = gs[i].getValue()
        strP = f'gs{i:d}:  {gsv[i]:6.1f}   {strP:s}'
        
        if gsv[i] > threshold:
            strS = f' █ {strS:s}'
        else:
            strS = f' _ {strS:s}'

    if left_s.getValue() > threshold:
        strS = f'L █|{strS:s}'
    else:
        strS = f'L _|{strS:s}'

    robot.setLabel(2, strP, 0, 0.96, 0.06, 0x00FF00, 0, 'Lucida Console')

    if state == FWD:
        if elapsed > T_FWD:
            speed = 0
            state = ST1
            state_start_time = current_time
    elif state == ST1:
        if elapsed > T_STP:
            speed = -random.uniform(MinSpeed, MaxSpeed)
            state = BWD
            state_start_time = current_time
    elif state == BWD:
        if elapsed > T_BWD:
            speed = 0
            state = ST2
            state_start_time = current_time
    elif state == ST2:
        if elapsed > T_STP:
            if forward == 1:
                rotate = -rotate
            speed = random.uniform(MinSpeed, MaxSpeed)
            state = FWD
            state_start_time = current_time

    L_motor.setVelocity(rotate * speed)
    R_motor.setVelocity(speed)
    
    #010310
    display.setColor(0x000000)
    display.fillRectangle(0, 0, display.width, display.height)
    display.setColor(0xFFFFFF)
    
    display.drawText('RMC-TRACER [Webots]', L_Margin, T_Margin)
    
    strP = f'Time:    {hms(current_time):s}'
    #robot.setLabel(0, strP, 0, 0.88, 0.06, 0x00FF00, 0, 'Lucida Console')
    display.drawText(strP, L_Margin, T_Margin + P_Spacing*1)

    strP = f'Elapsed: {hms(elapsed):s}'
    #robot.setLabel(1, strP, 0, 0.92, 0.06, 0x00FF00, 0, 'Lucida Console')
    display.drawText(strP, L_Margin, T_Margin + P_Spacing*2)

    if rotate == -1:
        strP = f'State:   {states_RL[state]:s}'
    else:
        strP = f'State:   {states_FB[state]:s}'
    
    display.drawText(strP, L_Margin, T_Margin + P_Spacing*3)

    strP = f'R speed: {speed:5.1f} rad/s'
    display.drawText(strP, L_Margin, T_Margin + P_Spacing*4)
    
    strP = f'L speed: {rotate * speed:5.1f} rad/s'
    display.drawText(strP, L_Margin, T_Margin + P_Spacing*5)
    
    display.drawText(strS, L_Margin, T_Margin + P_Spacing*6)


    #strP = f'State: {states[state]:s}    Motor Speed:  R {speed:6.2f}   L {-speed:6.2f}  rad/s'
    #robot.setLabel(2, strP, 0, 0.96, 0.06, 0x00FF00, 0, 'Lucida Console')

    ir = display.imageCopy(0, 0, display.width, display.height)
    display.imagePaste(ir, 0, 0, False)
    
    pass

