"""Weight_meter controller."""

from controller import Supervisor

robot = Supervisor()

timestep = int(robot.getBasicTimeStep()) * 4

w_sensor = robot.getDevice('w_sensor')
w_sensor.enable(timestep)

base = 0.0101 #kg
k = 4000
n_kg = 0.101972

timer = 0
mass0 = 0.01
old_w = 0.0
dec_w = 10000000

# Initialize display
display = robot.getDevice('display')
display.setFont('Verdana', 22, True)

def setWeight(wgh):
    display.setColor(0x00FF00)
    display.setAlpha(1.0)
    display.fillRectangle(0, 0, 200, 56)
    display.setAlpha(1.0)
    display.setColor(0x404040)
    
    
    str1 = f'{wgh:7.5f} kg'
    display.drawText(str1,14,14)
    ir = display.imageCopy(0, 0, 200, 56)
    display.imagePaste(ir, 0, 0, False)



setWeight(0.0)


# Main loop:
while robot.step(timestep) != -1:

    w_val = 1000 * w_sensor.getValue()
    
    #print('Meters: %6.4f' % (w_val))
    #print('%6.4f' % (w_val))
    
    weight = abs((w_val * k / 1000) * n_kg + base)

    percent = int(weight / 0.0014) - 99
    
    #strP = f'Increase: {percent:5.0f}%'
    #robot.setLabel(0, strP, 0, 0.97, 0.06, 0x00ff00, 0, 'Lucida Console')
    
    #if int(weight*dec_w)/dec_w > int(old_w*dec_w)/dec_w:
    if timer < 700:
        timer += 1
        #print(timer)
    
    old_w = weight
    
    #if timer % 10 == 0:
    setWeight(weight)
       # wgh_field.setMFString(0, 'display.jpg')
    
    pass

# Enter here exit cleanup code.
