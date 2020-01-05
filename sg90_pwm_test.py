'''範例 https://www.instructables.com/id/Servo-Motor-Control-With-Raspberry-Pi/'''
'''妖桐-原理 https://blog.everlearn.tw/%E7%95%B6-python-%E9%81%87%E4%B8%8A-raspberry-pi/raspberry-pi-3-mobel-3-%E5%88%A9%E7%94%A8-pwm-%E6%8E%A7%E5%88%B6%E4%BC%BA%E6%9C%8D%E9%A6%AC%E9%81%94'''
'''妖桐-範例看不太懂 https://blog.everlearn.tw/%E7%95%B6-python-%E9%81%87%E4%B8%8A-raspberry-pi/raspberry-pi-3-mobel-b-%E5%88%A9%E7%94%A8%E7%A1%AC%E9%AB%94-pwm-%E6%8E%A7%E5%88%B6%E4%BC%BA%E6%9C%8D%E9%A6%AC%E9%81%94'''
'''
plug the one coming off the 紅色電源red wire into pin #2 (5v), 
the one coming off of the 棕色接地brown into pin #6 (gnd), 
and the one coming out of the 黃色輸入yellow wire into pin #3(x)#12(o) (pwm0).
'''
import RPi.GPIO as GPIO
from time import sleep

cur_angle = 0
PWM_FREQ = 50
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
pwm=GPIO.PWM(12, PWM_FREQ) #setup PWM on pin #12 at 50Hz
pwm.start(0) # start it with 0 duty cycle so it doesn't set any angles on startup

def setAngle(angle):
    duty = ""
    if angle > 180 or angle < 0:
        print('angle out of range')
    else:
        # duty = angle / 18 + 2 # bad method
        duty = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * angle / 180)
        print('angle: '+str(angle)+'° dc: '+str(duty))
        GPIO.output(12, True)
        pwm.ChangeDutyCycle(duty)
        sleep(0.3) # at least 0.3s
        GPIO.output(12, False)
        pwm.ChangeDutyCycle(0)
    return duty
    '''
    The first line sets up a function called 'SetAngle' that we can call later in the code and give our input as an angle.
    The second line (which needs to be indented inside the function) sets a variable equal to our angle divided by 18 and 2 added like I showed above
    The third line turns on the pin for output
    The fourth line changes the duty cycle to match what we calculated
    The fifth line waits 1 second so the servo has time to make the turn. Depending on the speed of your servo you might need longer, or you might not need this long
    The sixth line turns off the pin
    And the seventh line changes the duty back to 0 so we aren't continuously sending inputs to the servo
    '''

def rotate():
    i, step = 0, 15
    while i <= 180:
        dc = setAngle(i)
        #sleep(1)
        i += step
    return

def wiper():
    try:
        while True:
            setAngle(0)
            sleep(0.5)
            setAngle(180)
            sleep(0.5)
    except KeyboardInterrupt:
        print('關閉程式')
    return

def servomotor_turn15(cur_angle):
    step = 15
    cur_angle += step
    dc = setAngle(cur_angle)
    return cur_angle

def servomotor_turn05(cur_angle):
    step = 5
    cur_angle += step
    dc = setAngle(cur_angle)
    return cur_angle

def servomotor_back05(cur_angle):
    step = -5
    cur_angle += step
    dc = setAngle(cur_angle)
    return cur_angle

def servomotor_reset0(cur_angle):
    cur_angle = 0
    setAngle(cur_angle)
    return cur_angle

def servomotor_set90(cur_angle):
    cur_angle = 90
    setAngle(cur_angle)
    return cur_angle
    
#setAngle(40) # test for a spec angle
# rotate()
# wiper()
current = 11 #0° > 105° > 0°
current = servomotor_reset0(current)
current = servomotor_turn05(current)
current = servomotor_turn05(current)

current = servomotor_reset0(current)

# At the end of your code, make sure to write
pwm.stop()
GPIO.cleanup()
