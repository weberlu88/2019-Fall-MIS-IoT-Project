# import the necessary packages
import RPi.GPIO as GPIO
from time import sleep
import numpy as np
import cv2

# setup PWM tools
cur_angle = 0
PWM_FREQ = 50
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
pwm=GPIO.PWM(12, PWM_FREQ) #setup PWM on pin #12 at 50Hz

''' PWM SG90 methods '''
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

# set to 0°
def servomotor_set0(cur_angle):
    cur_angle = 0
    setAngle(cur_angle)
    return cur_angle

# set to 90°
def servomotor_set90(cur_angle):
    cur_angle = 90
    setAngle(cur_angle)
    return cur_angle

# turn rigth x°
def servomotor_right(step, cur_angle):
    cur_angle -= step
    dc = setAngle(cur_angle)
    return cur_angle

# turn rigth 15°
def servomotor_right15(cur_angle):
    step = 15
    cur_angle += step
    dc = setAngle(cur_angle)
    return cur_angle

# turn left x°
def servomotor_left(step, cur_angle):
    cur_angle += step
    dc = setAngle(cur_angle)
    return cur_angle

''' HOG person detection's methods '''
# tell which detection is human, by comparing the area of box.
# @return (idx, loc). idx=-1 means no detection.  
# @return (idx, loc). loc means xAvg locaton of the box.   
def getBiggestBox(boxes):
    biggest_box_idx = -1
    biggest_area = 0
    loc = 0
    i = 0 # current index

    for (xA, yA, xB, yB) in boxes:
        xLength = abs(xA - xB)
        yLength = abs(yA - yB)
        area = xLength*yLength
        if (area > biggest_area):
            biggest_area = area
            biggest_box_idx = i
            loc = (xA + xB)/2
        i += 1

    return biggest_box_idx, loc

# track the person, move 0°/1° at one call, it takes 0.3s time.sleep()
# @return cur_angle after movong
def track(loc, cur_angle):
    # frame=0~400, middle=200+-50=150~250
    safe_zone = [150, 250]
    if safe_zone[0] <= loc <= safe_zone[1]: # safe
        return cur_angle
    elif loc > safe_zone[1]: # loc>250 turn right
        print("right")
        return servomotor_right(1, cur_angle)
    elif loc < safe_zone[0]: # loc<150 turn left > angle++ let loc++
        print("left")
        return servomotor_left(1, cur_angle)
        

''' main function '''
def main( cur_angle ):

    # start it with 0 duty cycle so it doesn't set any angles on startup
    # set the camera to 90 degree
    pwm.start(0) 
    cur_angle = servomotor_set90(cur_angle)

    # initialize the HOG descriptor/person detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    cv2.startWindowThread()

    # open webcam video stream
    cap = cv2.VideoCapture(0)

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # resizing for faster detection
        frame = cv2.resize(frame, (400, 300))
        # using a greyscale picture, also for faster detection
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # detect people in the image
        # returns the bounding boxes for the detected objects
        boxes, weights = hog.detectMultiScale(frame, winStride=(8,8) )
        boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

        for (xA, yA, xB, yB) in boxes:
        
            # display the detected boxes in the colour picture
            cv2.rectangle(frame, (xA, yA), (xB, yB),
                            (0, 255, 0), 2)
            xAvg = (xA + xB)/2
            yAvg = (yA + yB)/2
            locText = str(xA) + ' ,' + str(yA)
            #print("detect xloc: "+str(xAvg))
            cv2.putText(frame, locText, (xA, yA), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (165, 42, 42), 1, cv2.LINE_AA)
        ''' end for '''

        ''' important '''
        # biggest box > human
        idx, loc = getBiggestBox(boxes)

        # track the detected person by rotating SG90 
        if idx >= 0 and loc != 0:
            cur_angle = track(loc, cur_angle)
            print("person xloc: "+str(loc))
            #pass

        # Display the resulting frame
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        sleep(0.1)

    ''' end while '''

    # When everything done, release the capture
    cap.release()
    # finally, close the window
    cv2.destroyAllWindows()
    cv2.waitKey(1)

    # At the end of your code, make sure to write
    pwm.stop()
    GPIO.cleanup()
    pass

''' execute main function '''
main( cur_angle )

