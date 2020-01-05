# IoT final project: *The Welcome Robot*

Have you ever seen Japan made robot *Peper*? **Welcome Robot** has some similarities with Peper, it can detect a passerby, rotate itself and make itself facing with the passerby. The core component of Welcome Robot is a webcam, as a result, you can see the vision that Welcome Robot actually capturing on the screen.

We apply machine learning about person-detection to the detecting technique, GPIO PWM to control robot rotating. 

You may see my demo vedio in the YouTube link. Remember it can only detect a whole person with standing position, I can't tell the effect on sitting person. If you move on the left, Welcome Robot will turn to itself right side (your left side) to follow you, and vice versa. 

Everything is perfect except there's a 1 to 2 second latency between sensing and rotating, it is caused by the weak computing power of raspbarry pi. I've try my best to optimize the process of detecting and vedio output, the lagging in the begining is 6s. Hope you can enjoy my progress.

## Demos

Full Project Demo：https://youtu.be/Qy7nXLi_5VM  
SG90 Rotating Test：https://youtu.be/r-zx7K57Q54  
Person Detection Test：https://youtu.be/nfIdwzGbYoM  

## Items Used

Here's the items you need, if you want to take a try on this project.

* Raspbian Pi 3 x1
* SG90 Servomotor x1
* Cable camera/USB webcam x1
* Camera support x1
* Dupont lines x3
* FFC, Flexible flat cable x1
* T-cobbler x1
* Bread board x1

There's some items I hope to add on the robot. But due to less time, it keep still with planning section.

* PIR Infrared PIR Motion Sensor x1
* Bluetooth speaker x1

## Component Make-Up

photos

## Raspberry Pi Invironment Setting

cv2 package setup
```
sudo apt-get install python-opencv
```
See more at https://docs.opencv.org/3.4.1/d2/de6/tutorial_py_setup_in_ubuntu.html

## Start Programming

We devide the program the into 2 parts: SG90 Servomotor Controlling section and Person-detection section. If you have time, you could add PIR sensing or audio output methods.

* ### SG90 Servomotor Controlling section

> References:
> 1) https://blog.everlearn.tw/%E7%95%B6-python-%E9%81%87%E4%B8%8A-raspberry-pi/raspberry-pi-3-mobel-b-%E5%88%A9%E7%94%A8%E7%A1%AC%E9%AB%94-pwm-%E6%8E%A7%E5%88%B6%E4%BC%BA%E6%9C%8D%E9%A6%AC%E9%81%94
> 2) https://www.instructables.com/id/Servo-Motor-Control-With-Raspberry-Pi/

Here we use PWM to control SG90, we will write a function `setAngle(int:angle)` to set SG90's current angle with input value.

This function is the basic method of the program, then we will biuld
* `servomotor_right(int:step, int:cur_angle)`
* `servomotor_left(int:step, int:cur_angle)`

these 2 function act as an interface to control the servomotor.

**Step1. Import the packages we need.**
```python
# import the necessary packages
import RPi.GPIO as GPIO
from time import sleep
```

**Step2. Set up the pins and PWM prequency.**
```python
# setup PWM tools
cur_angle = 0
PWM_FREQ = 50
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
pwm=GPIO.PWM(12, PWM_FREQ) #setup PWM on pin #12 at 50Hz
```

**Step3.** Now we can create the `setAngle` method. It and accept a 0~180 input `int` , and set its duty cycle by this formula `duty = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * angle / 180)`, finally the raspberry pi will charge a currnt for 0.3s with the computed duty cycle.

The derivation of the formula is in the `References` part on the top.
![](https://i.imgur.com/IORPH2c.png)

```python
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
```

**Step4.** Write the turn left and turn right methods with `setAngle()`. We use `cur_angle`globle variable to store the current angle of SG90, so remember to return the current angle.
```python
# turn left x°
def servomotor_left(step, cur_angle):
    cur_angle += step
    dc = setAngle(cur_angle)
    return cur_angle
```

* ### Real-time person detection section

> References:
> 1) https://thedatafrog.com/en/human-detection-video/
> 2) https://github.com/opencv/open_model_zoo

With the intel NCS stick provide by our course, we can take advantage of intel's per-trended model. However, I can't figur out the usage of the model without documentary. There's only a sample with face-detection.

I supposted to use [person-detection-retail-0002](http://docs.openvinotoolkit.org/latest/person-detection-retail-0002.html) initailly but failed to complete. If you know how to call the model corractly and apply to real-time detection, please let me know. Thx.

**Step1. Access the camera/webcam.**

This step is as same as [reference 1](https://thedatafrog.com/en/human-detection-video/). Make sure you can stream with camera.
```python
import numpy as np
import cv2

cv2.startWindowThread()
cap = cv2.VideoCapture(0)

while(True):
    # reading the frame
    ret, frame = cap.read()
    # displaying the frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # breaking the loop if the user types q
        # note that the video window must be highlighted!
        break

cap.release()
cv2.destroyAllWindows()
# the following is necessary on the mac,
# maybe not on other platforms:
cv2.waitKey(1)
```

**Step2. People detecting via HOG.**

OpenCV features an implementation for a very fast human detection method, called HOG (Histograms of Oriented Gradients).

This method is trained to detect pedestrians, which are human mostly standing up, and fully visible. So do not expect it to work well in other cases.

Here I incountered a problem, while testing on my laptop, the output process is smooth and lagless. But on rapsbian, I had a 5 seconds latenency. So I had some adjust with the codes:
```python=
# resizing for faster detection
frame = cv2.resize(frame, (400, 300))
```
```python=
# sleep a while before capturing next frame
sleep(0.1)
```

Hopefully the latenency decreased to 1~2 seconds. It's more accaptable. You may view the final edition in the last chapter, or fork it from my Git.

**Step3. Determine which person to follow**

The HOG's `detectMultiScale`function can return an array of detected person's coordinates. By logic, our robot could only follow a single person, and better keep tracing the person.

Here I compute the area each detection pocesses. The bigger of the area, the closer to the robot. The nearest (biggest box) is the on that my robot should track. 

By choosing to follow the biggest box, we can also avoid some mis-detection on small objects such as T-shirts.
```python
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
```

After making sure who to follow, we can go on to `track` method, which turn 1° each call. The frame width is `0~400`, and we set x loction `150~250` as `safe zone` (middle). 

While the person to follow is in the safe zone, the robot will not rotate itself. And if the person is out of the safe zone, this method will call `servomotor_right` or `servomotor_left`to rotate the camera.
![](https://i.imgur.com/pRKVEJk.png)

```python
# track the person, move 0°/1° at one call, it takes 0.3s time.sleep()for PWM
# @return cur_angle after moving
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
```
