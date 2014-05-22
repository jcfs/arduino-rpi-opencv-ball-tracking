#!/usr/bin/python
import cv2
import numpy as np
import os
import math
import serial
import wiringpi2 as wiringpi
import sys

SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240

GREEN_PIN = 4
RED_PIN = 5

panServoAngle = 90
tiltServoAngle = 180

hsv_min = np.array((0, 43, 81))
hsv_max = np.array((10, 255, 255))

def find_ball(capture):
    global hsv_min
    global hsv_max
    
    Cx, Cy, W, H, X, Y = 0, 0, 0, 0, 0, 0
    maxdiag = 0
    
    ret, frame = capture.read()

    if frame is not None:
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        thresholded = cv2.inRange(hsv_frame, hsv_min, hsv_max)
        contours,hierarchy = cv2.findContours(thresholded,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w/2, y + h/2
            current_diag = math.sqrt(w*w+h*h)
            if (current_diag > maxdiag):
                maxdiag=current_diag
                Cx, Cy, W, H, X, Y=cx, cy, w, h, x, y
    else:
        print "Cannot get frame"

    return (maxdiag, Cx, Cy)
    

def update_servos(diagonal, center_x, center_y):
    global panServoAngle
    global tiltServoAngle    

    increment_pan, increment_tilt = 1, 1

    if diagonal > 10:
        # focal distance, this must be adapted
        distance = 5 * 420 / diagonal

        if distance > 30:
            thresh =  20
        else:
            thresh = 40

        if center_x > SCREEN_WIDTH - 20 or center_x < 20:
            increment_pan = 3

        if center_y > SCREEN_HEIGHT - 20 or center_y < 20:
            increment_tilt = 3

        if center_x < SCREEN_WIDTH/2-thresh:
            panServoAngle -= increment_pan
            panServoAngle = max(0, panServoAngle)
        if center_x > SCREEN_WIDTH/2+thresh:
            panServoAngle += increment_pan
            panServoAngle = min(180, panServoAngle)

        if center_y < SCREEN_HEIGHT/2-thresh:
            tiltServoAngle += increment_tilt
            tiltServoAngle = min(180, tiltServoAngle)
        if center_y > SCREEN_HEIGHT/2+thresh:
            tiltServoAngle -= increment_tilt
            tiltServoAngle = max(0, tiltServoAngle)

def send_servo_update(port):
    global panServoAngle
    global tiltServoAngle

    port.write('S')
    port.write(chr(panServoAngle))
    port.write(chr(tiltServoAngle))
    port.write('#')
       
def update_leds(diagonal):
    if (diagonal > 10):
        wiringpi.digitalWrite(GREEN_PIN, 1)
        wiringpi.digitalWrite(RED_PIN, 0)
    else:
        wiringpi.digitalWrite(GREEN_PIN, 0)
        wiringpi.digitalWrite(RED_PIN, 1)


def main(argv=None):

    #setup camera capture
    print('Setting up webcam'),
    capture = cv2.VideoCapture(0)
    capture.set(3, SCREEN_WIDTH)
    capture.set(4, SCREEN_HEIGHT)

    if (capture is not None):
        print('... OK')
    else:
        return

    # setup wiring pi and leds
    print('Setting up wiring pi'),
    wiringpi.wiringPiSetup()
    wiringpi.pinMode(GREEN_PIN, 1)
    wiringpi.pinMode(RED_PIN, 1) 
    print('... OK')

    # setup serial
    print("Setting up serial connection to Arduino"),
    port = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=3.0)
    if (port is not None):
        print('... OK')
    else:
        return 

    print("Starting object tracking") 
    while True:
        diagonal, center_x, center_y = find_ball(capture)
        update_servos(diagonal, center_x, center_y)
        send_servo_update(port)
        update_leds(diagonal)

    return

if __name__ == "__main__":
    sys.exit(main())

