#!/usr/bin/python
import numpy as np
import wiringpi2 as wiringpi
import sys, getopt, math, os, serial, cv2, time
import pid as pidlib

SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240

GREEN_PIN = 5
RED_PIN = 4

HSV_MIN = np.array((45, 100, 30))
HSV_MAX = np.array((65, 256, 256))

panServoAngle = 90
tiltServoAngle = 180

def find_ball(capture, noimage, nothreshold):
    global HSV_MIN
    global HSV_MAX
    
    Cx, Cy = 0, 0
    maxdiag = 0
    
    ret, frame = capture.read()

    frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5) 

    if frame is not None:
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        thresholded = cv2.inRange(hsv_frame, HSV_MIN, HSV_MAX)

        # Raspberry pi can't handle this without too much of a fps loss
        #thresholded = cv2.erode(thresholded, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))
        #thresholded = cv2.dilate(thresholded, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))
        #thresholded = cv2.erode(thresholded, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))
        #thresholded = cv2.dilate(thresholded, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))

        _, contours, hierarchy = cv2.findContours(thresholded.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w/2, y + h/2
            current_diag = math.sqrt(w*w+h*h)
            if (current_diag > maxdiag):
                maxdiag, Cx, Cy=current_diag, cx, cy
       
        if maxdiag > 0:
            cv2.circle(frame, (Cx, Cy), int(maxdiag), (0,255,255),5)

        if noimage == False:
            cv2.imshow("original image",frame)

        if nothreshold == False:
            cv2.imshow("thresholded image",thresholded)

        cv2.waitKey(1)


 
    else:
        print "Cannot get frame"

    return (maxdiag, Cx, Cy)
    

def update_servos(diagonal, pid_value):
    global panServoAngle
    global tiltServoAngle    

    panServoAngle += int(pid_value)
    if panServoAngle > 180:
        panServoAngle = 180
    if panServoAngle < 0:
        panServoAngle = 0

def send_servo_update(port):
    global panServoAngle
    global tiltServoAngle

    port.write('S')
    port.write(chr(panServoAngle))
    port.write(chr(180))
    port.write('#')


def send_motor_update(port, diagonal):
    global panServoAngle

    if (panServoAngle > 60 and panServoAngle < 120 and diagonal < 30 and diagonal > 5):
        port.write('M')
        port.write('F')
        port.write('@')
        port.write('#')
    else:
        port.write('M')
        port.write('S')
        port.write('@')
        port.write('#')


def read_servo_update(port):
    global panServoAngle
    
    if (port.inWaiting() > 0):
        panServoAngle = ord(port.read())
        return True
    else:
        return False

       
def update_leds(diagonal):
    if (diagonal > 0):
        wiringpi.digitalWrite(GREEN_PIN, 1)
        wiringpi.digitalWrite(RED_PIN, 0)
    else:
        wiringpi.digitalWrite(GREEN_PIN, 0)
        wiringpi.digitalWrite(RED_PIN, 1)


def usage():
    print "Usage python robot.py [options]"
    print "Option: --noarduino - Don't move the servos (skip Serial setup)"
    print "        --noleds - Don't blink the leds (skip wiring pi setup)"
    print "        --noimage - Don't show captured image"
    print "        --nothreshold - Don't show thresholded image"

def main():
    pid = pidlib.PID(0.03,0,0.001)
    pid.setPoint(0)

    noleds = False
    noarduino = False
    noimage = False
    nothreshold = False

    next_servo_update = 0

    try:
        opts, args = getopt.getopt(sys.argv[1:], None, ["help","noarduino","noleds","noimage","nothreshold"])
    except getopt.GetoptError as err:
        usage()
        sys.exit(2)

    for option, value in opts:
        if option == "--noleds":
            noleds = True
        elif option == "--noarduino":
            noarduino = True
        elif option == "--noimage":
            noimage = True
        elif option == "--nothreshold":
            nothreshold = True
        elif option == "--help":
            usage()
            sys.exit(0)

    #setup camera capture
    print('Setting up webcam'),
    capture = cv2.VideoCapture(-1)

    if (capture is not None):
        print('... OK')
    else:
        return

    # setup wiring pi and leds
    if noleds == False:
        print('Setting up wiring pi'),
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(GREEN_PIN, 1)
        wiringpi.pinMode(RED_PIN, 1) 
        print('... OK')
    else:
        print "Wiringpi setup skipped"

    # setup serial
    if noarduino == False:
        print("Setting up serial connection to Arduino"),
        port = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=3.0)
        if (port is not None):
            print('... OK')
        else:
            return 
    else:
        print "Serial setup skipped"

    print "Starting object tracking"

    frames = 0
    start_time = time.time()
    while True:

        diagonal, center_x, center_y = find_ball(capture, noimage, nothreshold)

        if diagonal > 0:        
            pid_value = pid.update(SCREEN_WIDTH/2 - center_x)
            if noarduino == False:
                update_servos(diagonal, pid_value)
                if (read_servo_update(port)):
                    next_servo_update = wiringpi.millis()+100;
                send_servo_update(port)

            if noleds == False:
                update_leds(diagonal)

        frames += 1
        currtime = time.time()
        numsecs = currtime - start_time
        fps = frames / numsecs

        sys.stdout.write("Found ball at: (%d, %d) diag=%d       "%(center_x,center_y, diagonal) + " Deviation %d     " %(90-panServoAngle)+"Current FPS: %d     \t\t\r" %fps)
        sys.stdout.flush()

    return

if __name__ == "__main__":
    sys.exit(main())

