import cv2
import time
import serial
import struct
from threading import Thread

TOLERANCE = 0.5
MAX_TIME = 5

global xoff
global yoff
global detect
detect = False
global TARGET
global faceDim
global newTarget
global running
running = True


def faceDetect():
    global xoff
    global yoff
    global detect
    global newTarget
    global running
    global TARGET
    global faceDim

    # Load the cascade
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    # To capture video from webcam. 
    cap = cv2.VideoCapture(1)
    # To use a video file as input 
    # cap = cv2.VideoCapture('filename.mp4')

    _, img = cap.read()
    SCREEN_HEIGHT, SCREEN_WIDTH = img.shape[:2]
    OFFSETX=0
    OFFSETY=0
    CENTER_X=(SCREEN_WIDTH/2)-(OFFSETX/2)
    CENTER_Y=(SCREEN_HEIGHT/2)-(OFFSETY/2)
    TARGET = (int(CENTER_X), int(CENTER_Y))

    wmax_prev = 0
    hmax_prev = 0
    lastSeen = 0

    while running:
        # Read the frame
        _, img = cap.read()

        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect the faces
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        
        wmax = 0.0
        hmax = 0.0
        for (x, y, w, h) in faces:
            if wmax*hmax < w*h:
                wmax = w
                hmax = h
                xmax = x
                ymax = y
        # Draw the rectangle around biggest face
        if wmax*hmax and (wmax*hmax > TOLERANCE*wmax_prev*hmax_prev or time.time() - lastSeen > MAX_TIME):
            cv2.rectangle(img, (xmax, ymax), (xmax+wmax, ymax+hmax), (255, 0, 0), 2)
            wmax_prev = wmax
            hmax_prev = hmax
            xmax_prev = xmax
            ymax_prev = ymax

            xcmax = (xmax + wmax / 2)
            ycmax = (ymax + hmax / 2)
            facemax =(int(xcmax), int(ycmax))
            xoff = CENTER_X - xcmax
            yoff = CENTER_Y - ycmax

            faceDim = (wmax, hmax)

            if time.time() - lastSeen > MAX_TIME:
                print("Timeout target!'")
                newTarget = True

            lastSeen = time.time()
            detect = True
            img = cv2.circle(img, facemax, radius=5, color=(255, 0, 0), thickness=1)
        else:
            detect = False
        img = cv2.circle(img, TARGET, radius=5, color=(0, 255, 0), thickness=1)
        # Display
        cv2.imshow('img', img)
        # Stop if escape key is pressed
        k = cv2.waitKey(30) & 0xff
        if k==27:
            running = False
    # Release the VideoCapture object
    cap.release()

def smallDistance():
    global faceDim
    global xoff
    global yoff
    return abs(xoff) < faceDim[0] / 8 and abs(yoff) < faceDim[1] / 8

PIDSize = 2000
DEFKP = 50/PIDSize #3.7
DEFKI = 10/PIDSize #24
DEFKD = 5/PIDSize #0.11

def PID(Kp, Ki, Kd, MV_bar=0):
    # initialize stored data
    e_prev = 0
    t_prev = time.time() - 1
    I = 0
    
    # initial control
    MV = MV_bar
    
    while True:
        # yield MV, wait for new t, PV, SP
        t, e = yield MV

        P = Kp*e
        Iprev = I
        I = I + Ki*e*(t - t_prev)
        #print("t: {0}, t_prev: {1}, I: {2}, Iprev: {3}".format(t,t_prev,I,Iprev))
        D = Kd*(e - e_prev)/(t - t_prev)
        
        MV = MV_bar + P + I + D
        
        # update stored data for next iteration
        e_prev = e
        t_prev = t

arduino = serial.Serial(port='COM3', baudrate=115200)

def arduinoSend(x, y):
    arduino.write(struct.pack('>BB',x<0,y<0))
    arduino.write(struct.pack('>BB',abs(x),abs(y)))
def arduinoRecv():
    data = arduino.readline()
    data = data.decode()
    data = data.strip()
    return data


def posAdjust():
    global xoff
    global yoff
    global detect
    global newTarget
    global running

    controllerOX = None
    controllerOY = None

    while running:
        if detect:
            if newTarget:
                print("NEW")
                controllerOX = PID(DEFKP, DEFKI, DEFKD)
                controllerOX.send(None)

                controllerOY = PID(DEFKP, DEFKI, DEFKD)
                controllerOY.send(None)
                
                newTarget = False
            
            if smallDistance():
                arduinoSend(0, 0)
                MVOX = controllerOX.send([time.time(), 0])
                MVOY = controllerOY.send([time.time(), 0])
                arduinoRecv()
                continue

            MVOX = controllerOX.send([time.time(), xoff])
            MVOY = controllerOY.send([time.time(), yoff])
            
            SMVOX = max(MVOX, -180)
            SMVOY = max(MVOY, -180)

            SMVOX = min(SMVOX, 180)
            SMVOY = min(SMVOY, 180)

            SMVOX = int(SMVOX)
            SMVOY = int(SMVOY)

            print("Command: {0} {1}, input: {2} {3}, arduino: {4} {5}".format(MVOX, MVOY, xoff, yoff, SMVOX, SMVOY))
            arduinoSend(SMVOX, SMVOY)
            arduinoRecv()

if __name__ == '__main__':
    print(arduinoRecv())
    Thread(target = faceDetect).start()
    time.sleep(3)
    Thread(target = posAdjust).start()
    cv2.destroyAllWindows()
