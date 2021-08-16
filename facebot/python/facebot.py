import cv2
import time
import serial
import struct
from threading import Thread
import dlib
import numpy as np
import argparse
import imutils
from imutils import face_utils

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

global detector
detector = None
global predictor
predictor = None
global rects

def rect_to_bb(rect):
	# take a bounding predicted by dlib and convert it
	# to the format (x, y, w, h) as we would normally do
	# with OpenCV
	x = rect.left()
	y = rect.top()
	w = rect.right() - x
	h = rect.bottom() - y
	# return a tuple of (x, y, w, h)
	return (x, y, w, h)

def shape_to_np(shape, dtype="int"):
	# initialize the list of (x, y)-coordinates
	coords = np.zeros((68, 2), dtype=dtype)
	# loop over the 68 facial landmarks and convert them
	# to a 2-tuple of (x, y)-coordinates
	for i in range(0, 68):
		coords[i] = (shape.part(i).x, shape.part(i).y)
	# return the list of (x, y)-coordinates
	return coords


def coords_size(point, sfactor):
    return (int(point[0]*sfactor),int(point[1]*sfactor))

def faceDetect():
    global xoff
    global yoff
    global detect
    global newTarget
    global running
    global TARGET
    global faceDim
    
    global detector
    global predictor
    global rects

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
    SFACTOR = SCREEN_WIDTH/500
    FLTOL = 20

    wmax_prev = 0
    hmax_prev = 0
    lastSeen = 0

    while running:
        # Read the frame
        _, image = cap.read()

        # Convert to grayscale
        img = imutils.resize(image, width=500)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect the faces
        rects = detector(gray, 1)

        wmax = 0.0
        hmax = 0.0
        
        for (i, rect) in enumerate(rects):
            # determine the facial landmarks for the face region, then
            # convert the facial landmark (x, y)-coordinates to a NumPy
            # array
            shape = predictor(gray, rect)
            shape = face_utils.shape_to_np(shape)

            # convert dlib's rectangle to a OpenCV-style bounding box
            # [i.e., (x, y, w, h)], then draw the face bounding box
            (x, y, w, h) = face_utils.rect_to_bb(rect)
            cv2.rectangle(image, coords_size((x, y),SFACTOR), coords_size((x + w, y + h),SFACTOR), (0, 255, 0), 2)
            # show the face number
            cv2.putText(image, "Face #{}".format(i + 1), coords_size((x - 10, y - 10), SFACTOR),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # loop over the (x, y)-coordinates for the facial landmarks
            # and draw them on the image
            for (xf, yf) in shape:
               cv2.circle(image, coords_size((xf, yf),SFACTOR), 1, (0, 0, 255), -1)
            # show the output image with the face detections + facial landmarks


            if wmax*hmax < w*h and len(shape) > FLTOL:
                wmax = int(w * SFACTOR)
                hmax = int(h * SFACTOR)
                xmax = int(x * SFACTOR)
                ymax = int(y * SFACTOR)
                
        # Draw the rectangle around biggest face
        if wmax*hmax and (wmax*hmax > TOLERANCE*wmax_prev*hmax_prev or time.time() - lastSeen > MAX_TIME):
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
            image = cv2.circle(image, facemax, radius=5, color=(255, 0, 0), thickness=1)
        else:
            detect = False
        image = cv2.circle(image, TARGET, radius=5, color=(0, 255, 0), thickness=1)
        # Display
        cv2.imshow('img', image)
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

PIDSize = 1800
DEFKP = 50/PIDSize #3.7
DEFKI = 10/PIDSize #24
DEFKD = 0.5/PIDSize #0.11

def PID(Kp, Ki, Kd, MV_bar=0):
    # initialize stored data
    e_prev = 0
    t_prev = time.time() - 1
    I = 0
    PI_D_prev = 0
    
    # initial control
    MV = MV_bar
    
    while True:
        # yield MV, wait for new t, PV, SP
        t, e = yield MV

        if not PI_D_prev:
            P = Kp*e
            Iprev = I
            I = I + Ki*e*(t - t_prev)
            MV = MV_bar + P + I
            PI_D_prev = 1
        else: 
            D = Kd*(e - e_prev)/(t - t_prev)
            PI_D_prev = 0
        
        #print("t: {0}, t_prev: {1}, I: {2}, Iprev: {3}".format(t,t_prev,I,Iprev))
        
        # update stored data for next iteration
        e_prev = e
        t_prev = t


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
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--shape-predictor", required=True,
        help="path to facial landmark predictor")
    ap.add_argument("-i", "--image", required=False,
        help="path to input image")
    args = vars(ap.parse_args())

    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor(args["shape_predictor"])

    global arduino
    arduino = serial.Serial(port='COM3', baudrate=115200)
    print(arduinoRecv())

    Thread(target = faceDetect).start()
    time.sleep(3)
    Thread(target = posAdjust).start()
    cv2.destroyAllWindows()
