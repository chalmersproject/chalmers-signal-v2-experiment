import datetime
import math
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
import imutils
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
#GPIO library for buttons
import RPi.GPIO as GPIO
# display libraries
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

#declare and instantiate display
RST = None
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)
# Initialize library.
disp.begin()
# Clear display.
disp.clear()
disp.display()
# create blank image
#use mode '1' for 1-bit color
width = disp.width
height = disp.height
image = Image.new('1', (width,height))
#Get drawing object to draw on image
draw = ImageDraw.Draw(image)
#draw a black filled box to clear the image
#to prepare for text
draw.rectangle((0,0,width,height), outline=0,fill=0)
# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height-padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0


# Load default font.
font = ImageFont.load_default()

#global variables
width = 0
height = 0
EntranceCounter = 0
ExitCounter = 0
MinCountourArea = 5000  #Adjust ths value according to your usage
BinarizationThreshold = 90  #Adjust ths value according to your usage
OffsetRefLines = 75  #Adjust ths value according to your usage

#Set GPIO pins to default board breakout
# GPIO.setmode(GPIO.BOARD)
#Set Green Button GPIO pin 37 to input pullup
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#Set Red Button GPIO pin 37 to input pullup
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#define what happens when button is pressed
def buttonPressed(channel):
    global EntranceCounter
    global ExitCounter
    if GPIO.input(26):
        print "Green button released"
    else:
        print "Green Button pressed"
        EntranceCounter = EntranceCounter + 1
        time.sleep(0.5)
    if GPIO.input(16):
        print "Red button released"
    else:
        print "Red Button pressed"
        ExitCounter = ExitCounter + 1
        time.sleep(0.5)

#Add interrupt for Green/Red Button GPIO pins
GPIO.add_event_detect(16, GPIO.BOTH, callback=buttonPressed)
GPIO.add_event_detect(26, GPIO.BOTH, callback=buttonPressed)

#Check if an object is entering in monitored zone
def CheckEntranceLineCrossing(y, CoorYEntranceLine, CoorYExitLine):
  AbsDistance = abs(y - CoorYEntranceLine)
  if ((AbsDistance <= 2) and (y < CoorYExitLine)):
		return 1
  else:
		return 0

#Check if an object in exitting from monitored zone
def CheckExitLineCrossing(y, CoorYEntranceLine, CoorYExitLine):
    AbsDistance = abs(y - CoorYExitLine)
    if ((AbsDistance <= 2) and (y > CoorYEntranceLine)):
		return 1
    else:
		return 0

#initialize the Camera
##force 640x480 webcam resolution
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)

W = 640
H = 480

#camera.set(3,640)
#camera.set(4,480)
# grab an image from the camera
ReferenceFrame = None

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #update OLED LCD
    # Draw a black filled box to clear the image.
    draw.rectangle((0,0,width,height), outline=0, fill=0)
    draw.text((x, top+8),       "Entrance Counter = " + str(EntranceCounter),  font=font, fill=255)
    draw.text((x, top+24),     "Exit Counter = " + str(ExitCounter), font=font, fill=255)
    # disp.image(image)
    disp.display()

	# grab the raw NumPy array representing the image
    image = frame.array
    vs = frame.array
#    height = np.size(vs,0)
#    width = np.size(vs,1)
    GrayFrame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    GrayFrame = cv2.GaussianBlur(GrayFrame, (21, 21), 0)

    if ReferenceFrame is None:
        ReferenceFrame = GrayFrame
        #continue

    #Background subtraction and image binarization
    FrameDelta = cv2.absdiff(ReferenceFrame, GrayFrame)
    FrameThresh = cv2.threshold(FrameDelta, BinarizationThreshold, 255, cv2.THRESH_BINARY)[1]

#    #Background subtraction and image binarization
    FrameThresh = cv2.dilate(FrameThresh, None, iterations=3)
    #cnts = cv2.findContours(FrameThresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    major = cv2.__version__.split('.')[0]
    if major == '3':
		_, contours, _ = cv2.findContours(FrameThresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    else:
		contours, _ = cv2.findContours(FrameThresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#
    QttyOfContours = 0
#
#    #plot reference lines (entrance and exit lines)
    CoorYEntranceLine = 220#(height / 2)-OffsetRefLines
    CoorYExitLine = 520#(height / 2)+OffsetRefLines
    cv2.line(image, (320, 0), (320, H), (0,0,255), 5)#blue entrance
    cv2.line(image, (420, 0), (420, H), (255,0,0), 5)#red exit

 #   cv2.line(image, (0,CoorYEntranceLine), (width,CoorYEntranceLine), (255, 0, 0), 2)
 #   cv2.line(image, (0,CoorYExitLine), (width,CoorYExitLine), (0, 0, 255), 2)
    for c in contours:
        #if a contour has small area, it'll be ignored
        if cv2.contourArea(c) < MinCountourArea:
            continue
        QttyOfContours = QttyOfContours+1

        #draw an rectangle "around" the object
        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        #find object's centroid
        CoordXCentroid = (x+x+w)/2
        CoordYCentroid = (y+y+h)/2
        ObjectCentroid = (CoordXCentroid,CoordYCentroid)
        print ObjectCentroid
        cv2.circle(image, ObjectCentroid, 1, (0, 0, 0), 5)
#        if CoordXCentroid > 220:
#            ExitCounter += 1

        if (CheckEntranceLineCrossing(CoordXCentroid,320,420)):
            EntranceCounter += 1
        if (CheckEntranceLineCrossing(CoordXCentroid,420,320)):
            ExitCounter += 1

        print "Total countours found: "+str(QttyOfContours)
    #
#    #Write entrance and exit counter values on frame and shows it
    cv2.putText(image, "Entrances: {}".format(str(EntranceCounter)), (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (250, 0, 1), 2)
    cv2.putText(image, "Exits: {}".format(str(ExitCounter)), (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
#    cv2.imshow("Original Frame", frame)


	# show the frame
    cv2.imshow("Frame", image)
    # cv2.imshow("Thresh", FrameThresh)
    key = cv2.waitKey(1) & 0xFF
 	# clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # grab the raw NumPy array representing the image
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        camera.close()
        cv2.destroyAllWindows()
        break
