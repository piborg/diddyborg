#!/usr/bin/env python
# coding: Latin-1

# Load library functions we want
import time
import os
import sys
import PicoBorgRev
import io
import threading
import picamera
import picamera.array
import cv2
import numpy

# Re-direct our output to standard error, we need to ignore standard out to hide some nasty print statements from pygame
sys.stdout = sys.stderr
print 'Libraries loaded'

# Global values
global running
global PBR
global camera
global processor
running = True

# Setup the PicoBorg Reverse
PBR = PicoBorgRev.PicoBorgRev()
#PBR.i2cAddress = 0x44                  # Uncomment and change the value if you have changed the board address
PBR.Init()
if not PBR.foundChip:
    boards = PicoBorgRev.ScanForPicoBorgReverse()
    if len(boards) == 0:
        print 'No PicoBorg Reverse found, check you are attached :)'
    else:
        print 'No PicoBorg Reverse at address %02X, but we did find boards:' % (PBR.i2cAddress)
        for board in boards:
            print '    %02X (%d)' % (board, board)
        print 'If you need to change the I²C address change the setup line so it is correct, e.g.'
        print 'PBR.i2cAddress = 0x%02X' % (boards[0])
    sys.exit()
#PBR.SetEpoIgnore(True)                 # Uncomment to disable EPO latch, needed if you do not have a switch / jumper
# Ensure the communications failsafe has been enabled!
failsafe = False
for i in range(5):
    PBR.SetCommsFailsafe(True)
    failsafe = PBR.GetCommsFailsafe()
    if failsafe:
        break
if not failsafe:
    print 'Board %02X failed to report in failsafe mode!' % (PBR.i2cAddress)
    sys.exit()
PBR.ResetEpo()

# Power settings
voltageIn = 12.0                        # Total battery voltage to the PicoBorg Reverse
voltageOut = 6.0                        # Maximum motor voltage

# Camera settings
imageWidth  = 320                       # Camera image width
imageHeight = 240                       # Camera image height
frameRate = 3                           # Camera image capture frame rate

# Auto drive settings
autoMaxPower = 1.0                      # Maximum output in automatic mode
autoMinPower = 0.2                      # Minimum output in automatic mode
autoMinArea = 10                        # Smallest target to move towards
autoMaxArea = 10000                     # Largest target to move towards
autoFullSpeedArea = 300                 # Target size at which we use the maximum allowed output

# Setup the power limits
if voltageOut > voltageIn:
    maxPower = 1.0
else:
    maxPower = voltageOut / float(voltageIn)
autoMaxPower *= maxPower

# Image stream processing thread
class StreamProcessor(threading.Thread):
    def __init__(self):
        super(StreamProcessor, self).__init__()
        self.stream = picamera.array.PiRGBArray(camera)
        self.event = threading.Event()
        self.terminated = False
        self.start()
        self.begin = 0

    def run(self):
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    # Read the image and do some processing on it
                    self.stream.seek(0)
                    self.ProcessImage(self.stream.array)
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()
    
    # Image processing function
    def ProcessImage(self, image):
        # Get the red section of the image
        image = cv2.medianBlur(image, 5)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV) # Swaps the red and blue channels!
        red = cv2.inRange(image, numpy.array((115, 127, 64)), numpy.array((125, 255, 255)))
        # Find the contours
        contours,hierarchy = cv2.findContours(red, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # Go through each contour
        foundArea = -1
        foundX = -1
        foundY = -1
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            cx = x + (w / 2)
            cy = y + (h / 2)
            area = w * h
            if foundArea < area:
                foundArea = area
                foundX = cx
                foundY = cy
        if foundArea > 0:
            ball = [foundX, foundY, foundArea]
        else:
            ball = None
        # Set drives or report ball status
        self.SetSpeedFromBall(ball)

    # Set the motor speed from the ball position
    def SetSpeedFromBall(self, ball):
        global PBR
        driveLeft  = 0.0
        driveRight = 0.0
        if ball:
            x = ball[0]
            area = ball[2]
            if area < autoMinArea:
                print 'Too small / far'
            elif area > autoMaxArea:
                print 'Close enough'
            else:
                if area < autoFullSpeedArea:
                    speed = 1.0
                else:
                    speed = 1.0 / (area / autoFullSpeedArea)
                speed *= autoMaxPower - autoMinPower
                speed += autoMinPower
                direction = (x - imageCentreX) / imageCentreX
                if direction < 0.0:
                    # Turn right
                    driveLeft  = speed
                    driveRight = speed * (1.0 + direction)
                else:
                    # Turn left
                    driveLeft  = speed * (1.0 - direction)
                    driveRight = speed
                print '%.2f, %.2f' % (driveLeft, driveRight)
        else:
            print 'No ball'
        PBR.SetMotor1(driveRight)
        PBR.SetMotor2(-driveLeft)

# Image capture thread
class ImageCapture(threading.Thread):
    def __init__(self):
        super(ImageCapture, self).__init__()
        self.start()

    def run(self):
        global camera
        global processor
        print 'Start the stream using the video port'
        camera.capture_sequence(self.TriggerStream(), format='bgr', use_video_port=True)
        print 'Terminating camera processing...'
        processor.terminated = True
        processor.join()
        print 'Processing terminated.'

    # Stream delegation loop
    def TriggerStream(self):
        global running
        while running:
            if processor.event.is_set():
                time.sleep(0.01)
            else:
                yield processor.stream
                processor.event.set()

# Startup sequence
print 'Setup camera'
camera = picamera.PiCamera()
camera.resolution = (imageWidth, imageHeight)
camera.framerate = frameRate
imageCentreX = imageWidth / 2.0
imageCentreY = imageHeight / 2.0

print 'Setup the stream processing thread'
processor = StreamProcessor()

print 'Wait ...'
time.sleep(2)
captureThread = ImageCapture()

try:
    print 'Press CTRL+C to quit'
    PBR.MotorsOff()
    # Loop indefinitely
    while running:
        # Change the LED to reflect the status of the EPO latch
        # We do this regularly to keep the communications failsafe test happy
        PBR.SetLed(PBR.GetEpo())
        # Wait for the interval period
        time.sleep(0.1)
    # Disable all drives
    PBR.MotorsOff()
except KeyboardInterrupt:
    # CTRL+C exit, disable all drives
    print '\nUser shutdown'
    PBR.MotorsOff()
except:
    # Unexpected error, shut down!
    e = sys.exc_info()[0]
    print
    print e
    print '\nUnexpected error, shutting down!'
    PBR.MotorsOff()
# Tell each thread to stop, and wait for them to end
running = False
captureThread.join()
processor.terminated = True
processor.join()
del camera
PBR.SetLed(True)
print 'Program terminated.'
