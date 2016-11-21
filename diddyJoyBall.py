#!/usr/bin/env python
# coding: Latin-1

# Load library functions we want
import time
import os
import sys
import pygame
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
global autoMode
global running
global PBR
global camera
global controllerLost
global processingPool
global lock
autoMode = False
running = True
controllerLost = False
processingPool = []
lock = threading.Lock()

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

# Settings for the joystick
axisUpDown = 1                          # Joystick axis to read for up / down position
axisUpDownInverted = False              # Set this to True if up and down appear to be swapped
axisLeftRight = 2                       # Joystick axis to read for left / right position
axisLeftRightInverted = False           # Set this to True if left and right appear to be swapped
buttonResetEpo = 3                      # Joystick button number to perform an EPO reset (Start)
buttonSlow = 8                          # Joystick button number for driving slowly whilst held (L2)
slowFactor = 0.5                        # Speed to slow to when the drive slowly button is held, e.g. 0.5 would be half speed
buttonFastTurn = 9                      # Joystick button number for turning fast (R2)
interval = 0.02                         # Time between updates in seconds, smaller responds faster but uses more processor time
controllerLostLoops = 20                # Number of loops without any joystick events before announcing the joystick as out of range
buttonSetAutoMode = 4                   # Joystick button number to enable automatic control (D-Pad UP)
buttonSetManualMode = 6                 # Joystick button number to enable manual control (D-Pad DOWN)

# Power settings
voltageIn = 12.0                        # Total battery voltage to the PicoBorg Reverse
voltageOut = 6.0                        # Maximum motor voltage

# Camera settings
imageWidth  = 320                       # Camera image width
imageHeight = 240                       # Camera image height
threadCount = 8                         # Number of image processing threads to run
frameRate = 30                          # Camera image capture frame rate

# Auto drive settings
autoMaxPower = 1.0                      # Maximum output in automatic mode
autoMinPower = 0.2                      # Minimum output in automatic mode
autoMinArea = 10                        # Smallest target to move towards
autoMaxArea = 30000                     # Largest target to move towards
autoFullSpeedArea = 1000                # Target size at which we use the maximum allowed output

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
        global processingPool
        global lock
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
                    # Return ourselves to the processing pool
                    with lock:
                        processingPool.append(self)
    
    # Image processing function
    def ProcessImage(self, image):
        global autoMode
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
        if autoMode:
            self.SetSpeedFromBall(ball)
        else:
            if ball:
                print 'Ball at %d,%d (%d)' % (foundX, foundY, foundArea)
            else:
                print 'No ball'
            
    # Set the motor speed from the ball position
    def SetSpeedFromBall(self, ball):
        global PBR
        global controllerLost
        driveLeft  = 0.0
        driveRight = 0.0
        if ball:
            x = ball[0]
            area = ball[2]
            if area < autoMinArea:
                print '<Too small / far>'
            elif area > autoMaxArea:
                print '<Close enough>'
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
                print '<%.2f, %.2f>' % (driveLeft, driveRight)
        else:
            print '<No ball>'
        if controllerLost:
            print '<Waiting for lost controller...>'
        else:
            PBR.SetMotor1(driveRight)
            PBR.SetMotor2(-driveLeft)

# Image capture thread
class ImageCapture(threading.Thread):
    def __init__(self):
        super(ImageCapture, self).__init__()
        self.start()

    def run(self):
        global camera
        global processingPool
        print 'Start the stream using the video port'
        camera.capture_sequence(self.TriggerStream(), format='bgr', use_video_port=True)
        print 'Terminating camera processing...'
        while processingPool:
            with lock:
                processor = processingPool.pop()
            processor.terminated = True
            processor.join()
        print 'Processing terminated.'

    # Stream delegation loop
    def TriggerStream(self):
        global running
        global processingPool
        while running:
            # Get the next available processing thread
            with lock:
                if processingPool:
                    processor = processingPool.pop()
                else:
                    processor = None
            if processor:
                # We have a thread, pass it the next frame when ready
                yield processor.stream
                processor.event.set()
            else:
                # No threads are ready, wait a while then try again
                time.sleep(0.01)

# Startup sequence
print 'Setup camera'
camera = picamera.PiCamera()
camera.resolution = (imageWidth, imageHeight)
camera.framerate = frameRate
imageCentreX = imageWidth / 2.0
imageCentreY = imageHeight / 2.0

print 'Setup the stream processing threads'
processingPool = [StreamProcessor() for i in range(threadCount)]

print 'Wait ...'
time.sleep(2)
captureThread = ImageCapture()

# Setup pygame and wait for the joystick to become available
PBR.MotorsOff()
os.environ["SDL_VIDEODRIVER"] = "dummy" # Removes the need to have a GUI window
pygame.init()
#pygame.display.set_mode((1,1))
print 'Waiting for joystick... (press CTRL+C to abort)'
while True:
    try:
        try:
            pygame.joystick.init()
            # Attempt to setup the joystick
            if pygame.joystick.get_count() < 1:
                # No joystick attached, toggle the LED
                PBR.SetLed(not PBR.GetLed())
                pygame.joystick.quit()
                time.sleep(0.5)
            else:
                # We have a joystick, attempt to initialise it!
                joystick = pygame.joystick.Joystick(0)
                break
        except pygame.error:
            # Failed to connect to the joystick, toggle the LED
            PBR.SetLed(not PBR.GetLed())
            pygame.joystick.quit()
            time.sleep(0.5)
    except KeyboardInterrupt:
        # CTRL+C exit, give up
        print '\nUser aborted'
        PBR.SetLed(True)
        sys.exit()
print 'Joystick found'
joystick.init()
PBR.SetLed(False)

try:
    print 'Press CTRL+C to quit'
    driveLeft = 0.0
    driveRight = 0.0
    hadEvent = False
    upDown = 0.0
    leftRight = 0.0
    loopsWithoutEvent = 0
    # Loop indefinitely
    while running:
        # Get the latest events from the system
        hadEvent = False
        events = pygame.event.get()
        # Handle each event individually
        for event in events:
            if event.type == pygame.QUIT:
                # User exit
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                # A button on the joystick just got pushed down
                hadEvent = True
            elif event.type == pygame.JOYAXISMOTION:
                # A joystick has been moved
                hadEvent = True
            if hadEvent:
                if joystick.get_button(buttonSetAutoMode):
                    autoMode = True
                if joystick.get_button(buttonSetManualMode):
                    autoMode = False
                if not autoMode:
                    # Read axis positions (-1 to +1)
                    if axisUpDownInverted:
                        upDown = -joystick.get_axis(axisUpDown)
                    else:
                        upDown = joystick.get_axis(axisUpDown)
                    if axisLeftRightInverted:
                        leftRight = -joystick.get_axis(axisLeftRight)
                    else:
                        leftRight = joystick.get_axis(axisLeftRight)
                    # Apply steering speeds
                    if not joystick.get_button(buttonFastTurn):
                        leftRight *= 0.5
                    # Determine the drive power levels
                    driveLeft = -upDown
                    driveRight = -upDown
                    if leftRight < -0.05:
                        # Turning left
                        driveLeft *= 1.0 + (2.0 * leftRight)
                    elif leftRight > 0.05:
                        # Turning right
                        driveRight *= 1.0 - (2.0 * leftRight)
                    # Check for button presses
                    if joystick.get_button(buttonResetEpo):
                        PBR.ResetEpo()
                    if joystick.get_button(buttonSlow):
                        driveLeft *= slowFactor
                        driveRight *= slowFactor
                    # Set the motors to the new speeds
                    PBR.SetMotor1(driveRight * maxPower)
                    PBR.SetMotor2(-driveLeft * maxPower)
        if hadEvent:
            # Reset the controller lost counter
            loopsWithoutEvent = 0
            if controllerLost:
                # We had lost the controller, we have now found it again
                if autoMode:
                    print 'Controller re-connected, auto-motion will resume'
                else:
                    print 'Controller re-connected, move joystick to resume operation'
                PBR.SetLed(False)
                controllerLost = False
        elif controllerLost:
            # Controller has been lost, pulse the LED at a regular loop count
            loopsWithoutEvent += 1
            if (loopsWithoutEvent % (controllerLostLoops / 10)) == 0:
                PBR.SetLed(not PBR.GetLed())
                # Attempt to reset the joystick module
                del joystick
                pygame.joystick.quit()
                pygame.joystick.init()
                if pygame.joystick.get_count() < 1:
                    # Controller has been disconnected, poll for reconnection
                    print 'Controller disconnected!'
                    while pygame.joystick.get_count() < 1:
                        time.sleep(interval * (controllerLostLoops / 10))
                        pygame.joystick.quit()
                        pygame.joystick.init()
                        PBR.SetLed(not PBR.GetLed())
                # Grab the joystick again
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                continue
            # Skip to the next loop after the interval
            time.sleep(interval)
            continue
        else:
            # No events this loop, check if it has been too long since we saw an event
            loopsWithoutEvent += 1
            if loopsWithoutEvent > controllerLostLoops:
                # It has been too long, disable control!
                print 'Controller lost!'
                PBR.MotorsOff()
                PBR.SetLed(True)
                controllerLost = True
                # Skip to the next loop after the interval
                time.sleep(interval)
                continue
        # Change the LED to reflect the status of the EPO latch
        PBR.SetLed(PBR.GetEpo())
        # Wait for the interval period
        time.sleep(interval)
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
while processingPool:
    with lock:
        processor = processingPool.pop()
    processor.terminated = True
    processor.join()
del camera
PBR.SetLed(True)
print 'Program terminated.'
