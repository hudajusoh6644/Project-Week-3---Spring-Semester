#colour following code- group 14
#project week 3 - 18/4/2024
#the car follow black and  all coloured lines
import RPi.GPIO as GPIO
import time
import numpy as np
import cv2
from picamera.array import PiRGBArray
import picamera

class Robot:
    def __init__(self):
        # Initialize the camera and GPIO at startup
        self.initialize_camera()
        self.initialize_gpio()
        # Priority dictionary for different colors detected
        self.color_priority = {'red': 3, 'blue': 2, 'green': 1, 'yellow': 4, 'black': 0}

    def initialize_camera(self):
        # Set up the camera with specific resolution and frame rate
        self.camera = picamera.PiCamera()
        self.camera.resolution = (320, 240)
        self.camera.framerate = 20
        self.rawCapture = PiRGBArray(self.camera, size=(320, 240))
        # Allow the camera to warm-up
        time.sleep(0.1)

    def initialize_gpio(self):
        # Set up GPIO pins for output and initialize PWM
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        pins = [15, 13, 11, 7, 22, 18]
        GPIO.setup(pins, GPIO.OUT)
        self.pulse1 = GPIO.PWM(22, 60) #ena - pwm start at 60
        self.pulse2 = GPIO.PWM(18, 63) #enb- pwm start at 63
        self.pulse1.start(40) # a duty cycle of 40%
        self.pulse2.start(40) # a duty cycle of 40%
    
    # Define movement functions with enhanced logging
    def straight(self):
        GPIO.output([15, 11], GPIO.HIGH)
        GPIO.output([13, 7], GPIO.LOW)
        self.pulse1.ChangeDutyCycle(40) #right side 
        self.pulse2.ChangeDutyCycle(40) #left side
        

    def back(self):
        GPIO.output([15, 11], GPIO.LOW)
        GPIO.output([13, 7], GPIO.HIGH)
        self.pulse1.ChangeDutyCycle(30) #right side 
        self.pulse2.ChangeDutyCycle(30) #left side

    def right(self):
        GPIO.output([15, 7], GPIO.LOW)
        GPIO.output([13, 11], GPIO.HIGH)
        self.pulse1.ChangeDutyCycle(40) #right side 
        self.pulse2.ChangeDutyCycle(50) #left side

    def left(self):
        GPIO.output([13, 11], GPIO.LOW)
        GPIO.output([15, 7], GPIO.HIGH)
        self.pulse1.ChangeDutyCycle(50) #right side 
        self.pulse2.ChangeDutyCycle(40) #left side
        

    def capture_frames(self):
        # Capture frames from the camera continuously
        try:
            for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                self.process_frame(frame)
                self.rawCapture.truncate(0)
        except KeyboardInterrupt:
            # Handle keyboard interrupt
            print("Program interrupted")
        finally:
            # Cleanup GPIO and camera on exit
            self.stop()
            GPIO.cleanup()
            cv2.destroyAllWindows()

    def process_frame(self, frame):
        # Process the image from each captured frame
        image = frame.array
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        valid_contours = self.find_valid_contours(hsv, image)
        self.choose_direction(valid_contours, image)
        cv2.imshow('Frame', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # Exit on 'q' key press
            raise KeyboardInterrupt

    def find_valid_contours(self, hsv, image):
        # Define color masks and find contours for each color
        masks = {
            #comment to make the car the colour
            'blue': ([100, 75, 2], [120, 255, 255]),
            'yellow': ([20, 100, 100], [40, 255, 255]),
            'red': ([0, 50, 20], [5, 255, 255]),
            #'green': ([48, 170, 60], [90, 255, 255]),
            'black': ([0, 0, 0], [360, 255, 50])
        }
        valid_contours = []
        for color, (lower, upper) in masks.items():
            mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cn in contours:
                if cv2.contourArea(cn) > 500:
                    valid_contours.append((cn, color))
        return valid_contours

    def choose_direction(self, valid_contours, image):
    # Determine the position of the highest priority color contour
        if valid_contours:
            chosen_contour, chosen_color = max(valid_contours, key=lambda x: self.color_priority[x[1]])
            M = cv2.moments(chosen_contour)
            cX = int(M["m10"] / M["m00"]) if M["m00"] != 0 else 0
            desired_position = image.shape[1] / 2
        
            if cX < desired_position - 12:
                position = 'line is on the left'
                self.left()
            elif cX > desired_position + 12:
                position = 'line is on the right'
                self.right()
            else:
                position = 'line is straight'
                self.straight()
                
            print(f"{chosen_color} : {position}")
        
        else:
        # Output if no line is detected
            self.back()
            print("None : no line detected")

    def stop(self):
        # Stop all motors
        GPIO.output([15, 13, 11, 7], GPIO.LOW)

robot = Robot()
robot.capture_frames()


