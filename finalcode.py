#final code- group 14
#project week 3 - 18/4/2024
#this code used only for shape detction (rectangle), we detect the code at the beginning of the track while the car stop.
#the car need to follow red and green lines
import RPi.GPIO as GPIO
import time
import numpy as np
import cv2
from picamera.array import PiRGBArray
import picamera

class Robot:
    def __init__(self):
        self.initialize_camera()
        self.initialize_gpio()
        self.color_priority = {'red': 3, 'blue': 2, 'green': 1, 'yellow': 4, 'black': 0}

    def initialize_camera(self):
        self.camera = picamera.PiCamera()
        self.camera.resolution = (640, 368)  # Adjusted to avoid rounding issues
        self.camera.framerate = 10
        self.rawCapture = PiRGBArray(self.camera, size=(640, 368))
        time.sleep(0.1)

    def initialize_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        pins = [15, 13, 11, 7, 22, 18]
        GPIO.setup(pins, GPIO.OUT)
        self.pulse1 = GPIO.PWM(22, 60)
        self.pulse2 = GPIO.PWM(18, 63)
        self.pulse1.start(40)
        self.pulse2.start(40)

    def straight(self):
        GPIO.output([15, 11], GPIO.HIGH)
        GPIO.output([13, 7], GPIO.LOW)
        self.pulse1.ChangeDutyCycle(40)
        self.pulse2.ChangeDutyCycle(40)

    def back(self):
        GPIO.output([15, 11], GPIO.LOW)
        GPIO.output([13, 7], GPIO.HIGH)
        self.pulse1.ChangeDutyCycle(35)
        self.pulse2.ChangeDutyCycle(35)

    def right(self):
        GPIO.output([15, 7], GPIO.LOW)
        GPIO.output([13, 11], GPIO.HIGH)
        self.pulse1.ChangeDutyCycle(40)
        self.pulse2.ChangeDutyCycle(50)

    def left(self):
        GPIO.output([13, 11], GPIO.LOW)
        GPIO.output([15, 7], GPIO.HIGH)
        self.pulse1.ChangeDutyCycle(50)
        self.pulse2.ChangeDutyCycle(40)

    def stop(self):
        GPIO.output([15, 13, 11, 7], GPIO.LOW)
    

    def capture_frames(self):
        try:
            for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                image = frame.array
                shape_detected, shape_name = self.detect_shapes(image)
                if shape_detected:
                    self.stop()  # Stop motors when a shape is detected
                    cv2.putText(image, shape_name, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imshow('Frame', image)
                    cv2.waitKey(3000)  # Pause  the frame for 3 seconds
                else:
                    self.process_frame(frame)
                cv2.imshow('Frame', image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                self.rawCapture.truncate(0)
        except KeyboardInterrupt:
            print("Program interrupted")
        finally:
            self.stop()
            GPIO.cleanup()
            cv2.destroyAllWindows()

    def process_frame(self, frame):
        image = frame.array
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        valid_contours = self.find_valid_contours(hsv, image)
        self.choose_direction(valid_contours, image)

    def find_valid_contours(self, hsv, image):
        masks = {
            #'blue': ([100, 75, 2], [120, 255, 255]),
            #'yellow': ([20, 100, 100], [40, 255, 255]),
            'red': ([0, 50, 20], [5, 255, 255]),
            'green': ([48, 170, 60], [90, 255, 255]),
            'black': ([0, 0, 0], [360, 255, 50])
        }
        valid_contours = []
        for color, (lower, upper) in masks.items():
            mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cn in contours:
                if cv2.contourArea(cn) > 500: #original 500
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

    def detect_shapes(self, image):
        imgBlur = cv2.GaussianBlur(image, (7, 7), 1)
        imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
        imgCanny = cv2.Canny(imgGray, 100, 200)
        kernel = np.ones((5, 5))
        imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
        contours, _ = cv2.findContours(imgDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 3000: #original 3000
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                shape = self.identify_shape(len(approx), cv2.boundingRect(approx)[2], cv2.boundingRect(approx)[3])
                return True, shape
        return False, ""

    def identify_shape(self, vertex_count, width, height):
        # Identify shape or arrow
        if vertex_count == 3:
            return "Triangle"   
        elif vertex_count == 4:
            return "Rectangle"
        elif vertex_count == 5:
            return "Pentagon"
        elif vertex_count == 6:
            return "Hexagon"
#         elif vertex_count == 7:
#             # Specific logic for arrow detection
#             cx = int(cv2.moments(contours)['m10'] / cv2.moments(contours)['m00'])
#             cy = int(cv2.moments(contours)['m01'] / cv2.moments(contours)['m00'])
#             base = max(approx, key=lambda p: np.hypot(cx - p[0][0], cy - p[0][1]))
#             tip = min(approx, key=lambda p: np.hypot(cx - p[0][0], cy - p[0][1]))
#             dx = base[0][0] - tip[0][0]
#             dy = base[0][1] - tip[0][1]
#             if abs(dx) > abs(dy):
#                  return "Arrow Left" if dx > 0 else "Arrow Right"
#             else:
#                 return "Arrow Up" if dy > 0 else "Arrow Down"
        elif vertex_count == 8:
            return "Circle"
        elif vertex_count == 9:
            return "Partial Circle"

    
    

robot = Robot()
robot.capture_frames()

