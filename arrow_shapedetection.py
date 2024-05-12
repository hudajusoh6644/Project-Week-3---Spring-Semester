#shape detection code- group 14
#project week 3 - 18/4/2024
#this is shape detection code, this version is revised version from the code in project week 2
#for the shape detction the code is working fine, however for the arrow detection it is not stable
# Library imports
import cv2
import numpy as np

# Resolution of the video
frameWidth = 640
frameHeight = 360

# Size of window for video
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)

def empty(x):
    # Dummy function for trackbar
    pass

# Create a window for the trackbars
cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640, 240)
cv2.createTrackbar("Threshold1", "Parameters", 100, 255, empty)
cv2.createTrackbar("Threshold2", "Parameters", 200, 255, empty)
cv2.createTrackbar("Area", "Parameters", 3000, 3300, empty)

def getContours(imgDil, imgContours):
    contours, hierarchy = cv2.findContours(imgDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    font = cv2.FONT_HERSHEY_COMPLEX

    for cnt in contours:
        areaMin = cv2.getTrackbarPos("Area", "Parameters")
        area = cv2.contourArea(cnt)
        if area > areaMin:
            cv2.drawContours(imgContours, cnt, -1, (255, 0, 255), 7)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)

            # Identify shape or arrow
            shape = "Unknown"
            if len(approx) == 3:
                shape = "Triangle"
            elif len(approx) == 4:
                aspect_ratio = w / float(h)
                shape = "Square" if 0.95 <= aspect_ratio <= 1.05 else "Rectangle"
            elif len(approx) == 5:
                shape = "Pentagon"
            elif len(approx) == 6:
                shape = "Hexagon"
            elif len(approx) == 7:
                # Specific logic for arrow detection
                cx = int(cv2.moments(cnt)['m10'] / cv2.moments(cnt)['m00'])
                cy = int(cv2.moments(cnt)['m01'] / cv2.moments(cnt)['m00'])
                base = max(approx, key=lambda p: np.hypot(cx - p[0][0], cy - p[0][1]))
                tip = min(approx, key=lambda p: np.hypot(cx - p[0][0], cy - p[0][1]))
                dx = base[0][0] - tip[0][0]
                dy = base[0][1] - tip[0][1]
                if abs(dx) > abs(dy):
                    shape = "Arrow Left" if dx > 0 else "Arrow Right"
                else:
                    shape = "Arrow Up" if dy > 0 else "Arrow Down"
            elif len(approx) == 8:
                shape = "Circle"
            elif len(approx) == 9:
                shape = "Partial Circle"

            cv2.rectangle(imgContours, (x, y), (x + w, y + h), (0, 255, 0), 5)
            cv2.putText(imgContours, shape, (x + w + 20, y + 20), font, 0.7, (0, 0, 255), 2)
            cv2.putText(imgContours, f"Points: {len(approx)}", (x + w + 20, y + 40), font, 0.7, (0, 0, 255), 2)
            cv2.putText(imgContours, f"Area: {int(area)}", (x + w + 20, y + 60), font, 0.7, (0, 0, 255), 2)

while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        break

    imgContours = img.copy()
    imgBlur = cv2.GaussianBlur(img, (7, 7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    t1 = cv2.getTrackbarPos("Threshold1", "Parameters")
    t2 = cv2.getTrackbarPos("Threshold2", "Parameters")
    imgCanny = cv2.Canny(imgGray, t1, t2)

    kernel = np.ones((5, 5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    getContours(imgDil, imgContours=imgContours)

    imgStack = np.hstack([img, imgContours])  # Stack the display for comparison
    cv2.imshow('Output', imgStack)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


