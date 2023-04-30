import cv2
import numpy as np

cap = cv2.VideoCapture(0)
fgbg = cv2.createBackgroundSubtractorMOG2()

while True:
    ret, frame = cap.read()
    fgmask = fgbg.apply(frame)
    fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lower_yellow = np.array([144, 108, 137])
    upper_yellow = np.array([255, 127, 182])
    mask = cv2.inRange(lab, lower_yellow, upper_yellow)

    # Apply mask to the foreground mask
    fgmask = cv2.bitwise_and(fgmask, mask)

    # Remove noise by gaussian blurry
    fgmask = cv2.GaussianBlur(fgmask, (5, 5), 0)

    # Erode the image to remove small details and noise
    kernel = np.ones((5, 5), np.uint8)
    fgmask = cv2.erode(fgmask, kernel, iterations=1)

    # Dilate the image to enhance the ball and remove gaps in between area ofpoints
    fgmask = cv2.dilate(fgmask, kernel, iterations=1)

    # Find contours in the thresholded image
    cnts = cv2.findContours(fgmask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    center = None

    # Only proceed if at least one contour was found
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) > 0:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if radius > 10:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (120, 255, 126), 2)
                    cv2.circle(frame, center, 5, (0, 255, 0), -1)

    cv2.imshow('frame', frame)
    cv2.imshow('fgmask', fgmask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
