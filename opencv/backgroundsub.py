import cv2
import numpy as np

cap = cv2.VideoCapture(0)
fgbg = cv2.createBackgroundSubtractorMOG2()

while True:
    ret, frame = cap.read()
    fgmask = fgbg.apply(frame)
    fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    # define range of yellow color in LAB
    lower_yellow = np.array([144, 108, 137])
    upper_yellow = np.array([255, 127, 182])

    # threshold the LAB image to get only yellow colors
    mask = cv2.inRange(lab, lower_yellow, upper_yellow)

    # apply mask to the foreground mask
    fgmask = cv2.bitwise_and(fgmask, mask)

    # find contours in the thresholded image
    cnts = cv2.findContours(fgmask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    center = None

    # only proceed if at least one contour was found
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
