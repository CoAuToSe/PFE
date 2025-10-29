import numpy as np
import cv2

cap = cv2.VideoCapture(10)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Display the resulting frame
    cv2.imshow('frame', frame)

    keyPress = cv2.waitKey(10)
    if keyPress == ord('q'):
        break
    # end if

# end while

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()