import cv2

cap = cv2.VideoCapture(cv2.CAP_OPENNI2)
if not cap.isOpened():
    raise RuntimeError("Xtion non détectée via OpenNI2")

cap.set(cv2.CAP_OPENNI_REGISTRATION, 1)

while True:
    if not cap.grab():
        continue

    ok_depth, depth = cap.retrieve(cv2.CAP_OPENNI_DEPTH_MAP)
    ok_rgb, rgb     = cap.retrieve(cv2.CAP_OPENNI_BGR_IMAGE)

    if ok_depth:
        depth_vis = cv2.convertScaleAbs(depth, alpha=0.03)
        cv2.imshow("Depth", depth_vis)
    if ok_rgb:
        cv2.imshow("RGB", rgb)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
