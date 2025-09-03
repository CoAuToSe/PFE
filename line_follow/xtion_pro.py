# import cv2

# # dev = "/dev/v4l/by-id/usb-ASUS_Xtion_Pro_Live-01-video-index0"  # adapte au tien
# # dev = "/dev/v4l/by-id/usb-CNFGH16O2464000D8182_Integrated_Webcam_HD_200901010001-video-index1"  # adapte au tien
# dev = "/dev/video0"  # adapte au tien
# cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)

# if not cap.isOpened():
#     raise RuntimeError(f"Impossible d'ouvrir {dev}")

# while True:
#     ok, frame = cap.read()
#     if not ok:
#         break
#     cv2.imshow("Xtion RGB", frame)
#     if cv2.waitKey(1) == 27:  # ESC
#         break

# cap.release()
# cv2.destroyAllWindows()
import cv2

cap = cv2.VideoCapture(cv2.CAP_OPENNI2)
if not cap.isOpened():
    cap = cv2.VideoCapture(cv2.CAP_OPENNI)  # fallback
print("capture opened")

if not cap.isOpened():
    raise RuntimeError("Xtion non détectée via OpenNI.")
print("Xtion détectée via OpenNI.")
cap.set(cv2.CAP_OPENNI_REGISTRATION, 1)  # aligne depth et couleur

while True:
    if not cap.grab():
        continue

    ok_depth, depth = cap.retrieve(cv2.CAP_OPENNI_DEPTH_MAP)
    ok_rgb,   rgb   = cap.retrieve(cv2.CAP_OPENNI_BGR_IMAGE)

    if ok_depth:
        depth_vis = cv2.convertScaleAbs(depth, alpha=0.03)
        cv2.imshow("Depth", depth_vis)

    if ok_rgb:
        cv2.imshow("RGB", rgb)

    if cv2.waitKey(1) == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
