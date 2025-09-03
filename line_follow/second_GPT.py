import cv2
import numpy as np
import time
from collections import deque

# ---------------- PID simple ----------------
class PID:
    def __init__(self, kp=1.0, ki=0.0, kd=0.15, out_min=-1.0, out_max=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_t = None

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_t = None

    def __call__(self, err):
        t = time.time()
        dt = 0.0 if self.prev_t is None else max(1e-3, t - self.prev_t)
        self.integral += err * dt
        deriv = (err - self.prev_err) / dt if dt > 0 else 0.0
        u = self.kp*err + self.ki*self.integral + self.kd*deriv
        u = max(self.out_min, min(self.out_max, u))
        self.prev_err = err
        self.prev_t = t
        return u

# ---------------- Stubs moteurs ----------------
def set_steering(u):  # u in [-1, 1]
    # Exemple : angle = 90 + u*40
    pass

def set_speed(v):     # v in [0, 1]
    pass

# ---------------- Trackbars HSV ----------------
def nothing(_): pass

def create_hsv_trackbars():
    cv2.namedWindow("HSV", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("HSV", 420, 420)
    # Plage 1
    cv2.createTrackbar("H1 low",  "HSV", 20, 179, nothing)
    cv2.createTrackbar("H1 high", "HSV", 30, 179, nothing)
    cv2.createTrackbar("S1 low",  "HSV", 100, 255, nothing)
    cv2.createTrackbar("S1 high", "HSV", 255, 255, nothing)
    cv2.createTrackbar("V1 low",  "HSV", 100, 255, nothing)
    cv2.createTrackbar("V1 high", "HSV", 255, 255, nothing)
    # Seconde plage (optionnelle, utile pour le rouge)
    cv2.createTrackbar("Use 2nd Range", "HSV", 0, 1, nothing)
    cv2.createTrackbar("H2 low",  "HSV", 170, 179, nothing)
    cv2.createTrackbar("H2 high", "HSV", 180, 180, nothing)
    cv2.createTrackbar("S2 low",  "HSV", 120, 255, nothing)
    cv2.createTrackbar("S2 high", "HSV", 255, 255, nothing)
    cv2.createTrackbar("V2 low",  "HSV", 70,  255, nothing)
    cv2.createTrackbar("V2 high", "HSV", 255, 255, nothing)

    # Morphologie & Canny rapides
    cv2.createTrackbar("Open k",  "HSV", 5, 15, nothing)   # impair
    cv2.createTrackbar("Close k", "HSV", 5, 15, nothing)   # impair
    cv2.createTrackbar("Canny L", "HSV", 50, 255, nothing)
    cv2.createTrackbar("Canny H", "HSV", 150, 255, nothing)
    cv2.createTrackbar("Min Area", "HSV", 500, 5000, nothing)

def read_hsv_trackbars():
    get = lambda name: cv2.getTrackbarPos(name, "HSV")
    h1l, h1h = get("H1 low"),  get("H1 high")
    s1l, s1h = get("S1 low"),  get("S1 high")
    v1l, v1h = get("V1 low"),  get("V1 high")
    use2 = get("Use 2nd Range")
    h2l, h2h = get("H2 low"),  get("H2 high")
    s2l, s2h = get("S2 low"),  get("S2 high")
    v2l, v2h = get("V2 low"),  get("V2 high")
    open_k  = max(1, get("Open k") | 1)   # force impair
    close_k = max(1, get("Close k") | 1)
    canny_l = get("Canny L")
    canny_h = max(canny_l+1, get("Canny H"))
    min_area = max(10, get("Min Area"))
    lower1 = np.array([min(h1l,179), s1l, v1l], dtype=np.uint8)
    upper1 = np.array([min(h1h,179), s1h, v1h], dtype=np.uint8)
    lower2 = np.array([min(h2l,179), s2l, v2l], dtype=np.uint8)
    upper2 = np.array([min(h2h,179), s2h, v2h], dtype=np.uint8)
    return (lower1, upper1, use2, lower2, upper2, open_k, close_k, canny_l, canny_h, min_area)

# ---------------- Programme principal ----------------
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

create_hsv_trackbars()

pid = PID(kp=1.0, ki=0.0, kd=0.15)
base_speed = 0.6
roi_ratio  = 0.35
mode = "contour"    # ou "hough"
conf_hist = deque(maxlen=10)

print("Touches : 'c' = Contours, 'h' = Hough, 'q'/Echap = quitter")

while True:
    ok, frame = cap.read()
    if not ok:
        break

    h, w = frame.shape[:2]
    roi_h = int(h * roi_ratio)
    roi = frame[h - roi_h : h, 0 : w]

    # HSV + masques
    lower1, upper1, use2, lower2, upper2, open_k, close_k, canny_l, canny_h, min_area = read_hsv_trackbars()
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower1, upper1)
    if use2:
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        mask = mask1

    # Nettoyage morphologique
    kernel_o = np.ones((open_k, open_k), np.uint8)
    kernel_c = np.ones((close_k, close_k), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel_o, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_c, iterations=1)

    cx = None
    confidence = 0.0

    if mode == "contour":
        # --------- Méthode CONTOURS ----------
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > min_area:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    confidence = min(1.0, area / (roi_h * w * 0.5))
                    cv2.drawContours(roi, [c], -1, (0, 255, 0), 2)
                    cv2.circle(roi, (cx, roi_h//2), 5, (0,0,255), -1)

    else:
        # --------- Méthode HOUGH ----------
        edges = cv2.Canny(mask, canny_l, canny_h)
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180,
                                threshold=50, minLineLength=30, maxLineGap=20)
        if lines is not None:
            xs, angles = [], []
            total_len = 0.0
            for l in lines:
                x1,y1,x2,y2 = l[0]
                cv2.line(roi, (x1,y1), (x2,y2), (0,255,0), 2)
                # Pondérer par la longueur du segment
                seg_len = np.hypot(x2-x1, y2-y1)
                xm = (x1+x2)/2.0
                xs.append(xm * seg_len)
                angles.append(np.arctan2(y2-y1, x2-x1) * seg_len)
                total_len += seg_len

            if total_len > 0:
                cx = int(np.sum(xs) / total_len)
                mean_angle = np.sum(angles) / total_len
                # Confiance grossière : proportion d'edges couverts
                confidence = min(1.0, np.count_nonzero(edges) / (edges.size * 0.15))
                cv2.circle(roi, (cx, roi_h//2), 5, (0,0,255), -1)
                cv2.putText(roi, f"angle={np.degrees(mean_angle):.1f} deg",
                            (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        # Affiche edges en debug
        cv2.imshow("Edges (Hough)", edges)

    # Ligne médiane repère + overlays
    cv2.line(roi, (w//2, 0), (w//2, roi_h), (255, 0, 0), 2)
    cv2.putText(roi, f"Mode: {mode.upper()}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    if cx is not None:
        err = (cx - w/2) / (w/2)   # [-1, 1]
        u = pid(err)
        v = base_speed * max(0.2, (1.0 - 0.7*abs(err))) * max(0.2, confidence + 0.2)
        set_steering(u)
        set_speed(v)
        cv2.putText(roi, f"err={err:+.2f} u={u:+.2f} v={v:.2f} conf={confidence:.2f}",
                    (10, roi_h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
    else:
        set_speed(0.2)
        cv2.putText(roi, "Ligne perdue - recherche...",
                    (10, roi_h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

    # Affichages
    view = frame.copy()
    view[h - roi_h : h, 0 : w] = roi
    cv2.imshow("Line Following (ROI)", view)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q'):
        break
    elif key == ord('c'):
        mode = "contour"
    elif key == ord('h'):
        mode = "hough"

cap.release()
cv2.destroyAllWindows()
