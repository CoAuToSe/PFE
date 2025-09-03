import cv2
import numpy as np
import time
from collections import deque

# --- PID très simple ---
class PID:
    def __init__(self, kp=0.8, ki=0.0, kd=0.12, out_min=-1.0, out_max=1.0):
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
        if self.prev_t is None:
            dt = 0.0
        else:
            dt = max(1e-3, t - self.prev_t)

        self.integral += err * dt
        deriv = (err - self.prev_err) / dt if dt > 0 else 0.0

        u = self.kp*err + self.ki*self.integral + self.kd*deriv
        u = max(self.out_min, min(self.out_max, u))

        self.prev_err = err
        self.prev_t = t
        return u

# --- Fonctions de sortie à substituer par vos drivers ---
def set_steering(u):  # u in [-1, 1]
    # Ex : mappez u vers l'angle servo
    # angle = 90 + u*40
    print("angle = ", u)
    pass

def set_speed(v):     # v in [0, 1]
    # Ex : PWM moteur
    print("speed = ", v)
    pass

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640) 
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

pid = PID(kp=1.0, ki=0.0, kd=0.15)
base_speed = 0.6
min_contour_area = 500  # à ajuster selon la taille de la ligne dans la ROI
roi_ratio = 0.35        # hauteur de la bande en bas de l'image

# Historique pour détection de perte
conf_hist = deque(maxlen=10)

COLOR = False
HOUGH = True

if HOUGH:
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        h, w = frame.shape[:2]
        roi_h = int(h * roi_ratio)
        roi = frame[h - roi_h : h, 0 : w]

        if not COLOR:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)

            # Otsu + inversion (ligne sombre -> pixels blancs)
            _, bin_inv = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

            # Nettoyage morphologique
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(bin_inv, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        if COLOR:
            # Conversion HSV
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # Choix d'une couleur (ex : jaune)
            lower = np.array([0, 127, 0])
            upper = np.array([127, 255, 127])
            mask = cv2.inRange(hsv, lower, upper)

            # Nettoyage
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Détection de bords
        edges = cv2.Canny(mask, 50, 150)

        # Hough transform
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50,
                                minLineLength=30, maxLineGap=20)

        cx = None
        if lines is not None:
            xs, ys, angles = [], [], []
            for l in lines:
                x1,y1,x2,y2 = l[0]
                # Dessin
                cv2.line(roi, (x1,y1), (x2,y2), (0,255,0), 2)
                # Milieu
                xm = (x1+x2)//2
                ym = (y1+y2)//2
                xs.append(xm)
                ys.append(ym)
                # Angle
                angle = np.arctan2(y2-y1, x2-x1)
                angles.append(angle)
            # Moyenne des milieux
            if xs:
                cx = int(np.mean(xs))
                cv2.circle(roi, (cx, roi.shape[0]//2), 5, (0,0,255), -1)
            # Moyenne des angles (optionnel pour l’orientation)
            mean_angle = np.mean(angles)
            cv2.putText(roi, f"angle={np.degrees(mean_angle):.1f}°",
                        (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,255,255),2)

        # Ligne médiane repère
        cv2.line(roi, (w//2,0), (w//2,roi.shape[0]), (255,0,0), 2)

        if cx is not None:
            err = (cx - w/2) / (w/2)
            u = pid(err)
            set_steering(u)
            set_speed(base_speed)
        else:
            set_speed(0.2)

        # Affichage
        view = frame.copy()
        view[h - roi_h : h, 0 : w] = roi
        cv2.imshow("Line Following (ROI en bas)", view)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break
else:

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        h, w = frame.shape[:2]
        roi_h = int(h * roi_ratio)
        roi = frame[h - roi_h : h, 0 : w]
        if not COLOR:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)

            # Otsu + inversion (ligne sombre -> pixels blancs)
            _, bin_inv = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

            # Nettoyage morphologique
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(bin_inv, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        if COLOR:
            # Conversion HSV
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # Choix d'une couleur (ex : jaune)
            lower = np.array([0, 127, 0])
            upper = np.array([127, 255, 127])
            mask = cv2.inRange(hsv, lower, upper)

            # Nettoyage
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Chercher le plus grand contour
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cx = None
        confidence = 0.0

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > min_contour_area:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    # Confiance simple: aire normalisée + compacité
                    confidence = min(1.0, area / (roi_h * w * 0.5))
                    # Dessins (debug)
                    cv2.drawContours(roi, [c], -1, (0, 255, 0), 2)
                    cv2.circle(roi, (cx, roi_h // 2), 5, (0, 0, 255), -1)

        # Ligne médiane pour repère
        cv2.line(roi, (w // 2, 0), (w // 2, roi_h), (255, 0, 0), 2)

        if cx is not None:
            err = (cx - w / 2) / (w / 2)  # err in [-1, 1]
            u = pid(err)
            # Vitesse diminue avec l'erreur et la faible confiance
            v = base_speed * max(0.2, (1.0 - 0.7 * abs(err))) * max(0.2, confidence + 0.2)
            set_steering(u)
            set_speed(v)

            # Overlay debug
            cv2.putText(roi, f"err={err:+.2f} u={u:+.2f} v={v:.2f} conf={confidence:.2f}",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        else:
            # Ligne perdue : ralentir et conserver la dernière commande
            set_speed(0.2)
            cv2.putText(roi, "Ligne perdue - recherche...", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Affichage
        view = frame.copy()
        view[h - roi_h : h, 0 : w] = roi
        cv2.imshow("Line Following (ROI en bas)", view)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
