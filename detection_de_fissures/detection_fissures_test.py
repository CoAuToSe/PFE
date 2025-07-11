import cv2
import numpy as np
import matplotlib.pyplot as plt

# === CONFIG ===
video_path = "video_test.mp4"
min_area = 100

# Couleurs à détecter (HSV)
colors_hsv = {
    'bleu': ([1, 157, 232], [119, 181, 254]),
}

# Initialisation
cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    print("Erreur : impossible d'ouvrir la vidéo.")
    exit()

unique_fissures = []

def contour_deja_connu(cnt, seuil=50):
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return False
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    for (x, y) in unique_fissures:
        if abs(x - cx) < seuil and abs(y - cy) < seuil:
            return True
    unique_fissures.append((cx, cy))
    return False

# === Affichage Matplotlib ===
plt.ion()
fig, ax = plt.subplots()

img_display = None

while True:
    ret, frame = cap.read()
    if not ret:
        print("✅ Fin de la vidéo.")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color_name, (lower, upper) in colors_hsv.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > min_area and not contour_deja_connu(cnt):
                print("Olala une couleur !")
                cv2.drawContours(frame, [cnt], -1, (255, 0, 0), 2)

    # Convertir pour affichage matplotlib (BGR → RGB)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    if img_display is None:
        img_display = ax.imshow(frame_rgb)
        plt.title("Détection de fissures (rose)")
        plt.axis("off")
    else:
        img_display.set_data(frame_rgb)
    
    plt.pause(1 / 30)  # Pause pour simuler 30 fps

cap.release()
plt.ioff()
plt.show()

print(f"🎯 Fissures uniques détectées : {len(unique_fissures)}")
