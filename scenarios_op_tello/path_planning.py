#!/usr/bin/python

import pygame
import json
import math

"""
Ce programme permet de capturer un itinéraire sur une image de carte en cliquant sur les points 
désirés, puis enregistre les waypoints sous forme de distances et d'angles dans un fichier JSON.

MAP_SIZE_COEFF permet de convertir la distance en pixels sur l'image en une distance réelle en cm.
56 pixels = 445 cm donc MAP_SIZE_COEFF = 445/56.
"""
MAP_SIZE_COEFF = 7.94642857143  # Coefficient de conversion pixel -> cm

# Initialisation de pygame et de l'affichage
pygame.init()
screen = pygame.display.set_mode([720, 720])  # Création d'une fenêtre de 720x720 pixels
screen.fill((255, 255, 255))  # Remplissage de l'écran en blanc
running = True  # Variable de boucle principale


class Background(pygame.sprite.Sprite):
    """
    Classe permettant de charger une image de fond pour la carte.
    """
    def __init__(self, image, location, scale):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load(image)  # Chargement de l'image
        self.image = pygame.transform.rotozoom(self.image, 0, scale)  # Mise à l'échelle
        self.rect = self.image.get_rect()  # Obtention du rectangle encadrant l'image
        self.rect.left, self.rect.top = location  # Positionnement de l'image


def get_dist_btw_pos(pos0, pos1):
    """
    Calcule la distance entre deux positions de la souris.
    Retourne la distance en cm et en pixels.
    """
    x = abs(pos0[0] - pos1[0])
    y = abs(pos0[1] - pos1[1])
    dist_px = math.hypot(x, y)  # Calcul de la distance en pixels avec le théorème de Pythagore
    dist_cm = dist_px * MAP_SIZE_COEFF  # Conversion en cm
    return int(dist_cm), int(dist_px)


def get_angle_btw_line(pos0, pos1, posref):
    """
    Calcule l'angle entre deux segments de droite formés par trois points.
    L'angle est calculé en utilisant le produit scalaire.
    """
    ax = posref[0] - pos0[0]
    ay = posref[1] - pos0[1]
    bx = posref[0] - pos1[0]
    by = posref[1] - pos1[1]
    
    _dot = (ax * bx) + (ay * by)  # Produit scalaire des deux vecteurs
    _magA = math.sqrt(ax**2 + ay**2)  # Norme du premier vecteur
    _magB = math.sqrt(bx**2 + by**2)  # Norme du second vecteur
    _rad = math.acos(_dot / (_magA * _magB))  # Calcul de l'angle en radians
    angle = (_rad * 180) / math.pi  # Conversion en degrés
    return int(angle)
# Chargement de l'image de fond (carte)
bground = Background('pfe_2025/scenarios_op_tello/map_batiment_s_ensta.png', [0, 0], 1.0)
screen.blit(bground.image, bground.rect)

# Liste des waypoints cliqués
path_wp = []
index = 0  # Index pour suivre le nombre de points cliqués

# Boucle principale pour capturer les clics de souris
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False  # Fermeture de la fenêtre
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()  # Récupération de la position du clic
            path_wp.append(pos)  # Ajout du waypoint à la liste
            if index > 0:
                pygame.draw.line(screen, (255, 0, 0), path_wp[index-1], pos, 2)  # Tracé d'une ligne rouge
            index += 1
    pygame.display.update()  # Mise à jour de l'affichage

# Ajout d'un point référence fictif pour le premier waypoint
path_wp.insert(0, (path_wp[0][0], path_wp[0][1] - 10))

# Listes des distances et angles
path_dist_cm = []
path_dist_px = []
path_angle = []

# Calcul des distances et des angles
for index in range(len(path_wp)):
    if index > 1:
        dist_cm, dist_px = get_dist_btw_pos(path_wp[index-1], path_wp[index])
        path_dist_cm.append(dist_cm)
        path_dist_px.append(dist_px)
    
    if index > 0 and index < (len(path_wp) - 1):
        angle = get_angle_btw_line(path_wp[index-1], path_wp[index+1], path_wp[index])
        path_angle.append(angle)

# Affichage des résultats dans la console
print('path_wp: {}'.format(path_wp))
print('dist_cm: {}'.format(path_dist_cm))
print('dist_px: {}'.format(path_dist_px))
print('dist_angle: {}'.format(path_angle))

# Sauvegarde des waypoints sous forme de JSON
waypoints = []
for index in range(len(path_dist_cm)):
    waypoints.append({
        "dist_cm": path_dist_cm[index],
        "dist_px": path_dist_px[index],
        "angle_deg": path_angle[index]
    })

# Enregistrement dans un fichier JSON
with open('pfe_2025/scenarios_op_tello/waypoint.json', 'w') as f:
    path_wp.pop(0)  # Suppression du point de référence fictif
    json.dump({
        "wp": waypoints,
        "pos": path_wp
    }, f, indent=4)
