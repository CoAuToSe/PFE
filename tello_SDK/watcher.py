import socket

# Obtenir l'adresse IP locale
nom_hote = socket.gethostname()
ip_locale = socket.gethostbyname(nom_hote)

# Configuration du serveur
# hôte = '0.0.0.0'  # écoute sur toutes les interfaces réseau
hôte = ip_locale  # écoute sur toutes les interfaces réseau
port = 11111      # port d'écoute (à adapter selon ton besoin)

# Création du socket
serveur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serveur.bind((hôte, port))
serveur.listen(5)  # nombre de connexions en attente autorisées

print(f"Le serveur écoute sur le port {port}...")

while True:
    client, adresse = serveur.accept()
    print(f"Connexion de {adresse}")

    données = client.recv(1024).decode()
    print(f"Données reçues : {données}")

    client.close()
