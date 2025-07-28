Très bon choix ! Utiliser **Webots avec ROS 2 via Docker** est une excellente manière de bénéficier de **Ubuntu 22.04** (et de ROS 2 Humble) tout en travaillant sur une machine hôte en **Ubuntu 20.04**. Voici un guide pas-à-pas pour y arriver.

---

## 🧭 Objectif

Créer un conteneur Docker tournant **Webots + ROS 2 (Humble)** dans un environnement **Ubuntu 22.04**, tout en permettant :

* l'accès à l'interface graphique de Webots sur la machine hôte,
* l'utilisation de ROS 2 avec les interfaces nécessaires (nodes, topics, etc.),
* un volume partagé si besoin pour garder vos fichiers persistants.

---

## ✅ Pré-requis

Sur ta machine **Ubuntu 20.04** :

1. **Docker** installé :

   ```bash
   sudo apt update
   sudo apt install docker.io
   sudo systemctl enable --now docker
   sudo usermod -aG docker $USER
   newgrp docker  # ou relancer la session
   ```

2. **NVIDIA GPU (optionnel)** : Si tu veux l'accélération graphique :

   * Installer **NVIDIA Container Toolkit** : [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

3. **X11 Forwarding** : Pour permettre à Webots (application graphique) de s’afficher :

   ```bash
   xhost +local:docker
   ```

---

## 🐳 Étapes Docker

### 1. Créer un `Dockerfile`

Tu peux partir de cette base :

```Dockerfile
# Dockerfile

FROM ubuntu:22.04

# Setup locale
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install -y \
    curl wget git lsb-release gnupg2 sudo locales \
    x11-apps libgl1-mesa-glx libxrender1 libxi6 \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8  
ENV LC_ALL en_US.UTF-8

# Install ROS 2 Humble
RUN apt update && apt install -y software-properties-common \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list' \
    && apt update \
    && apt install -y ros-humble-desktop python3-colcon-common-extensions

# Source ROS setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install Webots
RUN wget https://cyberbotics.com/ephemeral/webots_2024-a_amd64.deb \
    && apt install -y ./webots_2024-a_amd64.deb \
    && rm webots_2024-a_amd64.deb

# Install Webots ROS2 interface
RUN git clone https://github.com/cyberbotics/webots_ros2.git /root/webots_ros2 \
    && cd /root/webots_ros2 && rosdep update && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build

WORKDIR /root/webots_ros2
CMD ["/bin/bash"]
```

---

### 2. Build l'image Docker

Dans le dossier du `Dockerfile` :

```bash
docker build -t webots_ros2 .
```

---

### 3. Lancer le conteneur

Avec interface graphique X11 :

```bash
docker run -it \
    --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    --privileged \
    webots_ros2
```

> 📝 Si tu veux sauvegarder les projets/fichiers entre les sessions, ajoute un `-v` pour lier un volume :

```bash
-v ~/mon_ws:/root/webots_ros2
```

---

## 🧪 Vérification

1. Une fois dans le conteneur, source ROS 2 :

   ```bash
   source /opt/ros/humble/setup.bash
   cd /root/webots_ros2
   source install/setup.bash
   ```

2. Lance Webots :

   ```bash
   webots
   ```

3. Lance un exemple ROS 2 :

   ```bash
   ros2 launch webots_ros2_universal_robot universal_robot.launch.py
   ```

---

## 🧩 Conseils complémentaires

* Pour améliorer le support graphique :

  * Si tu as un GPU NVIDIA, installe `nvidia-docker2` et ajoute `--gpus all` dans la commande `docker run`.

* Pour le développement plus fluide :

  * Utilise **VSCode avec Docker plugin** pour coder directement dans le conteneur.

---

Souhaites-tu que je te génère un `docker-compose.yml` pour lancer tout ça plus facilement ?
