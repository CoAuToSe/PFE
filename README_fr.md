
# PFE — Makefile d’installation & de lancement

Ce dépôt contient un **Makefile** permettant d’installer les dépendances, cloner les sous-projets (ROS2, Tello, FaMe, Husky), configurer l’environnement, construire les paquets, et lancer différentes démos/simulations.

> Testé principalement sous **Ubuntu 20.04 (Focal)** et **Ubuntu 24.04 (Noble)**.  
> Les chemins par défaut supposent un dossier `~/PFE`. Le Makefile tente de détecter la version d’Ubuntu et d’installer les paquets correspondants.

---

## Sommaire

1. [Vue d’ensemble](#vue-densemble)
2. [Prérequis](#prérequis)
3. [Installation rapide (Quickstart)](#installation-rapide-quickstart)
4. [Gestion multi-versions Ubuntu](#gestion-multi-versions-ubuntu)
5. [Cibles principales](#cibles-principales)
6. [Structure & variables importantes](#structure--variables-importantes)
7. [Tello (exploitation)](#tello-exploitation)
8. [Husky A300](#husky-a300)
9. [FaMe (modeler, engine, simulation)](#fame-modeler-engine-simulation)
10. [Intégration GitHub (sauvegarde/restauration)](#intégration-github-sauvegarder--restaurer)
11. [Maintenance & nettoyage](#maintenance--nettoyage)
12. [Astuces & dépannage](#astuces--dépannage)
13. [Liens](#liens)

---

## Vue d’ensemble

- **Objectif :** automatiser l’installation (ROS2, Gazebo, Node/NVM, VS Code, etc.), la préparation des workspaces, la compilation via `colcon`, et le lancement des scénarios **Tello**, **Husky**, et **FaMe** (agricole/simu).
- **Philosophie :**
  - Cibles `install_*` pour installer des logiciels.
  - Cibles `clone_*` / `setup_*` pour récupérer et construire les projets ROS.
  - Cibles `launch_*` pour exécuter des scénarios ROS2/Gazebo.
  - Cibles `copy_*_to/from_github` pour sauvegarder/restaurer des fichiers et dossiers.

---

## Prérequis

- Système : **Ubuntu 20.04** ou **Ubuntu 24.04** (autres versions non supportées par défaut).
- Accès `sudo`.
- Connexion Internet.
- Espace disque suffisant (ROS2 + Gazebo + workspaces).

---

## Installation rapide (Quickstart)

### Installation par défaut

```bash
# Dans le répertoire où se trouve le Makefile
make
````

Cette cible exécute `git_init_PFE` (clonage du dépôt PFE dans `~/PFE`, init des submodules) puis lance `apt_install` selon ta version d’Ubuntu.

### Lancer un exemple

Après l’installation et la construction des paquets :

```bash
make launch_example
```

Cela orchestrera une simulation FaMe + engine avec un délai paramétrable (`DELAY`, par défaut 20s).

---

## Gestion multi-versions Ubuntu

Le Makefile détecte automatiquement la version d’Ubuntu :

* **Ubuntu 24.04** :

  * Dépendances via `make apt_install_24.04`
  * ROS 2 **Jazzy**, Gazebo **Harmonic**
  * Cible utilitaire : `setup_2404` (upgrade, GitHub Desktop, deps et logiciels)

* **Ubuntu 20.04** (ancien) :

  * Dépendances via `make apt_install_20.04`
  * ROS 2 **Foxy**, Gazebo Classic
  * **(Attention)** certaines parties sont anciennes et peuvent nécessiter des ajustements.

---

## Cibles principales

> Liste complète : `make list`

### Installation de base

* `make apt_install` : appelle automatiquement `apt_install_24.04` ou `apt_install_20.04`.
* `make install_software` : installe Terminator, Discord (snap), Fame Modeler (déprécié), etc.
* `make install_node` : installe **NVM** puis **Node.js LTS (Gallium)** et configure la version par défaut.
* `make install_python_3_10` : installe Python 3.10, pip, et quelques libs.
* **24.04** :

  * `make install_ros2_jazzy` (ou `install_ros2_jazzy_bis`)
  * `make install_gazebo_2404`
  * `make install_vscode_2404`
* **20.04** :

  * `make install_ros2_foxy`
  * `make install_gazebo_2004`
  * `make install_vscode_2004` et `make correct_vscode_2004`

### Clonage & construction des projets ROS

* `make clone_ros2_shared`, `make clone_tello_msgs`, `make clone_FaMe_bitbucket`, `make clone_husky_2004`
* `make setup_ros2_shared`, `make setup_tello_msgs`, `make setup_husky`, `make setup_tello`,
  `make setup_FaMe`, `make setup_FaMe_engine`, `make setup_FaMe_agricultural`, `make setup_FaMe_simulation`

  * Ces cibles **sourcent** les environnements requis (`install/setup.bash`, `ROS2_SETUP`), utilisent `nvm use` si requis, puis `colcon build` (souvent `--symlink-install`; certains paquets utilisent `build` ou `npm` + `colcon` selon la config du paquet).
* Nettoyage ciblé : `make clear_<nom_du_pkg>` (supprime `build/ install/ log/` + éventuellement `node_modules` selon le cas).

### Lancements (ROS2)

* **FaMe** :

  * `make launch_FaMe_engine_example`
  * `make launch_FaMe_simulation_multi`
  * `make launch_comportement_agri` (enchaîne simulation puis engine après `DELAY`)
  * `make FaMe_CATS`, `make FaMe_agricultural_multi`, etc.
* **Tello** :

  * `make launch_tello_controller` (ROS2 Tello)
* **Husky** :

  * `make FaMe_husky`, `make FaMe_husky_tello`

### Configuration de l’environnement shell

* `make setup_bashrc` : ajoute un bloc **« CATS Custom commands »** dans `~/.bashrc` avec :

  * `source /opt/ros/$ROS_DISTRO/setup.bash`
  * alias pratiques : `ros-build`, `ros-build-sym`, `ros-sc`, etc.
  * fonction `this-sc <path>` pour sourcer le `install/setup.bash` du dossier ciblé.

---

## Structure & variables importantes

* **Racine projet** : `PFE := $(HOME)/PFE`
* **Workspaces** :

  * `ROS2_SHARED := $(PFE)/ros2_shared`
  * `TELLO_MSGS := $(PFE)/tello_msgs`
  * `FAME := $(PFE)/my_FaMe`
  * `FAME_ENGINE := $(FAME)/fame_engine`
  * `FAME_AGRI := $(FAME)/fame_agricultural`
  * `FAME_SIMU := $(FAME)/fame_simulation`
  * `HUSKY_WS := $(HOME)/husky_ws`
* **Gazebo** : `GZ_MODEL_DIR := ~/.gazebo/models`
* **ROS** : `ROS2_SETUP := /opt/ros/$ROS_DISTRO/setup.bash` (défini dynamiquement selon ta distro, via le shell au moment de l’exécution)
* **Node/NVM** :

  * `NVM_SCRIPT := $HOME/.nvm/nvm.sh`
  * `NODE_VERSION := 16` (ABI 93, Gallium)
  * `NPM_VERSION := 16`
* **Délai orchestration** :

  * `DELAY ?= 20` (secondes) utilisé pour enchaîner certaines cibles `launch_*`.

---

## Tello (exploitation)

### Setup

```bash
make setup_ros2_shared
make setup_tello_msgs
make setup_tello
```

> Selon la version d’Ubuntu, `make correct_git_clone` installe les bons paquets ROS (`ros-$ROS_DISTRO-…`) et place des `COLCON_IGNORE` sur certains paquets Tello non utilisés.

### Lancement

#### Nœuds ROS Tello

```bash
make launch_tello_controller
```

#### Contrôle Tello via manette (SDK)

```bash
python3 ../PFE/tello_SDK/control_manette.py
# ou
python3 ../PFE/tello_SDK/control_manette_multiple.py
```

#### Gazebo

Tu peux lancer des scénarios Gazebo via les cibles FaMe (voir ci-dessous) ou tes propres lancements ROS2/Gazebo (selon les paquets construits).

---

## Husky A300

### Workspace Husky

* Clonage/build : `make clone_husky_2004` puis `make setup_husky`
* Utilitaire : `make setup_husky_launch` (copie un launch custom dans le ws Husky)

### Réinstallation complète

Si tu veux repartir de zéro :

```bash
make clear_husky
make setup_husky
```

> Certains paquets Husky dépendent des paquets ROS `xacro`, `ros2-control`, etc., déjà couverts par les cibles d’install (voir `apt_install_*`).

---

## FaMe (modeler, engine, simulation)

### FaMe modeler

* (Déprécié) `make install_FaMe_modeler` : clone + `npm install` + démarrage.
* Mise à niveau Linux pour le modeler : `make upgrade_linux_for_FaMe-modeler` (met à jour Node, `inotify`, liens `lib`, etc.)

### FaMe engine

* Setup : `make setup_FaMe_engine`
* Lancement (exemples) :

  ```bash
  make launch_FaMe_CATS
  make launch_FaMe_engine_example
  ```

### FaMe agricole

* Setup : `make setup_FaMe_agricultural`
* Lancement multi :

  ```bash
  make launch_FaMe_agricultural_multi
  make launch_comportement_agri
  ```

### FaMe simulation

* Setup : `make setup_FaMe_simulation`
* Lancement multi :

  ```bash
  make FaMe_simulation_multi
  make launch_example
  ```

> Remarque : certaines cibles sourcent `/usr/share/gazebo/setup.bash` et/ou exportent `NODE_OPTIONS="--unhandled-rejections=strict"`.

---

## Intégration GitHub (sauvegarder / restaurer)

Le Makefile fournit des cibles **génériques** pour copier des dossiers/fichiers entre ta machine et l’arborescence `$(PFE)` (utile pour versionner via Git) :

* **Sauvegarder vers PFE** (dans `$(PFE)/…`) :

  * `make copy_to_github`

    * inclut : `copy_simu_gazebo_to_github`, `copy_makefile_to_github`, `copy_bashrc_to_github`,
      `copy_code_setup_to_github`, `copy_gazebo_models_to_github`, `copy_FaMe_to_github`, etc.

* **Restaurer depuis PFE** (ATTENTION, confirme plusieurs fois) :

  * `make copy_from_github`

    * inclut : `copy_simu_gazebo_from_github`, `copy_code_setup_from_github`,
      `copy_gazebo_models_from_github`, `copy_FaMe_from_github`, `copy_makefile_from_github`, etc.

> Les copies utilisent `rsync -a --delete` pour les dossiers (synchronisation complète).
> Les cibles `clean_<nom>` permettent de **supprimer** la destination (protégées par des confirmations).

---

## Maintenance & nettoyage

* **Nettoyage ROS d’un paquet** (supprime `build/ install/ log/`) :

  ```bash
  make clear_<nom_du_pkg>
  # ex :
  make clear_FaMe_engine
  ```
* **Tout tuer côté Gazebo** :

  ```bash
  make kill_all
  ```
* **Rafraîchir l’environnement bash** :

  ```bash
  make setup_bashrc
  source ~/.bashrc
  sc-bash
  ```

---

## Astuces & dépannage

* **Lister toutes les cibles disponibles :**

  ```bash
  make list
  ```

* **Forcer la bonne version ROS dans le shell courant :**

  ```bash
  export ROS_DISTRO=jazzy   # ou foxy selon ta distrib
  source /opt/ros/$ROS_DISTRO/setup.bash
  ```

* **NVM/Node introuvable pendant un build/launch :**

  * Lance `make install_nvm` puis `make install_node`.
  * Ouvre un **nouveau terminal** (ou `source ~/.bashrc`).
  * Vérifie : `command -v nvm && node -v && npm -v`.

* **Erreurs de permissions/ownership lors des copies GitHub :**

  * Les cibles utilisent `sudo rsync` / `sudo install`. Assure-toi d’avoir le mot de passe sudo et des chemins corrects.

* **Conflits de versions Gazebo/ROS** :

  * 20.04 => ROS2 Foxy + Gazebo Classic.
  * 24.04 => ROS2 Jazzy + gz-harmonic.
    Assure-toi d’utiliser les bonnes cibles `apt_install_*` et `install_ros2_*`.

* **Changer le délai d’orchestration (ex. `launch_example`) :**

  ```bash
  make launch_example DELAY=10
  ```

---

## Liens

* **FaMe** : [https://bitbucket.org/proslabteam/fame/src/master/](https://bitbucket.org/proslabteam/fame/src/master/)


