# Jumeau Numérique PyBullet (Kuka IIWA & UDP)

Ce dépôt propose une démonstration de jumeau numérique robotique, simulant un bras Kuka IIWA (7 articulations) sous PyBullet, avec un envoi temps réel des données (positions, collisions, états de phases) via UDP vers un client de visualisation.

## Fonctionnalités principales
- **Simulation 3D** (PyBullet) d’un robot Kuka IIWA effectuant du pick-and-place de cubes.  
- **Communication UDP** pour transmettre en direct la position des articulations, les collisions et les phases courantes.  
- **Client de visualisation** qui reçoit les données et met à jour une scène PyBullet en parallèle, permettant de mesurer la **latence** réseau.  
- **Architecture distribuée** : deux scripts Python séparés, un pour la simulation (“simulateur”), un autre pour la visualisation (“client”).

## Structure du dépôt
- **simulateur_complex.py**  
  Script principal gérant la simulation. Il charge le robot, les cubes, la table, et exécute un mini-automate de phases (go_pick, descend, lift, etc.). Les données sont alors envoyées en UDP.
  
- **visualisation_client.py**  
  Script client recevant par UDP les informations envoyées par le simulateur. Met à jour en temps réel un second environnement PyBullet, calcule la latence, et affiche éventuellement les collisions.

## Prérequis
- **Python 3.7+** (testé sous Python 3.8/3.9)  
- **PyBullet** (`pip install pybullet`)  
- **pybullet_data** pour charger les URDF (inclus automatiquement via PyBullet)  
- Bibliothèques standard Python : `socket`, `json`, `math`, `random`, `time`, etc.

## Installation
1. Cloner le dépôt :
   ```bash
   git clone https://github.com/abdelaaziz0/JUMEAUX_NUM_PYBULLET.git
   ```
2. Installer les dépendances (dans un virtualenv de préférence) :
   ```bash
   pip install pybullet
   ```
3. Vérifier que **pybullet_data** est accessible (c’est généralement installé avec PyBullet).

## Utilisation
1. **Lancer le simulateur** dans un premier terminal :
   ```bash
   python3 simulateur.py
   ```
   Par défaut, il envoie les paquets UDP sur `127.0.0.1:5005`.

2. **Lancer le client** dans un autre terminal :
   ```bash
   python3 visualisation.py
   ```
   Par défaut, il écoute sur la même adresse/port (`127.0.0.1:5005`).

3. Deux fenêtres PyBullet s’ouvrent :
   - Une pour la **simulation** (le “monde réel” ou proxy),
   - Une pour la **visualisation** (le “jumeau numérique”).

4. Sur le terminal du client, vous verrez s’afficher la **latence** de réception et les informations de collisions. 

## Points importants
- Le simulateur téléporte l’objet à l’effecteur quand il “descend” (pince non modélisée).  
- Les collisions ne sont gérées que côté simulateur (mouvement physique). Côté client, c’est surtout un rendu visuel.  
- Vous pouvez ajuster la position et le nombre de cubes dans `simulateur.py` (variable `NUM_CUBES` et placements aléatoires).

