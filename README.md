## Installation of OpenCV
### Installation with `apt`
You just need to run `sudo apt install python3-opencv`

### Installation with `pip`
Run `sudo pip3 install opencv-contrib-python`

You must now install the missing dependencies to be able to use `import cv2`. Use this link to list them https://blog.piwheels.org/how-to-work-out-the-missing-dependencies-for-a-python-package/.

With a fresh install of Raspbian Buster, you will need (with `apt install`):
- libhdf5-103
- libatlas3-base
- libjasper1
- libqtgui4
- libqt4-test


## How to use the ROS package
Rôles des cartes :

- raspberrypi2: ROS_MASTER et liaison série avec Arduino
- raspberrypi4: réseau Wifi, computer vision

### Connexion aux cartes
Se connecter au réseau Wifi NantrobotWifi (password: `NantroboWifi`)

Se connecter en ssh à la raspberrypi2 (login:`ubuntu`, password: `nantrobot`): 
```bash
ssh ubuntu@ubuntu.local
``` 

Dans une autre fenêtre, se connecter à la raspberrypi4 (login: `pi`, password: `nantrobot`, ip: 192.168.4.1): 
```bash
ssh pi@raspberrypi.local
```

### Configuration de ROS
Note: ces commandes sont déjà présentes dans `~/.bashrc`, il est donc inutile de les refaire. Ces commandes sont à répéter dans chaque nouvelle fenêtre de la console.

#### Configuration du ROS_MASTER
Sur chaque carte, configurer l'adresse du ROS_MASTER:
```bash
export ROS_MASTER=http://ubuntu.local:11311
```

Sur la raspberrypi4, configurer ROS_IP :
```bash
export ROS_IP=raspberrypi.local
```

#### Configuration du workspace
Pour lancer un package avec `roslaunch`, ne pas oublier d'exécuter la commande suivante dans le workspace catkin :
```bash
source devel/setup.bash
```


### Lancement de ROS
Sur les raspberrypi, on utilise `screen` pour pouvoir gérer plusieurs fenêtres avec une seule connexion `ssh`. Voir plus bas pour les commandes basiques. 

Sur la raspberrypi4, lancer le package contenant les scripts de computer vision :
```bash
roslaunch test_nantrobot test_com.launch
```

Dans une autre fenêtre, vérifier que le topic `/localization` est actif avec 
```bash
rostopic list
```

Pour voir ce qui est publié sur ce topic, utiliser
```bash
rostopic echo /localization
```

Enfin, il faut initialiser ROS sur l'Arduino, qui est connectée en USB à la raspberrypi2. Sur la raspberrypi2:
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

Et voilà !


### Utilisation de `screen`
`screen` permet de gérer plusieurs fenêtre au sein d'un même terminal, ce qui éviter de multiplier les connexions `ssh`. Au sein d'une session screen, toutes les commandes relatives à `screen` sont précédées de `CTRL + a` 

Pour lancer une session `screen` nommée `ros`:
```bash
screen -S ros
```

Pour ajouter une fenêtre à la session (semblable à un nouvel onglet dans le terminal) : `ctrl + a` puis `c`. 

Pour changer de fenêtre, `ctrl + a` puis taper le numéro de la fenêtre (0 ou 1 ici). 

Il est possible de "détacher" la session, c'est-à-dire de la mettre en tâche de fond pendant qu'on effectue une autre tâche. Celle-ci continue à s'exécuter même si la connexion `ssh` est terminée. Pour détacher une session : `ctrl + a` puis `d`.

Pour remettre une session au premier plan, on peut commencer par lister toutes les sessions en cours :
```bash
screen -ls
````

Pour rappeler une session nommée `ros` :
```bash
screen -r ros
```

Pour fermer la session `screen`, faire `ctrl + a` puis `k` et valider avec `y` dans chaque fenêtre.
