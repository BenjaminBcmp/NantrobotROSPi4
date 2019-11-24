# localizationROS
A ROS package to localize a robot with an ARUCO tag.


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
Add this package to your catkin workspace :

`git clone ... `

Start it with roslaunch :

`roslaunch ...`
