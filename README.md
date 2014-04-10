ardrone_tutorials
=================

This repository contains a ROS-Node allowing the Parrot AR.Drone to navigate itself to a landing destination via tags. It is based on ardrone_tutorials from Mike Hamer

Szenario
==========

The desired outcome is, that the drone lands reproducibly in the same position in a certain spot, so that one could attach some electric contacts to the feet, in order to charge it.
Right now, it just isn't reliable enough, but most of the times the drone manages to land pretty close to the tag.

The setup
=============

The 3 striped tag (Code is set for Orange-Green-Orange) is placed about 1.8 m above the ground (most likely on some kind of wall, preferably with some darker background)
About 1 meter in front of the tag lies the "oriented roundel" (That A4 sized tag glued inside the packaging) on the floor.
The drone is situated about 3 to 5 meters away from the wall. For my purposes I've just let it start from that point, but you could arrive at this situation via GPS navigation or any other method you wish.

What the drone is supposed to be doing
=======================================

Once the "BringMeHome()" method is called, the drone will turn on the spot until it found the 3 striped tag.
Then it will approach this tag, while making adjustments to keep it centered in the field of view.
Upon detecting the tag on the floor, it will try to center this tag in it's field of view.
Once centered, it will land.

How do I do this?
=========================

Setup
------------

Using Ubuntu 12.04, just follow Mike Hamers [Tutorial](http://robohub.org/up-and-flying-with-the-ar-drone-and-ros-getting-started/) to set everything up but instead of cloning his repository, clone mine. (The virtual machine provided on his website was too slow on my setup, but maybe a more powerful machine could handle it). Since there were a couple of updates, you might run into some issues:

ROS got updated to Hydra, therefore the ardrone_autonomy driver got updated and isn't compatible anymore with the tutorial. You can still get the old branch with this command 
```
git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b fuerte
```

Cloning the repository fails? Use `git clone https://[...]` instead of `git clone git@github[...]`

Getting Started
-----------------------

Once you downloaded the repositories and compiled the driver, you are ready to start. Connect to the Wifi of your drone, then open a terminal, and type `roscore` and hit enter.

In another terminal, type the following:
``` shell
roscd ardrone_tutorials/src
python drone_controller.py
```

This window will show you which commands are currently sent to the drone.

In another terminal window, you will launch the GUI via:
```
roslaunch ardrone_tutorials keyboard_controller_with_tags.launch
```

Once the GUI is open, you should see what the drone sees. If you point the camera at a tag, a small green dot will appear with the distance written right next to it.
You can fly around now, as described in Mikes tutorial

When you want to test the landing algorithm, open another terminal.
Navigate to the src folder via:
```
roscd ardrone_tutorials/src
```

Open the python shell by simply typing `python`
Then you will load the script via
```
execfile('landing_controller.py')
````
Now you have an object called `controller` available. You can check out the methods available in the source code, but to see everything in action, type
`controller.BringMeHome()`

Now you should see some stuff happening, both in the window running the *drone_controller.py* and here. You can interrupt it at any time with `Ctrl + C` and start it again simply by retyping the `controller.BringMeHomeCommand()`.
Once the drone sees the bottom tag, a second green dot will appear on the screen. You don't see the video feed, but the dot represents the position in the downwards facing camera

While the controller is navigating, you wont be able to properly steer the drone with the keyboard, since the controller is constantly sending commands overriding your keyboard. You have to stop the script first.

You can exit the python shell via `exit()`

Advanced
---------------

When you start tinkering around with the file, you need to load those changes by running `execfile(...)` again. If you changed some of the constants, those changes will only be loaded if you reload the whole python shell (`exit() and `python`).

If you want to get a better feeling for the performance, you can open a window, where values will be plotted with ROS. For example, if you want to see how the lateral corrections are doing related to the x position of the tag, you could use the following command:
```
rxplot /ardrone/navdata/tags_xc[0] /ardrone/steering_commands/m12
```
Don't be suprised, the plotting will only start once a tag was visible

If you want to plot which commands are sent to the drone you could use
```
rxplot /ardrone/steering_commands/m11:m12:m13:m21
```
