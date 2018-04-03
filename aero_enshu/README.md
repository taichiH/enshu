### about

This is a tutorial package for using the
[Seednoid platform](https://github.com/seed-solutions/aero-ros-pkg)
with the aero_object_detector and negomo planner.
Make sure to compile *aero_devel_lib* and *negomo_enshu*.
The GUI requires python cv2. Your machine should be Ubuntu.

Your platform must have a vision sensor.
The tutorial only works with objects and settings shown below.
(picture to be added)



#### How to run

Run on robot:
```
roscore
```
```
roslaunch aero_startup aero_bringup.launch
```

Run on GPU machine:
```
roslaunch aero_object_detector fcn_object_detector.launch
```
```
roslaunch aero_object_detector hand_detector.launch
```

and don't forget to stream your vision sensors.



### pick_caffe
```
roslaunch aero_enshu pick_caffe.launch
```
Try placing and removing the coffee drink from the container. When the robot detects an error, try recovering the task from the GUI.



### hand_caffe
```
roslaunch aero_enshu hand_caffe.launch
```
Try putting your hand in front of the robot's camera and see how the robot changes its actions.
