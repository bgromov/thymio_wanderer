## Running AR Track Alvar on the Thymio Robot##

### Thymio Robot ###

  1. To turn Thymio **ON**:
  
    - Shift the white micro-switch on Odroid PCB up (towards ON sign).
      
    - Wait ~1-2 minutes before it starts. The LED in front of Back Arrow sensor button (the one pointing to the back of Thymio)
  should start blinking in red.
  
  2. To turn Thymio **OFF**:
  
    - To gracefully turn the Thymio off, hold Back Arrow button for ~8 seconds (until the robot makes a sound and switch the LEDs off)
    
    - Shift the white micro-switch on Odroid PCB down
    
    - After previous step Thymio base may turn LEDs again. To switch it off hold the central round sensor button for ~5 seconds (at this moment the robot may try to move, so don't let it escape!)

### Network Configuration for PC ###

By default, Thymio's Wi-Fi is configured to act as an Access Point (AP) and is bridged to the Ethernet interface.

1. Connect your PC to WiFi network named `ThymioN`, where `N` is the number on the orange sticker on Thymio's body.

2. In order to make ROS-nodes on virtual machine (VM) to communicate properly with nodes running on Thymio robot,
  make sure the only network interface of the PC connected is Wi-Fi. Also, make sure no custom DNS servers are specified
  in network settings, the only DNS entry allowed is `192.168.168.1`. On OS X, the later located in Advanced settings of Wi-Fi.

3. If you have Internet over the wired connection, disconnect it from your PC and plug the cable to the robot, Thymio will
  share it back over Wi-Fi.

### Network Configuration for VM ###

1. Switch network adapter to the bridged mode. E.g. on VMware Fusion, in the menu `Virtual Machine` -> `Network Adapter` choose
`Bridged`.

### ROS ###

1. Install [AR Track Alvar](http://wiki.ros.org/ar_track_alvar) package:

    ```
    $ sudo apt-get install ros-indigo-ar-track-alvar
    ```

2. Pull the latest version of [thymio_wanderer](https://github.com/bgromov/thymio_wanderer) (this package):

    ```
    $ roscd thymio_wanderer
    $ git pull
    ```

  (In case of merge conflicts, see the notes for Assignment 2)

3. Specify ROS master URI:

    ```
    $ export ROS_MASTER_URI=http://192.168.168.1:11311
    ```

4. (optional) To make previous changes persistent over reboots, add the above `export` command to the `~/.bashrc` file.

5. Check that ROS tools are able to communicate with master on Thymio:

    ```
    $ rostopic list
    ```

    This command should produce a long list of topics.

### Marker Cubes ###

The array of poses of the tracked marker cubes (dices) is published on `/ar_pose_marker`, where each entity has an `id` field.
This ID corresponds to the number printed on the first face of each cube.

Pose stored in the array is exactly a pose of the marker on the first face of a cube. Therefore, to obtain a pose of the center
of a cube it should be shifted along Z-axis by -0.025 m (2.5 cm).

Pose of each detected marker can also be obtained from TF.

### RViz ###

The model of the robot, TF-frames, image from the camera and detected markers can be visualized in RViz. To start it with
an appropriate config:

``` shell
$ roslaunch thymioid rviz.launch
```
    
Camera display is disabled by default, but can be enabled by checking the corresponding box on the Displays pane on the left.
