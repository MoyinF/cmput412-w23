# CMPUT 412 - WINTER 2023
This repository contains the submissions for all CMPUT 412 Winter 2023 lab assignments.
For more information, visit: [My CMPUT 412 Website](https://sites.google.com/ualberta.ca/famobiwo-cmput-412/)
### Contributors
Moyinoluwa Famobiwo: famobiwo@ualberta.ca

## Installing and Running the Code
If you would simply like to run the code or make changes to this repository, you can copy the link to the repository from the green button labelled "<> Code" at the top right, and run `git clone <repository_url>`.
If you would like your own separate version, you can fork the repository by clicking the button labelled "Fork" at the top right of the page.

## Exercise 1 - Our Intro to ROS and Duckietown exercise. 
This exercise is about learning the basic knowledge and skills you need to operate a Duckiebot.
The files in this folder are for the Docker color detector program.

## Exercise 2 - ROS and Kinematics.
The goal of this exercise was learning different components of ROS, the Basic application of
Robotic Kinematics and Odometry.
To run this code, make sure you are in the directory containing the `Dockerfile` and run `dts devel build -f -H <hostname>.local` and `dts devel run -H <hostname>.local`, where `<hostname>` is the name of the duckiebot you will be running this program on.
#### Additional contributor: Austin Tralnberg: atralnbe@ualberta.ca

## Exercise 3 - Computer Vision in Robotics.
This exercise provided an introduction to computer vision and localization in robotics. It builds on top of the deadreckoning techniques developed in exercise 2, using fiducial markers to enable better pose estimation in the environment.

For solutions to part 1.1 of the exercise, please visit packages/augmented_reality_basics and packages/augmented_reality_apriltag

For solutions to part 1.2 of the exercise, please visit packages/apriltag_detector

For solutions to part 2 of the exercise, please visit packages/lane_following

The remainder of the exercise has code that spans both packages/apriltag_detector and packages/deadreckoning

To run this code, make sure you are in the directory containing the `Dockerfile` and run `dts devel build -f -H <hostname>.local` and then `dts devel run -H <hostname>.local`, where `<hostname>` is the name of the duckiebot you will be running this program on.
#### Additional contributor: Austin Tralnberg: atralnbe@ualberta.ca

## Exercise 4 - Don't Crash! Tailing Behaviour
This exercise involved implementation of autonomous safe tailing behavior on our Duckiebot.

We used the following template created by Xiaou Zepeng: https://github.com/XZPshaw/CMPUT412503_exercise4 and code from the Lane Follow node by Justin F.

To run this code, make sure you are in the directory containing the `Dockerfile` and run `dts devel build -f -H <hostname>.local` and then `dts devel run -H <hostname>.local`, where `<hostname>` is the name of the duckiebot you will be running this program on.

Implementation details:
Our duckiebot tailing node communicates with the duckiebot detection node and distance node which help determine whether there is a duckiebot in the frame and the distance to the duckiebot if present. If there is a duckiebot in the frame a PID controller controls the velocity to ensure our bot follows the leading duckiebot without crashing. 
If there is no duckiebot within the frame, a separate PID controller is used to execute lane following behaviour with proper road etiquette.
#### Additional contributor: Austin Tralnberg: atralnbe@ualberta.ca

## Exercise 5 - Machine Learning for Robotics
In this exercise, we implement a program for our duckiebot to detect and accurately decipher digits. Detection is done using april tags and deciphering is done using a trained neural network. We implement a duckiebot node which communicates with a ROS node on an external computer. This is necessary as the duckiebot's memory is not sufficient for running the neural network.

To run this code, navigate to the `local` directory containing the `Dockerfile` and run `dts devel build -f -H <hostname>.local` and then `dts devel run -H <hostname>.local`, where `<hostname>` is the name of the duckiebot you will be running this program on e.g. `csc229XX.local`.
To run the laptop package, navigate to the `remote` directory and run `dts devel build -f && dts devel run -R <hostname>.local`
#### Additional contributor: Austin Tralnberg: atralnbe@ualberta.ca

## Final Project - Autonomous Driving with Duckietown

In this final project, students were challenged to create a program that enables a duckiebot to traverse duckietown, whilst collecting points for completing various behaviours. There are three stages in the town, each with a unique set of objectives that the robot had to meet. The final project represents a synthesis of all the exercises that preceded it. Students were encouraged to draw upon their learnings in computer vision, machine learning, localization, and odometry as needed, selecting the most appropriate techniques to find an optimal solution to the tasks.

This repository contains implementation solutions for the final project. For information about the project, please read the [report](https://sites.google.com/ualberta.ca/famobiwo-cmput-412/labs/final-project).

To set the stall parameter, as well as other parameters such as wheel velocities, omega values, stopping distances, etc., ssh into the duckiebot and create a file in the `/data/` folder titled `final_config.yaml`. The exact file we used is in this repository. You can change the numbers in `/data/final_config.yaml`, for example with the following steps:

```
ssh duckie@csc229xx.local # where csc229xx is the duckiebot's hostname
vim /data/final_config.yaml # creates or opens the stall file, where you write the number of the stall and save
```

To run the program, ensure that the variable `$BOT` stores your robot's host name (ie. `csc229xx`), and run the following commands:

```
dts devel build -f -H $BOT.local
dts devel run -H $BOT.local
```

The program shuts down automatically after completing stage 3.
To shutdown the program before that, enter `CTRL + C` in your terminal.

### Credit:

This code is built from the Duckiebot detections starter code by Zepeng Xiao (https://github.com/XZPshaw/CMPUT412503_exercise4).
Autonomous lane following code was also borrowed from Justin Francis.
Build on top of by Nadeen Mohamed, Moyinoluwa Famobiwo, and Austin Tralnberg.
#### Additional contributors: Austin Tralnberg: atralnbe@ualberta.ca, Nadeen Mohamed: nadeen@ualberta.ca

