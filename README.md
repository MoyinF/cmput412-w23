# CMPUT 412 - WINTER 2023
This repository contains the submissions for all CMPUT 412 Winter 2023 lab assignments.
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
