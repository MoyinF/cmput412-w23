# Exercise 5: ML for Robotics
This lab provides an introduction to machine learning in robotics. The object of the lab is to drive a duckiebot autonomously around the environment, whilst detecting hand-written digits posted above apriltags around the town.

Code for deliverable 2 of our exercise is stored in [this google colab file](https://drive.google.com/file/d/1xCzCckQqqgaHs2HVH4e0ew5MFLqFDnb9/view?usp=sharing).

Code for satisfying deliverable 3 of the exercise is contained within this repository.
- Under the `local` folder, you will find a ROS docker image to be run locally on a duckiebot, which contains a node that orchestrates autonomous movement through the environment.
- Under the `remote` folder you will find a ROS docker image to be run on a desktop or laptop computer, containing a service node that manages prediction of hand-written digits.
- Because pytorch libraries are too large to load directly on the duckiebot, the deliverable has been subdivided into seperate projects as described.
