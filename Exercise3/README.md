## Exercise 3 - Computer Vision in Robotics.
This exercise provided an introduction to computer vision and localization in robotics. It builds on top of the deadreckoning techniques developed in exercise 2, using fiducial markers to enable better pose estimation in the environment.

For solutions to part 1.1 of the exercise, please visit packages/augmented_reality_basics and packages/augmented_reality_apriltag

For solutions to part 1.2 of the exercise, please visit packages/apriltag_detector

For solutions to part 2 of the exercise, please visit packages/lane_following

The remainder of the exercise has code that spans both packages/apriltag_detector and packages/deadreckoning

To run this code, make sure you are in the directory containing the `Dockerfile` and run `dts devel build -f -H <hostname>.local` and then `dts devel run -H <hostname>.local`, where `<hostname>` is the name of the duckiebot you will be running this program on.
#### Additional contributor: Austin Tralnberg: atralnbe@ualberta.ca
