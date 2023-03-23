# Exercise 4
## Don't Crash! Tailing Behaviour
This exercise involved implementation of autonomous safe tailing behavior on our Duckiebot.

We used the following template created by Xiaou Zepeng: https://github.com/XZPshaw/CMPUT412503_exercise4 and code from the Lane Follow node by Justin F.

To run this code, make sure you are in the directory containing the `Dockerfile` and run `dts devel build -f -H <hostname>.local` and then `dts devel run -H <hostname>.local`, where `<hostname>` is the name of the duckiebot you will be running this program on.

Implementation details:
Our duckiebot tailing node communicates with the duckiebot detection node and distance node which help determine whether there is a duckiebot in the frame and the distance to the duckiebot if present. If there is a duckiebot in the frame a PID controller controls the velocity to ensure our bot follows the leading duckiebot without crashing. 
If there is no duckiebot within the frame, a separate PID controller is used to execute lane following behaviour with proper road etiquette.
#### Additional contributor: Austin Tralnberg: atralnbe@ualberta.ca
