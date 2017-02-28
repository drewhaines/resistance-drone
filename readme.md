#The Resistance Drone

![alt text](images/the_resistance_logo.png "The Resistance Logo")

####The Resistance Drone is an autonomous drone created to deliver light payloads to specified locations via GPS. A Pixy Cam and Rasberry Pi are used for accuracy to get the drone closer to the target object. Once over the specified target, the drone can drop it's payload by rotating the dropping mechanism using a servo motor.  This project is for our Senior Design Class at SDSU.



##Requirements

- 3DR Solo Drone
- Python >= 2.7.0 
- [Clone The Resistance Drone Repo](https://github.com/drewhaines/resistance-drone)
- [DroneKit](http://python.dronekit.io/guide/quick_start.html)
- [Pixy Cam](http://charmedlabs.com/default/pixy-cmucam5/)
- Rasberry Pi 3
- Servo Motor and Batteries
- Dropping Mechanism



##Simulator

SDSU Field GPS Coordinates:
- 32.773632, -117.073654 - Home
- 32.773902, -117.072860 - First point
- 32.773523, -117.072120 - Second point
- 32.773180, -117.072764 - Third point

To run the simulator, you'll need:
- [MavProxy](https://erlerobotics.gitbooks.io/erle-robotics-mav-tools-free/content/en/installing_mavproxy.html)
- [APM Planner 2](http://ardupilot.org/planner2/docs/installing-apm-planner-2.html)

Open 3 terminals and enter one of these in each. Enter them in order!

1. dronekit-sitl solo-2.0.20 --home=32.773632,-117.073654,584,353
2. mavproxy.py --master tcp:127.0.0.1:5760 --out 127.0.0.1:14550 --out 127.0.0.1:14551
3. python /path-to-cloned-directory/the-resistance-drone.py

- The first command is to start SITL Simulator.
- The second command is to start MavProxy.
- The last command is to run the script that controls the drone.



## Team

- Ameenah Alnaser
- Artin Daryabari
- Drew Haines
- Edwin Corpus Jr. 
- Efrain Magallon
- Mark Lawrence Galvan
- Phoebe Nguyen
- Ray Johnson
- Ryan Estacio


## Contact

For more info contact Drew Haines at drew@dmdevco.com.



















