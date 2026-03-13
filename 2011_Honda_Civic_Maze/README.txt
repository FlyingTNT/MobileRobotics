## PHASE 2 - 2011 HONDA CIVIC SUBMISSION

Included in this file are two seperate python modules:
- Mapping.py
- wall_follower.py

## Mapping.py
Our Mapping.py controller is the primary solution to Phase 2. The code included in 
the file handles both the mapping and navigating to the end. Within the first segment, mapping, the robot 
will have none of the LEDs active. The robot will briefly trace the maze, and once it finds its way back to the start,
re-align itself to the center of the starting tile. During this process, the code will output a maze.txt file in the 
same directory as the controller. This file simply visualizes the map that the robot traces, and does not 
serve any importance to its functionalities.
This file expects the robot to begin generally in the center of the start tile.

To indicate when the robot will begin its pathfinder to the maze, led8 will be activated. This is the LED 
associated with the central body of the e-puck. Visually, the lower body will turn green. At this point, our
code will direct the robot to our determined shortest path. The LED will remain on until the robot reaches the end.

## wall_follower.py
For the purposes of the competition, the wall_follower.py is our fallback solution.
The code simply follows the right wall until it reaches the end. There robot begins searching for the
end as soon as the code begins to run, so there isn't any physical indicators when it begins/ends. 
