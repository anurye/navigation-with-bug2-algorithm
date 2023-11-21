# Task 5
## Introduction
The goal of this task is to introduce navigation realized by Bug2 algorithm. The basic idea is to move along the line connecting the target and initial point and in the case of being near to the obstacle follow its contour and thus circumnavigate it. The program should work as follows:  

1. Find the line l connecting initial and target position.  
2. Rotate towards the goal.  
3. Move towards the goal along the l line until reaching an obstacle or the goal. If the goal is reached, then stop.  
4. When the robot reaches the obstacle, save the distance to the goal d.  
5. Use “moving along the walls” controller to avoid the obstacle (Task 4).  
6. Depart immediately when the robot is on the l line again and the distance to the goal is lower than d.  
7. Go to the step 2.


Implement a control callback function solution5 that realizes the Bug2 navigation task. Use the input variables position and orientation to determine the line l and to calculate the distance d. The goal point should be provided as an additional input argument to the function solution5, so running the simualtion would be: run_simulation(@solution5, false, [goal_x, goal_y])  

