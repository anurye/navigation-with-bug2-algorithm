# Bug2

## Introduction

The goal of this task is to introduce navigation realized by the Bug2 algorithm. The basic idea is to move along the line connecting the target and initial point and in the case of being near to the obstacle follow its contour and thus circumnavigate it. 
The program should work as follows:

1. Find the line `l` connecting initial and target position.
2. Rotate towards the goal.
3. Move towards the goal along the `l` line until reaching an obstacle or the goal. If the goal is reached, then stop.
4. When the robot reaches the obstacle, save the distance to the goal `d`.
5. Use the “moving along the walls” controller to avoid the obstacle (Task 4).
6. Depart immediately when the robot is on the `l` line again and the distance to the goal is lower than `d`.
7. Go to step 2.

Implement a control callback function `solution5` that realizes the Bug2 navigation task. Use the input variables `position` and `orientation` to determine the line `l` and to calculate the distance `d`. 
The goal point should be provided as an additional input argument to the function `solution5`, so running the simulation would be: 

`run_simulation(@solution5, false, [goal_x, goal_y])`.
![image](https://github.com/ahmed-yesuf/navigation-with-bug2-algorithm/assets/122631227/1081150c-d979-4ced-ab78-ce3a048496ad)

# Setup
## Software components
Please read and follow the instruction carefully, to avoid errors and wasting your precious time!  

The programming/simulation environment used in EMOR tutorials reiles on four software components:  

- Matlab  
- Peter Corke’s RVC toolbox 
- The CoppeliaSim simulator (it was named V-REP in old times)  
- The Matlab bindings for CoppeliaSim
  
It is required to use a fairly recent version of Matlab. Please notice that Matlab versions older than 2011 may cause problems.  

## CoppeliaSim, Robotics Toolbox
The recommended procedure for setting up your computer is listed below:  
  
- Download the EMOR tutorials repository form github.com: the [ZIP snapshot](https://github.com/RCPRG-ros-pkg/emor_trs/archive/master.zip). Unzip the zip file 
  in the `~ws_emor` directory (in Ubuntu) or `C:/emor/ws_emor` directory (in Windows).  
- Download `CoppeliaSim EDU`. When you’re done downloading CoppeliaSim, unpack it in the `ws_emor` workspace directory.  
- Install the CoppeliaSim bindings for Matlab:  
1. You must copy three files from the directory of the CoppeliaSim app (downloaded at the previous step) to the directory named `youbot` within your local copy of the GitHub repository. The three files you need to copy are named  
- `remApi.m` – the file is located in `{CoppeliaSim_path}/programming/remoteApiBindings/matlab/matlab`; NOTE: in the newest version of CoppeliaSim, the files may be located in `{CoppeliaSim_path}/programming//legacyRemoteApi/remoteApiBindings/matlab/matlab`  
- `remoteApiProto.m` – the file is located in `{CoppeliaSim_path}/programming/remoteApiBindings/matlab/matlab`  
- `remoteApi.so` (if you use Linux) or `remoteApi.dll` (if you use Windows) or `remoteApi.dylib` (if you use a Mac). If you have a choice between a 32-bit or 64-bit `remoteApi`, pick the one that corresponds to your Matlab install (32-bit Matlab or 64-bit Matlab). If you have 32-bit Matlab, pick the 32-bit remoteApi, even if your kernel is 64-bit. The file is located in `{CoppeliaSim_path}/programming/remoteApiBindings/lib/lib` You will find these files in the directory containing the CoppeliaSim app. Look in the `programming/remoteApiBindings/lib/lib and programming/remoteApiBindings/matlab/matlab` subdirectories of the CoppeliaSim app directory (although this can change from version to version). You must copy these files to the youbot directory within your copy of the GitHub repo. If you closely followed the instructions above, the youbot directory is at `~/ws_emor/emor_trs/youbot` (Linux/Mac) or `C:/ws_emor/emor_trs/youbot` (Windows).
2. Run Matlab and change the current directory to the youbot directory (in Matlab Command Window), e.g. on Linux/Mac:

- `cd ~/ws_emor/emor_trs/youbot`
    
Then type (in Matlab):  
```
vrep=remApi('remoteApi');   
vrep.delete();
```  
If there is no error, the Matlab bindings to CoppeliaSim are loaded! If there is an error, check the steps above, and read CoppeliaSim Matlab bindings help.  
- In Matlab Command Window, run the `startup_robot.m` file provided via the Git repository. If you cloned the Git repository in the `ws_emor` workspace directory, run in Matlab Command Window:
 
`run('~/ws_emor/emor_trs/matlab/startup_robot.m');`

It will download and subsequently run the Peter Corke’s Robotics, Vision and Control toolkit. This line needs to be run every time Matlab restarts. You can add the script to your Matlab startup file.  


