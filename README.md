# Bug2

The basic idea is to move along a line connecting the target and initial point and in the case of being near to an obstacle follow its contour and circumnavigate it. 
The program should work as follows:

1. Find the line `l` connecting initial and target position.
2. Rotate towards the goal.
3. Move towards the goal along the `l` line until reaching an obstacle or the goal. If the goal is reached, then stop.
4. When the robot reaches an obstacle, save the distance to the goal `d`.
5. Use the `moving along the walls` controller to avoid the obstacle 
6. Depart immediately when the robot is on the `l` line again and the distance to the goal is lower than `d`.
7. Go to step 2.

# Setup
## Requirements
- Matlab  
- Peter Corke’s RVC toolbox 
- CoppeliaSim simulator
- Matlab bindings for CoppeliaSim
  
>[!IMPORTANT]
> It is required to use a fairly recent version of Matlab, versions older than 2011 may cause problems.

## CoppeliaSim and Robotics Toolbox
- Download `CoppeliaSim EDU` and unpack it in the `~/ws_emor` workspace directory.  
- Download [EMOR tutorials repository](https://github.com/RCPRG-ros-pkg/emor_trs/archive/master.zip) and unzip it in  `~/ws_emor`.

### Install the CoppeliaSim bindings for Matlab:  
Copy the following three files from the directory of the `CoppeliaSim` to the directory named `youbot` within your local copy of `EMOR tutorials repository` downloaded above. 

The three files you need to copy are:
- `remApi.m`: located in `{CoppeliaSim_path}/programming/remoteApiBindings/matlab/matlab`

  > [!NOTE]
  > In the newest version of CoppeliaSim, the files may be located in `{CoppeliaSim_path}/programming//legacyRemoteApi/remoteApiBindings/matlab/matlab`

- `remoteApiProto.m`: located in `{CoppeliaSim_path}/programming/remoteApiBindings/matlab/matlab`  
- `remoteApi.so` (if you use Linux) or `remoteApi.dll` (if you use Windows) or `remoteApi.dylib` (if you use a Mac). If you have a choice between a 32-bit or 64-bit `remoteApi`, pick the one that corresponds to your Matlab install. 

  > [!IMPORTANT]
  > If you have 32-bit Matlab, pick the 32-bit remoteApi, even if your kernel is 64-bit. 

  The file is located in `{CoppeliaSim_path}/programming/remoteApiBindings/lib/lib` You will find these files in the directory containing the CoppeliaSim app. Look in the `programming/remoteApiBindings/lib/lib and programming/remoteApiBindings/matlab/matlab` subdirectories of the CoppeliaSim app directory (although this can change from version to version). You must copy these files to the youbot directory within your copy of `EMOR tutorials repository`.
- Run Matlab and change the current directory to the youbot directory (in Matlab Command Window), e.g. on Linux/Mac:

  ```matlab
  `cd ~/ws_emor/emor_trs/youbot`
  ```
    
- Then type (in Matlab):  
  ```matlab
  vrep=remApi('remoteApi');   
  vrep.delete();
  ```  

## Testing
In MATLAB command window execute:
```matlab
start_robot
```

```matlab
goal_x = 1;
goal_y = 4;
run_simulation(@bug2, false, [goal_x, goal_y])
```

[![Watch the video](https://img.youtube.com/vi/IP27EjzqkGc/0.jpg)](https://youtu.be/IP27EjzqkGc)


