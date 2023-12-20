# README

## Quickstart
To use this code, please ensure that you have CoppeliaSim downloaded as well as the scenes from the modern robotics class library. 

To use the code please follow these steps.
1. Run the python file of choice in the "code" directory. Best yields a smooth run with the default placement, overshoot yields a run with the Ki gain tuned too high, and newTask shows a different trajectory for the cube. Each file will return a .csv file.
2. Open CoppeliaSim and begin "Scene6_youBot_cube". 
3. Copy the path to your csv file after pressing the play button. The robot should move to grab the cube.
    - Note that if using the newTask file, the initial and goal position of the cube must be changed to [1.0, 1.0, 0.0] and [1.0, -1.0, -1.5707].
4. Select the "Play File" button and watch the robot move!

## Notes
1. To test the individual portions of the project, the "Milestones" directory has broken the script into individual functions with comments.
2. Results from the experiments can be seen in the "results" directory. 
3. To change aspects of the code, the trajectory can be changed either in the transformations or the TrajectoryGenerator function.
