## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes:
 - Planning a path from the start to the goal, which is 10 meters to the top and to the left 
 - Grid is created based on obstacle data from colliders.csv file
 - Planning is done with A-star on the grid implementation, 
 - Valid actions include only moving to the west, east, north and south, no diagonal moves supported
 - Planning is done after arming before takeoff
 - Target altitude is set to 5 meters, safety distance is set to 5 meters


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

The first line of the file with lot and lan data is read and
set_home_position  method used to set the home postion


#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

Global to local method is used for it: global_to_local(global_position, self.global_home)


#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

We can get grid start position from local position by using grid offest in the following way:
grid_start = (int(current_local_position[0]) - north_offset, int(current_local_position[1]) - east_offset)

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

This can be done in the following way:
- get local position using global to local
- convert to grid position using offset
Here is the code:
goal_global = [-122.398805, 37.793372, 0]  # next corner
goal_local_position = global_to_local(goal_global, self.global_home)
grid_goal = (int(goal_local_position[0]) -north_offset, int(goal_local_position[1]) - east_offset)


#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

1. I've added new 4 types of actions representing diagonal moves with sqrt(2) cost
UPPERLEFT = (-1, -1, DIAGONAL_COST)
UPPERRIGHT = (-1, 1, DIAGONAL_COST)
LOWERLEFT = (1, -1, DIAGONAL_COST)
LOWERRIGHT = (1, 1, DIAGONAL_COST)
2. I've added new check far validity, checking if we the grid cell we're moving to
is inside the grid and doesn't contain obstacle

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

I've used collinearity check and checked across the whole path middle points 
lying on the same line which can be removed



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


