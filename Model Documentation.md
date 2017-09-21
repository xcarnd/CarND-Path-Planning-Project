# CarND-Path-Planning-Project
## Model Documentation

Here I'll describe how the trajectory is generated.

The overall strategy for path generation is much the same as the one mentioned in the walkthrough, that is, 
picking 5 control points the ego vehicle shall follow. If previously planned path information is
available, then reused that information for a smoother and continuous trajectory generation, otherwise
use the states of the vehicle to generate a path from scratch. 

The 5 control points are then used to fit a spline, which is a mathematical description of the final generated
path. After spline is fit, it is then being sampled at constant speed and constant time interval (0.02 seconds) since the 
simulator is updating every 0.02 seconds and will move to the next point in the planned path. The process can be illustrated as below:

![Sampling][figures/sampling.png]

Although this is not an accurate reflection of how the ego vehicle will move because the actually path it follows is not 
straight, the approximation performs well in this project.

The remaining details of the model can then be divided into two aspects:

### 1. How to pick up the 5 control points

The strategy for picking up the 5 control points is much the same as in the walkthrough. The first 2 points are from the last planned
trajectory, and 3 points for 30/60/90 meters further into the road from the vehicle's current location.

The important part is picking which lane shall the vehicle be at in the future (30/60/90 meters). To decide which lane is appropriate,
I've used a simple finite state machine to do the job.

In the FSM there're 3 states representing the vehicle's behavior: keeping lane (KL), lane change left (LCL) and lane change right (LCR). Given the current state
of the ego vehicle, the FSM can decide which following behavior shall be performed.

 For each possible future behaviors, the cost for it
is calculated by evaluating a set of cost functions. Here're the cost function I've used:

| Cost Function             |             Notes                  |
|:-------------------------:|:----------------------------------:|
| buffer_cost               | Awards behaviors which will end up gaining more buffer distance in the target lane.       |
| max_allowed_velocity_cost | Penalizes behaviors which will end up with low speed in the target lane. |
| lane_cost_cost | Penalizes lane changes. |
| safe_distance_cost | Penalizes behaviors which will end up making the ego vehicle getting too close (or even collides) with others |

The target lane is determined by behavior states. By choosing the state with minimum cost, I can determine where the ego vehicle shall be in the future.

### 2. How to determine the "constant speed" which is used in sampling.

The path planner has remembered a reference speed and this will be used when doing sampling. The reference speed cannot be fixed and shall be increased/decrease gradually, 
otherwise the ego vehicle will easily collide with other vehicles in the roads, and the acceleration/jerk will be very high.

Since I designed three different behavior states for the vehicle, I've assigned different reference speed updating strategy to different states:

1. For LCL/LCR, the reference speed won't change. (Whether it is safe to perform lane change with the reference speed is determined by the cost functions described previously)

2. For KL, things get a little bit involved. When staying lane, I want to avoid collisions at the best efforts. That may requires the ego vehicle decelerate. To do this, I first found the nearest leading vehicle who was driving at the same lane as the ego vehicle from the sensor fusion data. Next, I predicted the future location in 5 seconds of both the ego vehicle and the nearest vehicle. If in the future the two vehicle will get closer than a threshold distance, I will decrease the reference speed until the speed of ego car reach 90% the speed of the target vehicle.

If no potential collision risk, I will increase the reference speed of the ego vehicle until the reference speed reaches the maximum allowed speed.