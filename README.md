# particle_filter_project

## Team members
Kendrick Xie, Rory Butler

## Implementation plan

### Initializing the particle cloud (initialize_particle_cloud)

#### Component plan

Randomly generate a large number of (x,y,θ) tuples across the possible map positions. Remove any points where the robot would intersect a wall.

#### Test plan

Use RViz to visualize the generated particle cloud.

### Updating the position of the particles based on the movements of the robot (update_particles_with_motion_model)

#### Component plan

Add an (∆x,∆y) tuple to each particle based on the particle's θ and the velocity of the robot, or just update the θ if it rotates. This will require tuning the weight of the velocity of the robot on the values.

#### Test plan

Use RViz to visualize movement of particles that should match the robot's actual position, and make sure updates move the particles by a reasonable amount.

### Computing the importance weights of each particle after receiving the robot's laser scan data (update_particle_weights_with_measurement_model)

#### Component plan

We can start with the equation from the particle filter class exercise, and change it up if the weights are too unbalanced.

#### Test plan

Run the algorithm a few times with the robot in different locations to see if the points that closely match the robot also have high weight.

### Normalizing the particles' importance weights (normalize_particles) and resampling the particles (resample_particles)

#### Component plan

The weight normalization equation used in class should be effective for this step, but we might find a better one online. For resampling, Python has existing weighted sampling functions that we can take advantage of.

#### Test plan

Run the algorithm and ensure that after normalization, weights sum to 1, and highly weighted points are resampled most often.

### Updating the estimated pose of the robot (update_estimated_robot_pose)

#### Component plan

The easiest method is to take the most highly weighted particle, but we might also try finding highly grouped collections of particles if we have the time.

#### Test plan

See if the most highly weighted particle is generally successful at matching the robot. If not, try the grouped method.

### Incorporating noise into the particle filter localization

#### Component plan

When calculating the update to the particle tuples, we can add a certain amount of random noise as well. This may require tuning to make sure the noise is not too large.

#### Test plan

Make sure that particles sampled at the same location end up at different locations, and that particles do not diverge too much.

### Estimated Timeline

Finish before Apr 19
- Initialization of particle cloud
- Movement model

Finish before Apr 21
- Importance weights
- Normalization and resampling

Finish before Apr 26
- Updating estimated robot pose
- Incorporation of noise
- Final polish

## Writeup

https://user-images.githubusercontent.com/34782324/165139351-cbc851f3-d6aa-4a9c-8c49-6d783cd61e4f.mp4

### Objectives Description
The goal of this project was to implement particle filter localization. This meant given a map of an environment, our Python script is meant to estimate the position and orientation of a robot. This estimate would be found by filling the environment with virtual particles and iteratively filtering out the particles that don't match the robot's observations.

### High-level Description
Our script uses Monte Carlo Localization. Firstly, a uniform random distribution of particles are initialized over a map of a maze. As the robot moves through the maze and publishes new LiDAR scan measurements, each particle's weight is computed by comparing the robot's front, left, and right LiDAR measurements to the particle's hypothetical sensor measurments. Higher weights indicate a closer similarity with the robot's LiDAR measurements. Then, the weights of all the particles are normalized to a summation of one and the particles are resampled proportionally to their weights. The position of the robot is updated using the average position and orientation of all the particles.

### Main Steps

#### Initialization of Particle Cloud
##### Code Location
The initialization of the particle cloud was done in `initialize_particle_cloud` on line 89.
##### Function/Code Description
This function creates `self.num_particles` number of particles with uniformly random x, y, and theta values such that the particles can point in all directions and cover only the maze area of the map. Each particle is given an initial weight of `1.0`, and then the weights of all of the particles are normalized to sum to one. Lastly, the particles are published.

#### Movement Model
##### Code Location
The implementation of the movement model takes place largely in the function `update_particles_with_motion_model` on line 326. The function is called on line 225.
##### Function/Code Description
This function takes in the distance the robot has traveled since the last `pose` update, the direction relative to the robot's latest yaw, and the change in yaw since the last `pose` update. Then, for all the particles, the x, y, and yaw are adjusted as if they had the same change in distance and orientation as the robot since the last `pose` update.

#### Measurement Model
##### Code Location
The implementation of the measurement model takes place largely in the `update_particle_weights_with_measurement_model` and has helper function `interpolate` and `particle_distance`. These functions are implemented on lines 263-324. `update_particle_weights_with_measurement_model` is called on line 226.
##### Function/Code Description
###### `update_particle_weights_with_measurement_model`
This function lets us weight how closely each particle matches the current LiDAR observations. This function computes the weights for each particle by setting `particle.w = 1 / (abs(front - particle_front) + abs(left - particle_left) + abs(right - particle_right))`, where `front`, `left`, and `right` are the LiDAR scan ranges the robot found in the respective directions and `particle_front`, `particle_left`, and `particle_right` are the hypothetical range values the particle would have in the respsective directions.
###### `interpolate`
This function just does basic 1D interpolation, and is used to convert between world/gazebo coordinates to maze data coordinates. This helps the raycast function `particle_distance` to stick to operating in world coordinates and distances while still being able to read from the maze data array.
###### `particle_distance`
This function computes a raycast from a particle to the nearest wall or out of bounds point along a particular direction. This function recieves a 2d array of the occupancy grid, an x and y position, and the run and rise of the unit vector with an angle of a particle's yaw from the positive horizontal axis. The function computes the distance from the given x and y position to the closest occupied position in the direction of the run and rise. This function was used to find the hypothetical range values of the particles.

#### Resampling
##### Code Location
The particle resampling takes place in the function `resample_particles` on line 143. The function is called on line 228.
##### Function/Code Description
This function helps filter out the particles that least match the current observations of the robot. The function creates a brand new cloud of particles by choosing a weighted sample from the current cloud, where high-confidence particles have higher weight. The clouds are the same size, and duplicate sampling is allowed, therefore particles with low confidence are likely to never be chosen and get filtered out. The problem of having many identical particles in the new cloud is solved by randomly displacing the new particles, as discussed in the next section.

#### Incorporation of Noise
##### Code Location
The noise is incorporated in the function `resample_particles` on line 143.
##### Function/Code Description
As mentioned in the previous section, sampling with replacement means that there will likely be many identical particles in the new cloud. These duplicates serve no useful purpose, so to remove them and also make the algorithm robust to measurement errors, all particles in the new cloud are randomly shifted and rotated by a small gaussian distribution. Therefore, the new cloud will have no duplicate particles, and if there are errors in the measurements from the robot, new particles have a chance to be spawned close to the location the robot truly moved to.

#### Updating Estimated Robot Pose
##### Code Location
The estimation of the robot's current pose takes place in the function `update_estimated_robot_pose` on line 234. The function is called on line 229.
##### Function/Code Description
This function converts the whole particle cloud into a single prediction of the robot's current pose. To do this, the average particle position is found, as well as the average heading. The heading is found by averaging the unit vector "positions" of all particle headings, then converting that average "position" back into a direction.

#### Optimization of Parameters
##### Degrees of Laser Scanner Considered
###### Code Location
The optimization of degrees of laser scanner considered was done in `update_particle_weights_with_measurement_model` on line 263.
###### Function/Code Description
In this function we experimented by considering more angles but did not notice a difference in how fast our particles converged. For this reason, we used the relatively less computationally intensive solution of only consider the angles 0 degrees in front and 90 degrees to the left and right of the robot.

##### Parameter Tuning
###### Code Location
These optimizations were organized to parameters in lines 50, and 56 through 67, in the `__init__` function.
###### Function/Code Description
The first parameters that we tuned were the range of initial positions for particles. Ensuring this range encompassed the whole maze helped when the robot started in corners, and no particles matched the robot's observations.

The most important values we optimized turned out to be the movement thresholds before performing an update. To give an example of why this was useful: In the beginning, there are few particles per unit area (see the next paragraph for why there were few particles). If the robot moved a large distance before updating, it is possible that the few best matching particles were off by enough to move through a wall during the displacement. With smaller movements, the best matching particles would have a chance to be highly weighted and sampled before moving through the wall. This takes advantage of the original uniform distribution, since it is likely that there will be a few well-matching particles during the first timestep or two. After a few timesteps, the cloud is much less distributed, and if the true pose is no longer covered, it can be unrecoverable.

Finally, the random displacement of new particles was reduced to 0.1 laterally and 0.2 radians, and the number of particles was reduced to 1000. Larger values for these variables allowed too many particles to spawn too far away and through walls, which would sometimes generate new temporarily high-confidence particle clumps. The values could have been reduced even more to shrink the rather large cloud we have that follows the robot, but through a few tests, the large cloud allowed for extra error compensation when the robot was moving quickly.

One parameter that we did not optimize was the raycast step size of 0.1 (line 319). Optimizing this would only improve the speed, and since the program was performing correctly and without noticeable lag, we decided optimization was unnecessary.

### Challenges
One of the most difficult parts of the project was understanding how all the pieces of the script fit together. Something that helped with this challenge was focusing on what a function's input and output was and assuming it worked until work needed to be done on it. Additionally, using a small number of particles and using print statements made it easier to understand how the parts of the script fit together.

### Future Work
Given more time, we could have experimented with introducing noice in our motion model, instead of just in the resampling step. For example, we could try to sample it from a normal distribution. Additionally we could try to account for measurement noise, unexpected objects, failure to detect obstacles, and random measurements using the beam model for range finders algorithm specified in Class Meeting 06. As mentioned in the optimizations section, there were a few other optimizations that we decided against doing since they were not impactful to this particular project. Using this particle filter technique in a different map or with a different robot would likely require these optimizations to be finished.

### Takeaways
• Finding and reading documentation on new variable types is very helpful during implementation; however, there are also many videos on the internet that are helpful in explaining key concepts. For example, I was able to find videos that explained how occupancy grids worked.

• It is very important to test code as it is being written. Even if a function is not finished, parts of it can still likely be tested. For example, some time was wasted writing `update_particle_weights_with_measurement_model` under the assumption that the occupancy grid was already a 2-D array, but running the script to try and access the array would have revealed that this was not the case earlier.

• It would have helped to divide up responsibilities *after* reading through the provided code. Some of the code was possible to write independently of the rest, but testing/debugging much of the functionality depended on the previous step's completion. For example, the main logic of resampling was fairly easy, but a couple of bugs only showed up once importance weights and normalization were finished. This lineararity of the project made simultaneous work difficult. I would suggest to future students that they should take turns implementing functions rather than trying to work at the same time.
