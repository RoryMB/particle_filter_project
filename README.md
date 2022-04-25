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
The goal of this project was to implement particle filter localization. This meant given a map of an environment, our Python script is meant to estimate the position and orientation of a robot.
### High-level Description
Our script uses Monte Carlo Localization. Firstly, a uniform random distribution of particles are initialized over a map of a maze. As the robot moves through the maze and publishes new LiDAR scan measurements, each particle's weight is computed by comparing the robot's front, left, and right LiDAR measurements to the particle's hypothetical sensor measurments. Higher weights indicate a closer similarity with the robot's LiDAR measurements. Then the weights of all the particles are normalized to a summation of one and the particles are resampled proportionally to their weights.
### Code Description
#### Initialization of particle cloud
#### Movement model
#### Measurement model
#### Resampling
#### Incorporation of noise
#### Updating estimated robot pose
#### Optimization of parameters
### Challenges
### Future Work
### Takeaways
