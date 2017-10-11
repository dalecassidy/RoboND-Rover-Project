## Project: Search and Sample Return

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
I created a function for rock delineation called color_thresh_rocks.
The rock function uses a color range for each channel for detection. The red channel
is between 127 and 205, the g channel is between 109 
and 180 and the b channel is between 0 and 94.

```python
def color_thresh_rocks(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met   
    between_thresh = (img[:,:,0] >= 127) & (img[:,:,0] <= 205) \
                & (img[:,:,1] >= 109) & (img[:,:,1] <= 180) \
                & (img[:,:,2] >= 0) & (img[:,:,2] <= 94)
    color_select[between_thresh] = 1
    # Return the binary image  
    return color_select
```

To get obstacles I just took the inverse of the navigable threshold values and multiplied them by a mask:

```python
threshed_obstacles = np.absolute(np.float32(threshed)-1) * mask
```

where mask is defined in the perspect transform function 

```python
mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
```

Mask is a set of pixels set to all ones in the camera's field of view. When we multiply by the mask, we are saying that
only pixels in the camera's field of view have the possibility of being labeled an obstacle in that frame.


#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

First I set the source and destination coordinates to feed into the perspective transform function. 

```python
bottom_offset = 6
source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      ])
warped, mask = perspect_transform(Rover.img, source, destination)
```

This function converts the map to a top down map of what the rover's camera is seeing. Then I applied functions
on that top down map to determine what terrain was navigable, what part has obstacles and what part if any
contains rocks.

```python
threshed = color_thresh(warped)    
threshed_obstacles = np.absolute(np.float32(threshed)-1) * mask
threshed_rocks = color_thresh_rocks(warped)
      
Rover.vision_image[:,:,0] = threshed_obstacles * 255
Rover.vision_image[:,:,1] = threshed_rocks * 255
Rover.vision_image[:,:,2] = threshed * 255
```
Then those pixels have to be converted to an axis from which the rover sees the rocks
(rover_coords). The nav_dists and nav_angles are important in the decision step for navigation.

```python
xpix, ypix = rover_coords(threshed)
Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix, ypix)
```

Those rover_coords are then transformed again into a world map coordinate system (pix_to_world). 


```python
xpix_obstacles, ypix_obstacles = rover_coords(threshed_obstacles)
xpix_rocks, ypix_rocks = rover_coords(threshed_rocks)
scale = 10
navigable_x_world, navigable_y_world = pix_to_world(xpix, ypix, Rover.pos[0], 
                            Rover.pos[1], Rover.yaw, 
                            Rover.worldmap.shape[0], scale)
    
rock_x_world, rock_y_world = pix_to_world(xpix_rocks, ypix_rocks, Rover.pos[0], 
                            Rover.pos[1], Rover.yaw, 
                            Rover.worldmap.shape[0], scale)
    
obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obstacles, ypix_obstacles, Rover.pos[0], 
                            Rover.pos[1], Rover.yaw, 
                            Rover.worldmap.shape[0], scale)
```
Those world
coordinates are then put into data.worldmap for each corresponding terrain type.

```python    
Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 10
Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
```

The += 10 for the navigable terrain is just a step to help improve fidelity by saying
that if both a navigable pixel and obstacle pixel are seen in the same location, assume
that it's a navigable pixel more.

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

I pulled most of the code from the notebook's process_image() to perception_step().This perception step is 
responsible for creating the world map which the decision step will use to take action. Refer above 
to the reasons why I did this for each corresponding line of code. 
The only difference in the perception_step function from process_image() was the addition of:

```python
Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix, ypix)
```

decision_step uses Rover.nav_dists and Rover.nav angles to navigate the rover. The rover 
steers in the direction of the mean navigation angles. For the submitted project, I did not need
to modify the decision_step. However, I had tried some updates to that function as described below.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

The resolution I used was the default 1024 by 768. FPS is 25.

The results that I had meet the challenge specifications mapping at least 40% 
of the environment with 60% fidelity against the ground truth. 
I find at least one rock sample and generally several of them.

The approach I took has the rover steering in the direction of the mean angle. Of the different
changes I had made, this still worked the best. I had tried using the median of the nav angles in an attempt
to get a better fidelity but this did not work at all and also got the rover stuck more than using the mean 
angle did. In another attempt to increase the fidelity of the rover's mapping, I limited the valid nav
distances to only a certain distance such as 60. This increased the fidelity significantly. This
makes sense because pixels further out will be affected more by hard breaks and sharp turns than ones
closer by. The problem I had with this is that the rover kept getting stuck. Had I not run of out time,
I would have explored this path more. I would have needed a more reliable way to get unstuck. Once 
I identified I was in a stuck state, I would have tried reversing the vehicle with a negative throttle.
Then instead of using the mean angle of the navigable pixels (which would get me stuck again), I would have 
randomly chosen slightly to the
left or right of the mean by maybe 15% and then throttle forward. 

Another thing I would think about is when you are approaching a rock, you have an obstacle in the middle and
longer navigable paths on the left and/or right. I'd have the decision step recognize this and then choose
the left or the right part of the navigable section.

And finally I would also try making the vehicle navigate along the sides to pick up all the rocks. I had attempted
to do this with just adding a constant angle to the mean angle. This was working ok except I would get
stuck at times amongst the obstacles. I would combine this approach with obstacle detection as I mentioned
above.



