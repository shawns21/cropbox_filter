# Cropbox Filter and VoxelGrid Filter

This ROS2 Node creates a cropbox filter around the sensor, you can set the x, y and z values for how big you want to box to be around the sensor. 

Also, the leaf size can be configured to set the voxel grid filter. 

The input cloud that the node subscribes to is /input_cloud by default and the filtered output cloud is sent to the topic /output_cloud. 

All these parameters can be configured in the params.yml file in the config folder.

# Instructions to install

Do this in your ~/ros2_ws/src directory:

git clone https://github.com/shawns21/cropbox_filter.git

Go back to your ~/ros2_ws and install dependencies with rosdep: 

sudo rosdep init

rosdep update

rosdep install --from-paths src --ignore-src -r -y

Then build the package:

colcon build --packages-select cropbox_filter
