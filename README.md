# Camera Viewpoint Selection

## Dependencies
### ROS Packages
 - cv_bridge

### Packages
- Cmake: 3.12.4 version min
- OpenCV

## Usage
```
git clone https://github.com/uwgraphics/camera_viewpoint_interface.git
cd camera_viewpoint_interface
git submodule init && git submodule update
catkin build
roslaunch viewpoint_interface viewpoint_interface.launch config_file:=periscope.json
```
In the Layouts Control Panel, select Dynamic Camera from the drop down. Use 'C' to make the control panel disappear.