McGill Robotics Frontend
========================

###Package Set Up

1. Create a catkin workspace. Follow [this guide](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

2. In the src directory of your catkin workspace, clone the frontend repository.

3. Run the following to install the `rosbridge_server` dependency: 
	
   ```bash
   sudo apt-get install ros-jade-rosbridge-server
   ```

4. In the frontend directory run `$ npm install` to install all required npm dependencies.


###Running the Application

1. Run the following command in the frontend directory: 
	
   ```bash
   roslaunch frontend frontend.launch
   ```


###Trouble Shooting

If you run into any issues you should first check to make sure you have all dependencies installed. 
Another common mistake is to not source you setup.bash.

###ROS Dependencies
* catkin
* rosbridge_server
