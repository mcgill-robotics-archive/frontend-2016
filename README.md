McGill Robotics Frontend
========================

Build Status
------------

[master]: http://dev.mcgillrobotics.com:8080/buildStatus/icon?job=frontend_master
[master url]: http://dev.mcgillrobotics.com:8080/job/frontend_master

[dev]: http://dev.mcgillrobotics.com:8080/buildStatus/icon?job=frontend_dev
[dev url]: http://dev.mcgillrobotics.com:8080/job/frontend_dev

| Branch   | Status                  |
|:--------:|:-----------------------:|
| `master` | [![master]][master url] |
| `dev`    | [![dev]][dev url]       |

Setup
-----

1. Create a catkin workspace. Follow 
[this guide](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

2. In the src directory of your catkin workspace, clone the frontend 
repository.

3. Add the official NodeJS repositories to install the latest LTS:

  ```bash
  curl -sL https://deb.nodesource.com/setup_4.x | sudo -E bash -
  ```

4. Run the following to install all ROS package dependencies: 
	
   ```bash
   rosdep update
   rosdep install frontend
   ```

5. In the frontend directory, run `npm install` to install all required `npm` 
dependencies.

6. Run `bower install` to install all required `bower` dependencies.

Launching
---------

To run in production, use the following command: 
	
```bash
roslaunch frontend frontend.launch layout:=foo
```

where `foo` is the name of the desired jade layout file.

To run in a development environment, use the following command:

```bash
roslaunch frontend frontend-test.launch
```

This will run with the jade layout file `test.jade`. One can optionally use an
extra argument specifying a layout file by appending `layout:=foo` where `foo`
is the name of the desired jade layout file to test.

Testing
-------

To test and lint the package, simply install all dependencies and run:

```bash
npm test
```

Troubleshooting
---------------

If you run into any issues you should first check to make sure you have all 
dependencies installed. Another common mistake is to not source your 
`setup.bash`.

ROS Dependencies
----------------
* `catkin`
* `rosbridge_server`
* `web_video_server`
