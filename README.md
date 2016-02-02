McGill Robotics Frontend
========================

Build Status
------------

[master]: http://dev.mcgillrobotics.com:8080/buildStatus/icon?job=frontend_master
[master url]: http://dev.mcgillrobotics.com:8080/job/frontend_master

[dev]: http://dev.mcgillrobotics.com:8080/buildStatus/icon?job=frontend_dev
[dev url]: http://dev.mcgillrobotics.com:8080/job/frontend_dev

[all]: http://dev.mcgillrobotics.com:8080/buildStatus/icon?job=frontend_all
[all url]: http://dev.mcgillrobotics.com:8080/job/frontend_all

| Branch   | Status                  |
|:--------:|:-----------------------:|
| `master` | [![master]][master url] |
| `dev`    | [![dev]][dev url]       |
| `all`    | [![all]][all url]       |

Setup
-----

1. Create a catkin workspace. Follow 
[this guide](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

2. In the src directory of your catkin workspace, clone the frontend 
repository.

3. Run the following to install the `rosbridge_server` dependency: 
	
   ```bash
   sudo apt-get install ros-jade-rosbridge-server
   ```

4. In the frontend directory, run `npm install` to install all required npm 
dependencies.

5. Run `bower install` to install all required bower dependencies.

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

Gulp
----

To display a list of tasks simply run:

```js
gulp help
```

To lint, build, and start the server, run:

```js
gulp          // the taskname default is optional here
```

Or choose from any other task, such as:

```js
gulp start    // start the build and serve process (default)
gulp build    // build all dependencies
gulp serve    // start the webserver
gulp lint     // check dependencies for style
gulp clean    // remove all compiled dependencies
gulp styles   // compile/process styles
gulp scripts  // compile/process scripts
gulp images   // minify images (in production)
gulp elements // copy (and vulcanize in production)
gulp test     // run tests
```
