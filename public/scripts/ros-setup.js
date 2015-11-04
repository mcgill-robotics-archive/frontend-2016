/*global ROSLIB */

'use strict';

window.ros = new ROSLIB.Ros({
  url: 'ws://localhost:8080'
});

window.ros.on('connection', function () {
  console.log('Connected to websocket server.');
});

window.ros.on('error', function (error) {
  console.log('Error connecting to websocket server: ', error);
});

