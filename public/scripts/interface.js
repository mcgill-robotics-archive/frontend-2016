/*global ROSLIB */

'use strict';

/**
 * Defines a globally accessible object with commonly used javascript
 * parameters, functions, and other resources.
 * @author David Lougheed
 * @global 
 */
var MRFrontendInterface = function () {
  this.ros = new ROSLIB.Ros({
    url: 'ws://localhost:8080'
  });

  this.ros.on('connection', function () {
    console.log('Connected to websocket server.');
  });
  this.ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
  });

  this.polymerBaseProperties = {
    topic: String,
    messageType: String
  };
};

