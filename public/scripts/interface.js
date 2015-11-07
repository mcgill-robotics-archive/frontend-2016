/**
 * @file Defines any interface-related common object definitions or methods.
 */

/*global ROSLIB */

'use strict';

/**
 * Defines a globally accessible object with commonly used javascript
 * parameters, functions, and other resources.
 * @author David Lougheed
 * @constructor
 * @param host {string} - Specifies host address for ROSBridge server
 * @param port {number} - Specifies ROSBridge server port
 * @global 
 */
var MRFrontendInterface = function (host, port) {
  // Connect to ROS
  this.ros = new ROSLIB.Ros({
    url: 'ws://' + host + ':' + port.toString()
  });

  this.ros.on('connection', function () {
    console.log('Connected to websocket server.');
  });
  this.ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
  });
};

/**
 * Return a collection of basic Polymer properties that will be included in
 * all Polymer components defined.
 * @function
 */
MRFrontendInterface.prototype.getPolymerBaseProperties = function () {
  return {
    topic: String,
    messageType: String
  };
};

