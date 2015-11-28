/**
 * @file Defines any interface-related common object definitions or methods.
 */

/*global ROSLIB, BaseWidget */

'use strict';

/**
 * Defines a globally accessible object with commonly used javascript
 * parameters, functions, and other resources.
 * @author David Lougheed
 * @constructor
 * @param properties {Object} - Specifies configuration settings for interface
 * @global 
 */
var MRFrontendInterface = function (properties) {
  // The base set of Polymer properties that all components share.

  this.baseComponentPolymerProperties = {
    topic: String,
    messageType: String,
  };

  this.host = properties.host;
  this.port = properties.port;
  this.containerId = properties.containerId;

  // Connect to ROS
  this.ros = new ROSLIB.Ros({
    url: 'ws://' + this.host + ':' + this.port.toString()
  });

  this.ros.on('connection', function () {
    console.log('Connected to websocket server.');
  });
  this.ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
  });

  this.widgets = [];
};

/**
 * Combine the set of base properties that all Polymer components share and
 * any specific component-defined properties into a single object and return
 * it.
 * @function
 * @param ext {Object} - Contains Polymer component-specific properties
 */
MRFrontendInterface.prototype.buildComponentPolymerProps = function (ext) {
  var allPolymerProperties = {},
    prop;

  // Add all base Polymer properties to the return object.
  for (prop in this.baseComponentPolymerProperties) {
    // Make sure no inherited properties are overwritten
    if (this.baseComponentPolymerProperties.hasOwnProperty(prop)) {
      allPolymerProperties[prop] = this.baseComponentPolymerProperties[prop];
    }
  }

  // Add "extended" properties, i.e. ones specific to the component itself.
  for (prop in ext) {
    if (ext.hasOwnProperty(prop)) {
      allPolymerProperties[prop] = ext[prop];
    }
  }

  return allPolymerProperties;
};

MRFrontendInterface.prototype.refreshDOM = function () {
  var container = document.getElementById(this.containerId),
    w;
  
  // Clear all current DOM elements
  container.innerHTML = '';

  for(var w in this.widgets) {
    container.appendChild(this.widgets[w]);
  }
};

MRFrontendInterface.prototype.addWidget = function (position) {
  this.widgets.push(new BaseWidget(position));
  this.refreshDOM();
};

