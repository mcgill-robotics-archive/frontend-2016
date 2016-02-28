/**
 * Define behaviour for 2D pose component
 */

/*global Polymer, ROSLIB, frontendInterface */

'use strict';

var PoseComponent = Polymer({
  is: "pose-component",
  properties: frontendInterface.buildComponentPolymerProps({
    angle: Number,
    canvas: Object,
    compassRadius: Number
  }),

  /**
   * Handle adjusting angle of pose line
   * @function
   * @param {Object} polymerContext - Stores a reference to the Polymer element
   */
  rotate: function (polymerContext) {
    var canvasContext = this.canvas.getContext("2d"),
      length = this.compassRadius;
    canvasContext.clearRect(0, 0, 2 * length, 2 * length);
    // draw circle
    this.drawCircle(canvasContext, length);
    // draw pose line
    this.drawPoseLine(canvasContext, length);
    // rotate
    this.rotateToAngle(polymerContext, canvasContext, length);
  },

  attached: function () {
    var polymerContext = this;
    this.initCanvas(polymerContext);

    /*
     * Subscribe to topic defined by HTML attribute / Polymer 
     * property 'topic'.
     */
    this.topicListener = new ROSLIB.Topic({
      ros: frontendInterface.ros,
      name: this.topic,
      messageType: this.messageType
    });

    // Set the message handling function and pass the Polymer element context.
    this.topicListener.subscribe(function (message) {
      polymerContext.handleMessage(polymerContext, message);
    });
  },

  /**
   * Handle component removal.
   * @function
   */
  detached: function () {
    this.topicListener.unsubscribe();
  },
  /**
   * Recieve pose stamped message from the subscribed topic 
   * and set Polymer property 'angle' to the yaw.
   * NOTE: 0 deg. is NORTH
   * @function
   * @param {Object} polymerContext - Stores a reference to the Polymer element
   * @param {Object} message - pose stamped message containing updated angle
   */
  handleMessage: function (polymerContext, message) {
    // message orientation is a quaternion. convert to euler - yaw
    var q = message.pose.orientation,
      numeratorYawEqn = 2.0 * ((q.x * q.y) + (q.w * q.z)),
      denominatorYawEqn = (q.w * q.w) - (q.z * q.z) - (q.y * q.y) + (q.x * q.x),
      yaw = Math.atan2(numeratorYawEqn, denominatorYawEqn);
    //update current angle
    polymerContext.angle = yaw;
    //rotate image
    polymerContext.rotate(polymerContext);
  },



  /** Create the html canvas object to contain the compass
   * @function
   * @param {Object} polymerContext - Stores a reference to the Polymer element
   */
  initCanvas: function (polymerContext) {
    // Set up the canvas 
    var canvas = document.createElement('canvas');
    canvas.id = "canvas";
    canvas.style.height  = '100%';
    canvas.height  = canvas.width;
    this.compassRadius = canvas.width / 2;
    canvas.style.position = "relative";
    Polymer.dom(polymerContext.root).appendChild(canvas);
    this.canvas = canvas;
  },

  /** Draw a circle in the canvas for the background
   * @function
   * @param {Object} canvasContext - Stores a reference to the canvas element
   * @param {Number} radius - The length of the pose line, which is also 
   *                          the radius of the circle
   */
  drawCircle: function (canvasContext, radius) {
    canvasContext.beginPath();
    canvasContext.arc(radius, radius, radius, 0, 2 * Math.PI, false);
    canvasContext.fillStyle = 'black';
    canvasContext.fill();
  },
  /** Draw a north facing line in the canvas to indicate the pose line
   * @function
   * @param {Object} canvasContext - Stores a reference to the canvas element
   * @param {Number} length - The length of the pose line
   */
  drawPoseLine: function (canvasContext, length) {
    canvasContext.beginPath();
    canvasContext.moveTo(length, length);
    canvasContext.lineTo(length, 0);
    canvasContext.strokeStyle = 'red';
    canvasContext.lineWidth = length / 20;
    canvasContext.stroke();
  },

  /**
   * Rotate the canvas to the current value of angle
   * @function
   * @param {Object} polyCtx - Stores a reference to the polymer element
   * @param {Object} canvasContext - Stores a reference to the canvas element
   * @param {Number} length - The length of the pose line
   */
  rotateToAngle: function (polyCtx, canvasContext, length) {
    var tempCanvas,
      tempCanvasContext;
    // save image
    tempCanvas = document.createElement("canvas");
    tempCanvasContext = tempCanvas.getContext("2d");
    tempCanvas.width = 2 * length;
    tempCanvas.height = 2 * length;
    tempCanvasContext.drawImage(polyCtx.canvas, 0, 0, 2 * length, 2 * length);
    canvasContext.save();
    // center frame of rotation
    canvasContext.translate(length, length);
    // rotate
    canvasContext.rotate(polyCtx.angle);
    // center image
    canvasContext.translate(-length, -length);
    canvasContext.drawImage(tempCanvas, 0, 0);
    canvasContext.restore();
  }


});
