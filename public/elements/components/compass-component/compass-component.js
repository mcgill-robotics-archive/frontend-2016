/**
 * Define behaviour for 2D pose component, which display a compass like figure
 */

/*global Polymer, ROSLIB, frontendInterface, THREE */

'use strict';

var PoseComponent = Polymer({
  is: "compass-component",
  properties: frontendInterface.buildComponentPolymerProps({
    angle: Number,
    canvas: Object,
    compassRadius: Number
  }),

  /**
   * Initialize component.
   * @function
   */
  attached: function () {
    var polymerContext = this;
    this.initCanvas(polymerContext);

    // Subscribe to topic defined by HTML attribute / Polymer property 'topic'
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
    var q,
      rotation;

    q = message.pose.orientation;
    rotation = new THREE.Euler(0, 0, 0, 'XYZ');
    rotation.setFromQuaternion(q, 'XYZ');

    polymerContext.angle = rotation.z; // Update current angle
    polymerContext.rotate(polymerContext); // Rotate image

  },

  /**
   * Handle adjusting angle of pose line
   * @function
   * @param {Object} polymerContext - Stores a reference to the Polymer element
   */
  rotate: function (polymerContext) {
    var canvasContext = this.canvas.getContext("2d"),
      radius = this.compassRadius;
    canvasContext.clearRect(0, 0, 2 * radius, 2 * radius);
    this.drawCircle(canvasContext, radius);
    this.drawPoseLine(canvasContext, radius);
    this.rotateToAngle(polymerContext, canvasContext, radius);
    this.addVisualCues(polymerContext, canvasContext);
  },

  /**
   * Create the html canvas object to contain the compass
   * @function
   * @param {Object} polymerContext - Reference to the Polymer element
   */
  initCanvas: function (polymerContext) {
    var canvas = document.createElement('canvas'), // Set up canvas
      container = polymerContext.parentElement,
      limitingDimension = null;

    while (container.tagName !== 'PAPER-CARD') {
      container = container.parentElement;
    }

    limitingDimension = Math.min(container.clientHeight - 90,
        container.clientWidth);
    polymerContext.compassRadius = limitingDimension / 2;
    canvas.height = 2 * polymerContext.compassRadius;
    canvas.width = 2 * polymerContext.compassRadius;
    canvas.style.position = "relative";
    Polymer.dom(polymerContext.root).appendChild(canvas);
    polymerContext.canvas = canvas;
  },

  /**
   * Draw a circle in the canvas for the background
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

  /**
   * Draw a north facing line in the canvas to indicate the pose line
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

    tempCanvas = document.createElement("canvas"); // Save image
    tempCanvasContext = tempCanvas.getContext("2d");
    tempCanvas.width = 2 * length;
    tempCanvas.height = 2 * length;
    tempCanvasContext.drawImage(polyCtx.canvas, 0, 0, 2 * length, 2 * length);
    canvasContext.save();

    canvasContext.translate(length, length); // Center frame of rotation
    canvasContext.rotate(polyCtx.angle);
    canvasContext.translate(-length, -length);
    canvasContext.drawImage(tempCanvas, 0, 0);
    canvasContext.restore();
  },

 /**
   * Draw lines and add labels to the 0, 90, 180 and 270 degree
   * locations on the compass canvas
   * @function
   * @param {Object} polymerContext - Stores a reference to the polymer element
   * @param {Object} canvasContext - Stores a reference to the canvas element
   */
  addVisualCues: function (polymerContext, canvasContext) {
    this.addLines(polymerContext, canvasContext);
    this.labelAngles(polymerContext, canvasContext);
  },

 /**
   * Draw marklines to 0, 90, 180  and 270 degree
   * locations on the compass canvas
   * @function
   * @param {Object} polymerContext - Reference to the polymer element
   * @param {Object} canvasContext - Reference to the canvas element
   */
  addLines: function (polymerContext, canvasContext) {
    var max = polymerContext.compassRadius,
      start = (1 / 8) * max,
      maxOffset = 2 * max - start;
    this.drawLine(polymerContext, canvasContext, max, 0, max, start);
    this.drawLine(polymerContext, canvasContext, max, 2 * max, max, maxOffset);
    this.drawLine(polymerContext, canvasContext, 0, max, start, max);
    this.drawLine(polymerContext, canvasContext, 2 * max, max, maxOffset, max);
  },

/**
   * Label angles at 0, 90, 180  and 270 degree locations on the compass canvas
   * @function
   * @param {Object} polymerContext - Stores a reference to the polymer element
   * @param {Object} canvasContext - Stores a reference to the canvas element
   */
  labelAngles: function (polymerContext, canvasContext) {
    var max = polymerContext.compassRadius,
      start = (1 / 4) * max,
      maxOffset = (2 * max) - start;
    canvasContext.fillStyle = 'white';
    canvasContext.font = '14pt Arial';
    canvasContext.textBaseline = 'middle';
    canvasContext.textAlign = "center";
    canvasContext.fillText("0", max, start);
    canvasContext.fillText("90 ", maxOffset, max);
    canvasContext.fillText("180", max, maxOffset);
    canvasContext.fillText("270", start, max);
  },

 /**
   * Draw line from (x1,y1) to (x2,y2) canvas
   * @function
   * @param {Object} polymerContetx - Stores a reference to the polymer element
   * @param {Object} canvasContext - Stores a reference to the canvas element
   * @param {Number} x1 x poistion of line start
   * @param {Number} y1 y poistion of line start
   * @param {Number} x2 x poistion of line end
   * @param {Number} y2 y poistion of line end
   */
  drawLine: function (polymerContext, canvasContext, x1, y1, x2, y2) {
    canvasContext.beginPath();
    canvasContext.moveTo(x1, y1);
    canvasContext.lineTo(x2, y2);
    canvasContext.strokeStyle = 'white';
    canvasContext.lineWidth = polymerContext.compassRadius / 40;
    canvasContext.stroke();
  }
});
