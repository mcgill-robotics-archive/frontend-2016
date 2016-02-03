/**
*/

/*global Polymer, ROSLIB, frontendInterface */

'use strict';

var PoseComponent = Polymer({
  is: "pose-component",
  properties: frontendInterface.buildComponentPolymerProps({
    angle: Number,
    rad: 200,
    canvas: null,
    canvasContext: null,
    tempCanvas: null,
    tempContext: null

  }),

  attached: function () {
    var context = this,
      rad = context.rad,
      height = 2 * rad;
    context.canvas = document.getElementById("canvas");
    context.canvasContext = context.canvas.getContext("2d");
    context.tempCanvas = document.createElement("canvas");
    context.tempContext = context.tempCanvas.getContext("2d");
    context.canvas.width = 2 * context.rad;
    context.canvas.height = 2 * context.rad;
   // draw circle
    context.canvasContext.beginPath();
    context.canvasContext.arc(rad, rad, rad, 0, 2 * Math.PI, false);
    context.canvasContext.fillStyle = 'black';
    context.canvasContext.fill();
  // draw pose line      
    context.canvasContext.beginPath();
    context.canvasContext.moveTo(rad, rad);
    context.canvasContext.lineTo(rad, 0);
    context.canvasContext.strokeStyle = 'blue';
    context.canvasContext.lineWidth = 10;
    context.canvasContext.stroke();
    context.tempCanvas.width = context.canvas.width;
    context.tempCanvas.height = context.canvas.height;
    context.tempContext.drawImage(context.canvas, 0, 0, height, height);
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
      context.handleMessage(context, message);
    });
  },

  /**
   * Handle component removal.
   * @function
   */
  detached: function () {
    this.topicListener.unsubscribe();
  },

  rotate: function (angle, context) {
  // clear
    var rad = context.rad;
    context.canvasContext.clearRect(0, 0, 2 * rad, 2 * rad);
    context.canvasContext.save();
  //center
    context.canvasContext.translate(context.rad, context.rad);
  // rotate
    context.canvasContext.rotate(angle);
    context.canvasContext.translate(-context.rad, -context.rad);

  // Finally draw the image data from the temp canvas.
    context.canvasContext.drawImage(context.tempCanvas, 0, 0);
    context.canvasContext.restore();
    context.requestAnimationFrame(context.rotate);
  },
  /**
   * Recieve message from the subscribed topic and set Polymer property
   * 'value' to the message's data.
   * @function
   * @param {Object} context - Stores a reference to the Polymer element
   * @param {Object} message - Message data from topic
   */
  handleMessage: function (context, message) {
    var newAngle = message.orientation.getAngle();
    context.rotate(newAngle - context.angle, context);
    context.angle = newAngle();
  }

});
