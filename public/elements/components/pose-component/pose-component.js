/**
*/

/*global Polymer, ROSLIB, frontendInterface */

'use strict';

var PoseComponent = Polymer({
  is: "pose-component",
  properties: frontendInterface.buildComponentPolymerProps({
    angle: Number,
    canvas: Object,
    rad: Number
  }),

  /**
   * handle adjusting angle of pose line
   * @function
   * @param {Object} context - Stores a reference to the Polymer element
   */
  rotate: function (context) {
    var ctx = this.canvas.getContext("2d"),
      rad = this.rad,
      tempCanvas,
      tempCtx;
    ctx.clearRect(0, 0, 2 * rad, 2 * rad);
    // draw circle, circle may be unnecessary
    ctx.beginPath();
    ctx.arc(rad, rad, rad, 0, 2 * Math.PI, false);
    ctx.fillStyle = 'black';
    ctx.fill();
    // draw pose line      
    ctx.beginPath();
    ctx.moveTo(rad, rad);
    ctx.lineTo(rad, 0);
    ctx.strokeStyle = 'red';
    ctx.lineWidth = rad / 20;
    ctx.stroke();
    // save image
    tempCanvas = document.createElement("canvas");
    tempCtx = tempCanvas.getContext("2d");
    tempCanvas.width = 2 * rad;
    tempCanvas.height = 2 * rad;
    tempCtx.drawImage(this.canvas, 0, 0, 2 * rad, 2 * rad);
    ctx.save();
    // center
    ctx.translate(rad, rad);
    // rotate
    ctx.rotate(context.angle);
    // center rotate
    ctx.translate(-rad, -rad);
    ctx.drawImage(tempCanvas, 0, 0);
    ctx.restore();
  },
  attached: function () {
    var context = this,
    // Set up the canvas... 
      canvas = document.createElement('canvas');
    canvas.id = "canvas";
    //should do something about style, also %
    canvas.style.height  = '100%';
    canvas.height  = canvas.width;
    this.rad = canvas.width / 2;
    canvas.style.position = "relative";
    Polymer.dom(this.root).appendChild(canvas);
    this.canvas = canvas;

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
  /**
   * Recieve pose stamped message from the subscribed topic 
   * and set Polymer property 'angle' to the yaw.
   * NOTE: 0 deg. is NORTH
   */
  handleMessage: function (context, message) {
    // message orientation is a quaternion. convert to euler - yaw
    var q = message.pose.orientation,
      temp1 = 2.0 * ((q.x * q.y) + (q.w * q.z)),
      temp2 = (q.w * q.w) - (q.z * q.z) - (q.y * q.y) + (q.x * q.x),
      yaw = Math.atan2(temp1, temp2);
    //update current angle
    context.angle = yaw;
    //rotate image
    context.rotate(context);
  }
});
