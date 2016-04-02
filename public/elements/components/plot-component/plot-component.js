/**
 * @file Defines behaviours for plot component.
 */

/*global Polymer, ROSLIB, Plotly, frontendInterface */

'use strict';

var PlotComponent = Polymer({
  is: "plot-component",

  /*
   * Build an object with all Polymer properties for the component. Specify 
   * Polymer properties unique to the component defined, and set their
   * JavaScript types.
   */
  properties: frontendInterface.buildComponentPolymerProps({
    uniqueId: String,
    maxPoints: Number,
    useTimeAsX: Boolean
  }),

  /**
   * Polymer constructor. Copy initialization properties to the internal
   * component object.
   * @function
   * @param {Object} props - Contains initialization properties for component.
   */
  factoryImpl: function (props) {
    this.topic = props.topic;
    this.messageType = props.messageTypes;
    this.uniqueId = props.uniqueId;
    this.maxPoints = props.maxPoints || 50;
    this.useTimeAsX = (props.useTimeAsX.toLowerCase() === 'true');
  },

  /**
   * Handle topic listening, related response behaviours and any
   * initialization procedure that requires the DOM to be fully loaded.
   * @function
   */
  attached: function () {
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
    this.topicListener.subscribe(this.handleMessage.bind(this));

    this.data = [{
      x: [],
      y: []
    }];

    this.plotContainer = document.getElementById(this.uniqueId);

    Plotly.plot(this.plotContainer, this.data, {
      margin: {t: 0, b: 20, l: 10, r: 10}
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
   * Recieve message from subscribed topic.
   * @function
   * @param {Object} message - Message data from topic
   */
  handleMessage: function (message) {
    // Handle adding new value to plot element

    if (this.useTimeAsX) {
      var d = new Date(message.header.stamp.secs * 1000
        + message.header.stamp.nsecs / 1000000);
      this.plotContainer.data[0].x.push(d);
    } else {
      this.plotContainer.data[0].x.push(message.point.x);
    }

    this.plotContainer.data[0].y.push(message.point.y);

    if (this.plotContainer.data[0].x.length > this.maxPoints) {
      this.plotContainer.data[0].x.shift();
      this.plotContainer.data[0].y.shift();
    }

    Plotly.redraw(this.plotContainer);
  }
});

