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
    label: String,
    maxPoints: Number,
    useTimeAsX: Boolean,
    staticPlot: Boolean,
    plotType: String,
    height: Number
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
    this.label = props.label;
    this.maxPoints = props.maxPoints || 50; // A default maximum of 50 points
    this.useTimeAsX = (props.useTimeAsX.toLowerCase() === 'true');
    this.staticPlot = (props.staticPlot.toLowerCase() === 'true');
    this.connected = (props.connected.toLowerCase() === 'true');
    this.height = props.height || 80; // A default height of 80 px
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

    // Initialize a blank dataset with a single trace
    this.data = [{
      x: [],
      y: []
    }];

    /*
     * Set the mode to either be just dots or dots with connected lines
     * depending on configuration.
     */
    this.data[0].mode = '';
    if (this.connected) {
      this.data[0].mode = 'lines+';
    }
    this.data[0].mode += 'markers';

    this.plotContainer = document.getElementById(this.uniqueId);
    this.plotContainer.style.height = this.height.toString() + 'px';

    var positionSettings = {
      margin: {t: 0, b: 20, l: 30, r: 30},
      xaxis: {tickangle: 0}
    }, dispSettings = {
      displaylogo: false,
      staticPlot: this.staticPlot
    };

    /*
     * Initialize the plot with some basic default styling and the initial
     * dataset.
     */
    Plotly.plot(this.plotContainer, this.data, positionSettings, dispSettings);
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
      /*
       * Convert ros' stamp format to milliseconds and create date with that
       * time if the plot is configurted to use current time as the x-axis.
       */
      var d = new Date(message.header.stamp.secs * 1000
        + message.header.stamp.nsecs / 1000000);
      this.plotContainer.data[0].x.push(d);
    } else {
      this.plotContainer.data[0].x.push(message.point.x);
    }

    this.plotContainer.data[0].y.push(message.point.y);

    // Bump the oldest data off if the data length exceeds the maximum.
    if (this.plotContainer.data[0].x.length > this.maxPoints) {
      this.plotContainer.data[0].x.shift();
      this.plotContainer.data[0].y.shift();
    }

    // Redraw the entire plot to incorporate the new data.
    Plotly.redraw(this.plotContainer);
  }
});

