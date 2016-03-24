/**
 * @file Defines behaviours for plot component.
 */

/*global Polymer, ROSLIB, frontendInterface */

'use strict';

var PlotComponent = Polymer({
  is: "plot-component",

  /*
   * Build an object with all Polymer properties for the component. Specify 
   * Polymer properties unique to the component defined, and set their
   * JavaScript types.
   */
  properties: frontendInterface.buildComponentPolymerProps({
    uniqueID: String
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
    this.uniqueID = props.uniqueId;
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

    console.log(this.topic);

    Plotly.plot(document.getElementById(this.uniqueID), [{
        x: [1, 2, 3, 4, 5],
        y: [1, 2, 4, 8, 16]
    }], { margin: {t: 0} });
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
   * @param {Object} context - Stores a reference to the Polymer element
   * @param {Object} message - Message data from topic
   */
  handleMessage: function (message) {
    // Handle adding new value to plot element
  }
});

