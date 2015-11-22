/**
 * @file Defines behaviours for bar component.
 */

/*global Polymer, ROSLIB, frontendInterface */

'use strict';

Polymer({
  is: "bar-component",

  /*
   * Build an object with all Polymer properties for the component. Specify 
   * Polymer properties unique to the component defined, and set their
   * JavaScript types.
   */
  properties: frontendInterface.buildComponentPolymerProps({
    label: String,
    value: Number,
    min: Number,
    max: Number,
    step: Number
  }),

  /**
   * Handle topic listening, related response behaviours and any
   * initialization procedure that requires the DOM to be fully loaded.
   * @function
   */
  attached: function () {
    var context = this;

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
   * Recieve message from subscribed topic and set Polymer property 'value'
   * (the current bar fill) to the message's data.
   * @function
   * @param {Object} context - Stores a reference to the Polymer element
   * @param {Object} message - Message data from topic
   */
  handleMessage: function (context, message) {
    context.value = Number(message.data);
  }
});

