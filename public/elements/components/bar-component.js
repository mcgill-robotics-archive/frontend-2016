/**
 * @file Defines behaviours for bar component.
 */

/*global Polymer, ROSLIB, frontendInterface */

'use strict';

/*
 * Import shared set of base Polymer properties from front end interface 
 * object global, initialized in main template files.
 */
var polymerProperties = frontendInterface.getPolymerBaseProperties();

/*
 * Add and specify Polymer properties unique to the component defined, and 
 * set their JavaScript types.
 */
polymerProperties.label = String;
polymerProperties.value = Number;
polymerProperties.min = Number;
polymerProperties.max = Number;
polymerProperties.step = Number;

Polymer({
  is: "bar-component",
  properties: polymerProperties,

  /**
   * Handle topic listening, related response behaviours and any
   * initialization procedure that requires the DOM to be fully loaded.
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
   */
  detached: function () {
    this.topicListener.unsubscribe();
  },

  /**
   * Recieve message from subscribed topic and set Polymer property 'value'
   * (the current bar fill) to the message's data.
   * @function
   * @param {object} context - Stores a reference to the Polymer element
   * @param {object} message - Message data from topic
   */
  handleMessage: function (context, message) {
    context.value = Number(message.data);
  }
});

