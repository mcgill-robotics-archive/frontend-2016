/**
 * @file Defines behaviours for bar component.
 */

/*global Polymer, ROSLIB, frontendInterface */

'use strict';

var BarComponent = Polymer({
  is: "img-feed-component",

  /*
   * Build an object with all Polymer properties for the component. Specify 
   * Polymer properties unique to the component defined, and set their
   * JavaScript types.
   */
  properties: frontendInterface.buildComponentPolymerProps({
    topic: String
  }),

  /**
   * Polymer constructor. Copy initialization properties to the internal
   * component object.
   * @function
   * @param {Object} props - Contains initialization properties for component.
   */
  factoryImpl: function (props) {
    this.topic = props.topic;
  },

  /**
   * Handle topic listening, related response behaviours and any
   * initialization procedure that requires the DOM to be fully loaded.
   * @function
   */
  attached: function () {
    this.uiSetupHelpers.setupMJPEGCanvas.call(this);
  },

  /**
   * Handle component removal.
   * @function
   */
  detached: function () {
    this.topicListener.unsubscribe();
  },
  
  /* Contains all ui setup methods not directly used by Polymer. */
  uiSetupHelpers : {

    /**
     * Create element to visualize image feeds for a particular topic.
     * @function
     */
    setupMJPEGCanvas : function () {
      var viewer = new MJPEGCANVAS.Viewer({
        divID : this.topic,
        host : 'localhost',
        port : '8080',
        width : 640,
        height : 480,
        topic : this.topic
      });
    }
  }
});

