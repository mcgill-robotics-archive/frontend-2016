/**
 * @file Defines behaviours for bar component.
 */

/*global Polymer, ROSLIB, frontendInterface, MJPEGCANVAS */

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

  /* Contains all ui setup methods not directly used by Polymer. */
  uiSetupHelpers : {

    /**
     * Find the optimal size for the mjpegcanvas element based on the
     * the parent DOM element and the aspect ratio.
     * @function
     * @return {Object} dimensions - optimal dimensions for the image stream
     */
    resolveImageSize : function () {
      var fullnessRatio,
        aspectRatio,
        targetElement,
        header,
        useableHeight,
        height,
        width,
        dimensions;

      // Percentage of the space in the parent DOM node to fill with stream
      fullnessRatio = this.fullness || 0.95;

      // Aspect ratio of the particular feed
      aspectRatio = this.aspectRatio || 0.74;

      targetElement = this.parentElement;
      while (targetElement.tagName !== 'PAPER-CARD') {
        targetElement = targetElement.parentElement;
      }

      /*
       * Some of the vertical space is taken up by the header, which is
       * always the first div in the paper card.
       */
      header = targetElement.querySelector('div');
      useableHeight = targetElement.clientHeight - header.clientHeight;
      height = useableHeight * fullnessRatio;

      width = targetElement.clientWidth * fullnessRatio;

      if (height / width <= aspectRatio) {
        width = height / aspectRatio; // height is limiting factor
      } else {
        height = width * aspectRatio; // width is limiting factor
      }

      dimensions = {
        height : height,
        width : width
      };
      return dimensions;
    },

    /**
     * Create element to visualize image feeds for a particular topic.
     * @function
     */
    setupMJPEGCanvas : function () {
      var dimensions,
        viewer;

      dimensions = this.uiSetupHelpers.resolveImageSize.call(this);
      viewer = new MJPEGCANVAS.Viewer({
        divID : this.topic,
        host : 'localhost',
        port : '8080',
        width : dimensions.width,
        height : dimensions.height,
        topic : this.topic
      });
      return viewer;
    }
  }
});

