/**
 * @file Defines behaviours for base widget cards.
 */

/*global Polymer, ROSLIB, TextComponent, BarComponent, frontendInterface */

'use strict';

var BaseWidget = Polymer({
  is: 'base-widget',

  properties: {
    title: String,

    width: Number,
    height: Number,
    x: Number,
    y: Number,
  },

  /**
   * When the component is attached, set up proper widget dimensions.
   * @function
   */
  attached: function () {
    /*
     * Divide 12-column size parameter into a decimal percentage, then
     * multiply by 100 to convert to CSS percentages.
     */
    var container = document.getElementById('interfaceContainer');
    this.customStyle['--widget-width'] = 
      (this.width / 12.0 * 100).toString() + '%';

    this.customStyle['--widget-height'] = (this.height / 12.0 
      * container.clientHeight).toString() + 'px';

    this.customStyle['--widget-left'] = (this.x / 12.0 * 100).toString() + '%';

    this.customStyle['--widget-top'] = (this.y / 12.0
      * container.clientHeight).toString() + 'px';

    this.updateStyles();
  }
});

