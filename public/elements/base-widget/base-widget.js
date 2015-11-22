/**
 * @file Defines behaviours for base widget cards.
 */

/*global Polymer, ROSLIB, TextComponent, BarComponent, frontendInterface */

'use strict';

var BaseWidget = Polymer({
  is: "base-widget",

    /*
     * Build an object with all Polymer properties 
     * for the base widget. Testing file
     */

  properties: {
    size: Object,
    position: Object,
    components: Array
  },

  factoryImpl: function (props) {
    this.size = props.size;
    this.position = props.position;
  },

  addComponent: function (component) {
    this.components.push(component);
  },

  attached: function () {
    this.addComponent(new BarComponent());
  }
});

