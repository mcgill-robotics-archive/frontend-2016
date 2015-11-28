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
    position: Object,
    components: Array
  },

  factoryImpl: function (position) {
    this.position = position;
  },

  addComponent: function (component) {
    this.components.push(component);
  },

  attached: function () {
    this.components = [];
    this.addComponent(new BarComponent());
  }
});

