/**
 * @file Defines behaviours for base widget cards.
 */

/*global Polymer, ROSLIB, TextComponent, BarComponent, frontendInterface */

'use strict';

var BaseWidget = Polymer({
  is: 'base-widget',

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

  refreshDOM: function () {
    var container = this.getElementsByClassName('componentContainer')[0],
      c;
    container.innerHTML = '';

    for (c in this.components) {
      if(this.components.hasOwnProperty(c)) {
        container.appendChild(this.components[c]);
      }
    }
  },

  addComponent: function (component) {
    this.components.push(component);
    this.refreshDOM();
  },

  attached: function () {
    this.components = [];

    this.addComponent(new BarComponent({
      'topic': '/test_text_topic',
      'message-type': 'std_msgs/Int64',

      'label': 'Number',

      'min': 0,
      'max': 1000,
      'step': 1
    }));
  }
});

