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

    components: Array
  },

  factoryImpl: function (position) {
    this.position = position;
    this.components = [];

    this.addComponent(new BarComponent({
      'topic': '/test_text_topic',
      'message-type': 'std_msgs/Int64',

      'label': 'Number',

      'min': 0,
      'max': 1000,
      'step': 1
    }));
  },

  attached: function () {
    var container = document.getElementById('interfaceContainer');
    console.log(container.clientHeight);
    this.customStyle['--widget-width'] = (this.width / 12.0 * 100).toString()
      + '%';
    this.customStyle['--widget-height'] = (this.height / 12.0
      * container.clientHeight).toString() + 'px';
    this.customStyle['--widget-left'] = (this.x / 12.0 * 100).toString() + '%';
    this.customStyle['--widget-top'] = (this.y / 12.0
      * container.clientHeight).toString() + 'px';
    this.updateStyles();
  }
});

