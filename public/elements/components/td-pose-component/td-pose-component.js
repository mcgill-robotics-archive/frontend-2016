/**
 * Define behaviour for 3D pose component
 */

/*global Polymer, ROSLIB, frontendInterface */

'use strict';

var PoseComponent3D = Polymer({
  is: "td-pose-component",
  properties: frontendInterface.buildComponentPolymerProps({
    camera: Object,
    scene: Object,
    renderer: Object,
    arrow: Object,
    aspectRatio: Number,
    width: Number,
    height: Number,
    labelX: String,
    labelY: String,
    labelZ: String
  }),

  /**
   * Initialize component.
   * @function
   */
  attached: function () {
    var polymerContext = this;
    this.width = 400;
    this.height = 400;
    this.aspectRatio = this.width/this.height;
    this.init(polymerContext);

    // Subscribe to topic defined by HTML attribute / Polymer property 'topic'
    this.topicListener = new ROSLIB.Topic({
      ros: frontendInterface.ros,
      name: this.topic,
      messageType: this.messageType
    });

    // Set the message handling function and pass the Polymer element context.
    this.topicListener.subscribe(function (message) {
      polymerContext.handleMessage(polymerContext, message);
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
   * Recieve pose stamped message from the subscribed topic
   * and set Polymer property 'angle' to the yaw.
   * NOTE: 0 deg. is NORTH
   * @function
   * @param {Object} polymerContext - Stores a reference to the Polymer element
   * @param {Object} message - pose stamped message containing updated angle
   */
  handleMessage: function (polymerContext, message) {
    var q = message.pose.orientation;
    polymerContext.rotate(polymerContext, q);
  },

  init: function (polymerContext) {
    console.log("init");
    polymerContext.camera = new THREE.PerspectiveCamera(75,
        polymerContext.aspectRatio, 1, 1000);
    polymerContext.camera.position.z = polymerContext.width;
    polymerContext.scene = new THREE.Scene();
    var dir = new THREE.Vector3(1, 0, 0),
      origin = new THREE.Vector3(0, 0, 0),
      length = polymerContext.width/2,
      hexColour = 0xfff000,
      axisHelper = new THREE.AxisHelper(polymerContext.width/4),
      dm;
    polymerContext.scene.add(axisHelper);
    polymerContext.arrow = new THREE.ArrowHelper(dir,
        origin, length, hexColour);
    polymerContext.scene.add(polymerContext.arrow);
    polymerContext.renderer = new THREE.WebGLRenderer();
    polymerContext.renderer.setSize(polymerContext.width, polymerContext.height);
    dm = polymerContext.renderer.domElement;
    Polymer.dom(polymerContext.root).appendChild(dm);
    polymerContext.controls = new THREE.OrbitControls(polymerContext.camera,
        polymerContext.renderer.domElement);
    console.log(polymerContext.root);
  },

  rotate: function (polymerContext, q) {
    var dir, rotation;
    dir = new THREE.Vector3(1, 0, 0);
    rotation = new THREE.Euler(1, 0, 0, 'XYZ');
    rotation.setFromQuaternion(q, 'XYZ');
    requestAnimationFrame(polymerContext.rotate);
    dir.applyEuler(rotation);
    polymerContext.arrow.setDirection(dir);
    polymerContext.renderer.render(polymerContext.scene, polymerContext.camera);
      polymerContext.updateLabels(polymerContext, rotation);
  },

  updateLabels: function(polymerContext, rotation){
      console.log(rotation._x.toString());
      polymerContext.labelX = rotation._x.toString();
      polymerContext.labelY = rotation._y.toString();
      polymerContext.labelZ = rotation._z.toString();



  }
});
