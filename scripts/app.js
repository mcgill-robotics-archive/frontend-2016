#!/usr/bin/env nodejs

var roslib = require('roslib');
var express = require('express');
var app = express();


// Create Ros node object to communicate with rosbridge
var ros = new roslib.Ros({
  url : 'ws://localhost:8080'
});


// Listeners for connection, error and close events
ros.on('connection', function () {
  console.log('Connected to WebSocket Server.');
});

ros.on('error', function (error) {
  console.log('Connection to websocket server closed.');
});

ros.on('close', function () {
  console.log('Connection to websocket server closed.');
});


// Sample listener for String messages
var listener = new roslib.Topic({
  ros : ros, 
  name : '/listener',
  messageType : 'std_msgs/String'
});

listener.subscribe(function (message) {
  console.log('Received message on ' + listener.name + ': ' + message.data);
  listener.unsubscribe();
});


app.get('/', function (req, res) {
  res.send('Hello McGill Robotics!');
});


var server = app.listen(3000, '0.0.0.0', function() {
  var host = server.address().address;
  var port = server.address().port;
  console.log('\nNode server listening:');
  console.log('Running on http://%s:%s', host, port);
});
