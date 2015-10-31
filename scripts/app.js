#!/usr/bin/env nodejs

var express = require('express');
var app = express();

app.get('/', function (req, res) {
  res.send('Hello McGill Robotics!');
});


var server = app.listen(3000, '0.0.0.0', function() {
  var host = server.address().address;
  var port = server.address().port;
  console.log('\nNode server listening:');
  console.log('Running on http://%s:%s', host, port);
});
