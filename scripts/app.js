#!/usr/bin/env nodejs

var express = require('express');
var app = express();

app.use(express.static(__dirname + '/../public'));

app.set('view engine', 'jade');
app.set('views', __dirname + '/../views');

app.get('/', function (req, res) {
  res.render('index', { message: 'Hello McGill Robotics!!' });
});


var server = app.listen(3000, '0.0.0.0', function() {
  var host = server.address().address;
  var port = server.address().port;
  console.log('\nNode server listening:');
  console.log('Running on http://%s:%s', host, port);
});

