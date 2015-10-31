#!/usr/bin/env nodejs

var express = require('express');
var stylus = require('stylus');
var nib = require('nib');

function compileStylus (str, path) {
  return stylus(str)
      .set('filename', path)
      .use(nib());
}

var app = express();
var appBasePath = __dirname + '/..';

app.use(stylus.middleware({
  src: appBasePath + '/assets',
  dest: appBasePath + '/public',
  compile: compileStylus
}));

app.use(express.static(appBasePath + '/public'));
app.use('/static', express.static(appBasePath + '/bower_components'));

app.set('view engine', 'jade');
app.set('views', appBasePath + '/views');

app.get('/', function (req, res) {
  res.render('index', { message: 'Hello McGill Robotics!!' });
});


var server = app.listen(3000, '0.0.0.0', function() {
  var host = server.address().address;
  var port = server.address().port;
  console.log('\nNode server listening:');
  console.log('Running on http://%s:%s', host, port);
});

