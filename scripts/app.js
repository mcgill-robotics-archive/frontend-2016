#!/usr/bin/env nodejs

"use strict";

var express = require('express'),
  stylus = require('stylus'),
  nib = require('nib');

var app = express();

var appBasePath = __dirname + '/..';

function compileStylus(str, path) {
  return stylus(str).set('filename', path).use(nib());
}

app.use(stylus.middleware({
  src: appBasePath + '/assets',
  dest: appBasePath + '/public',
  compile: compileStylus
}));

app.use(express.static(appBasePath + '/public'));

/**
 * Serve bower components directory's static contents under the virtual path
 * prefix '/lib' so that any file under '/bower_components/foo/bar' will be
 * accessible as '/lib/foo/bar' in any browser-side references
 */
app.use('/lib', express.static(appBasePath + '/bower_components'));

app.set('view engine', 'jade');
app.set('views', appBasePath + '/views');

app.get('/', function (req, res) {
  // Render placeholder view template 'index.jade' with a message
  res.render('index', {message: 'Hello McGill Robotics!'});
});

var server = app.listen(3000, '0.0.0.0', function () {
  var host = server.address().address,
    port = server.address().port;
  console.log('\nNode server listening:');
  console.log('Running on http://%s:%s', host, port);
});

