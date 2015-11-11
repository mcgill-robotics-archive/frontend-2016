#!/usr/bin/env nodejs

/**
 * @file Implements document serving and preprocessing functionalities.
 */

'use strict';

var express = require('express'),
  stylus = require('stylus'),
  nib = require('nib');

var app = express();
var appBasePath = __dirname + '/..';

/**
 * Compile stylus files and place them at the given path
 * @constructor
 * @param {string} str - Unprocessed stylus code
 * @param {string} path - Output path for resulting CSS
 */
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
  // Render view template 'index.jade'
  res.render('index');
});

/*
 * Handle any requests for a component and return the template 
 * '[type]-component'; pass the component name to the template.
 */
app.get('/component/:type/', function (req, res) {
  // Fetch the component type from the URL parameters
  var componentType = req.params.type + '-component';
  res.render(componentType, {component: componentType});
});

var server = app.listen(3000, '0.0.0.0', function () {
  var host = server.address().address,
    port = server.address().port;
  console.log('\nNode server listening:');
  console.log('Running on http://%s:%s', host, port);
});

