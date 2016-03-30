#!/usr/bin/env nodejs

/**
 * @file Implements document serving and preprocessing functionalities.
 */

'use strict';

var express = require('express'),
  stylus = require('stylus'),
  nib = require('nib'),
  fs = require('fs');

var app = express();
var APP_BASE_PATH = __dirname + '/..';

var layout = process.argv[2] || 'test';

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
  src: APP_BASE_PATH + '/assets',
  dest: APP_BASE_PATH + '/public',
  compile: compileStylus
}));

app.use(express.static(APP_BASE_PATH + '/public'));

/**
 * Serve bower components directory's static contents under the virtual path
 * prefix '/lib' so that any file under '/bower_components/foo/bar' will be
 * accessible as '/lib/foo/bar' in any browser-side references
 */
app.use('/lib', express.static(APP_BASE_PATH + '/bower_components'));

app.set('view engine', 'jade');
app.set('views', APP_BASE_PATH + '/views');

app.get('/', function (req, res) {
  // Render view template 'index.jade'
  res.render('layouts/' + layout);
});

app.get('/homepage', function (req, res) {
  res.render('homepage');
});

/*
 * Handle any requests for a component and return the template 
 * '[type]-component'; pass the component name to the template.
 */
app.get('/component/:type/', function (req, res) {
  // Fetch the component type from the URL parameters
  var componentType = req.params.type + '-component';

  fs.readFile(APP_BASE_PATH +
    '/public/elements/components/' + componentType +
    '/test/index.html', 'utf8', function (err, data) {
      /* 
       * If the test HTML file doesn't exist (or some other error occurred
       * with reading the file), print the error. Otherwise, pass the read
       * file to the component template.
       */
      if (err) {
        return res.send(err.message);
      }
      res.render('component', {component: componentType, testHTML: data});
    });
});


var server = app.listen(3000, '0.0.0.0', function () {
  var host = server.address().address,
    port = server.address().port;
  console.log('\nNode server listening:');
  console.log('Running on http://%s:%s', host, port);
});

