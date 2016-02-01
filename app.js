/* jshint unused: false */
'use strict';

/* --------------------------------------------------------------------------
 * Module Dependencies
 * -------------------------------------------------------------------------- */

// node modules
var path = require('path');

// NPM modules
var express = require('express');
var bodyParser = require('body-parser');
var cookieParser = require('cookie-parser');
var favicon = require('serve-favicon');
var logger = require('morgan');

/* --------------------------------------------------------------------------
 * Variables
 * -------------------------------------------------------------------------- */

// routes
var index = require('./routes/index');
var component = require('./routes/component');

/* --------------------------------------------------------------------------
 * Config
 * -------------------------------------------------------------------------- */

// init
var app = express();
// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'jade');
// favicon
app.use(favicon(path.join(__dirname, 'public', 'images', 'favicon.ico')));
// log requests
app.use(logger('dev'));
// parser middleware
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({extended: false}));
app.use(cookieParser());

// static directories
app.use(express.static(path.join(__dirname, 'public')));
// use Bower statically
app.use('/lib', express.static(path.join(__dirname, 'bower_components')));

// Use routes
app.use('/', index);
app.use('/component', component);

/* --------------------------------------------------------------------------
 * Error Handlers
 * -------------------------------------------------------------------------- */

// catch 404 and forward to error handler
app.use(function(req, res, next) {
  var err = new Error('Not Found');
  err.status = 404;
  next(err);
});
// locals for error template
var errorLocals = {
  title: 'Frontend',
  description: ''
};
// development error handler: will print stacktrace
if (app.get('env') === 'development') {
  app.use(function(err, req, res, next) {
    errorLocals.message = err.message;
    errorLocals.error = err;
    res.status(err.status || 500);
    res.render('error', errorLocals);
  });
}
// production error handler: no stacktraces leaked to user
app.use(function(err, req, res, next) {
  errorLocals.message = err.message;
  errorLocals.error = {};
  res.status(err.status || 500);
  res.render('error', errorLocals);
});

// export server as node module
module.exports = app;
