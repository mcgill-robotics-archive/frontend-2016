'use strict';

// Load gulp modules
var gulp = require('gulp');
var $ = require('gulp-load-plugins')();
$.sequence = require('run-sequence');

// Get command line arguments
var argv = require('yargs').argv;

// Load config
var config = require('./tasks/config');
var bs = require('browser-sync').create();

// Define gulp task opts
var opts = {
  src: config.src,
  dest: config.dest,
  dev: argv.dev,
  stage: argv.stage,
  prod: argv.prod || argv.stage, // stage option just turns on production build
  bs: bs,
}

// Load gulp tasks
require('./tasks/lint')(gulp, opts, $);
require('./tasks/clean')(gulp, opts, $);
require('./tasks/styles')(gulp, opts, $);
require('./tasks/scripts')(gulp, opts, $);
require('./tasks/images')(gulp, opts, $);
require('./tasks/elements')(gulp, opts, $);
require('./tasks/serve')(gulp, opts, $);
require('./tasks/build')(gulp, $);
require('./tasks/default')(gulp, opts, $);
require('./tasks/help')(gulp, opts, $);
