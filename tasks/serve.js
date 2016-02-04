/**
 * Serve: Start the server in either production or dev (with live-reload).
 */

var ports = require('../modules/ports');

// delay before reloading browser
var BS_RELOAD = 500;

// prefix all BS logs
var BS_LOG_PREFIX = 'MR';

module.exports = function(gulp, opts, $) {
  gulp.task('serve', ['browser-sync']);

  // start server in production mode
  gulp.task('serve:prod', $.shell.task([
    'NODE_ENV=production node ' + opts.src.server.main
  ]));

  // live reload server
  gulp.task('nodemon', function(cb) {
    var started = false;
    return $.nodemon({
      // nodemon the express server
      script: opts.src.server.main,
      // dev mode
      env: {
        'NODE_ENV': 'development',
        'DEBUG': 'frontend'
      },
      // watch core server files
      watch: opts.src.server.all,
      // lint server scripts if changed
      tasks: ['lint:server']
    })
    .on('start', function onStart() {
      if (!started) {
        cb();
      };
      started = true;
    })
    .on('restart', function onRestart() {
      // reload connected browsers after delay
      setTimeout(function load() {
        opts.bs.reload({
          stream: false
        });
      }, BS_RELOAD);
    });
  });

  // live reload browser with LAN
  gulp.task('browser-sync', ['nodemon'], function() {
    opts.bs.init({
      // proxy expressjs app
      proxy: 'localhost:' + ports.express,
      // port for browsersync
      port: ports.bs,
      // port for ui
      ui: {
        port: ports.bsui,
        weinre: {
          port: ports.bsw
        }
      },
      // prefix before log messages
      logPrefix: BS_LOG_PREFIX,
      // log connections to server
      logConnections: true,
      // disable notificaitons
      notify: false,
      // disable automatic url opening
      open: false,
      // reload open windows on restart
      reloadOnRestart: true
    });

    // watch during development
    gulp.watch(opts.src.views, ['watch-views', opts.bs.reload]);
    gulp.watch(opts.src.styles.all, ['watch-styles', opts.bs.reload]);
    gulp.watch(opts.src.scripts.app, ['watch-scripts', opts.bs.reload]);
    gulp.watch(opts.src.images, ['watch-images', opts.bs.reload]);
    gulp.watch(opts.src.elements.all, ['watch-elements', opts.bs.reload]);
  });

  gulp.task('watch-views', function (cb) {
    $.sequence(
      'lint:views',
      cb
    );
  });
  gulp.task('watch-styles', function (cb) {
    $.sequence(
      'lint:styles',
      'styles',
      cb
    );
  });
  gulp.task('watch-scripts', function (cb) {
    $.sequence(
      'lint:scripts',
      'scripts',
      cb
    );
  });
  gulp.task('watch-images', ['images']);
  gulp.task('watch-elements', function (cb) {
    $.sequence(
      'lint:elements',
      'elements',
      cb
    );
  });
}
