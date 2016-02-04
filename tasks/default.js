/**
 * Default: Run the default gulp task.
 */

// local requirements

module.exports = function(gulp, opts, $) {
  gulp.task('default', ['start']);

  // start server
  gulp.task('start', function(cb) {
    if (opts.stage) {
      $.sequence(
        'build',
        'serve',// serve in live reload mode
        cb
      );
    } else if (opts.prod) {
      $.sequence(
        'serve:prod',
        cb
      );
    } else {
      $.sequence(
        'build',
        'serve',
        cb
      );
    }
  });
}
