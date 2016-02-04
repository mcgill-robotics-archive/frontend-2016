/**
 * Build: run build task (see subtasks for more information).
 */

module.exports = function(gulp, $) {
  gulp.task('build', function(cb) {
    $.sequence(
      'lint',// lint first
      'clean',
      ['styles','scripts','images', 'elements'],
      cb
    );
  });
}
