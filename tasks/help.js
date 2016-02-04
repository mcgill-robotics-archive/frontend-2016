/**
 * Help: provide command line ussage for frontend gulp tasks.
 */

var taskListing = require('gulp-task-listing');

module.exports = function(gulp, opts, $) {
  gulp.task('help', taskListing.withFilters(
    function(task) {
      switch(task) {
        case 'start':
          return false;
        case 'build':
          return false;
        case 'serve':
          return false;
        case 'lint':
          return false;
        case 'clean':
          return false;
        case 'styles':
          return false;
        case 'scripts':
          return false;
        case 'images':
          return false;
        case 'elements':
          return false;
        case 'test':
          return false;
        default:
          return true;
      }
    }, /[:-]/
  ));
}
