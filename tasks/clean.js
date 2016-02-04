/**
 * Clean: Remove processed dependencies (delete 'public' directory).
 */

var del = require('del');
var vpaths = require('vinyl-paths');

module.exports = function(gulp, opts, $) {
  gulp.task('clean', function() {
    return gulp.src(opts.dest.public)
    .pipe(vpaths(del))
    .pipe($.size({title: 'clean'}));
  });
}
