/**
 * Elements: copy and vulcanize Polymer elements for frontend.
 */

module.exports = function(gulp, opts, $) {
  gulp.task('elements', function() {
    return gulp.src(opts.src.elements.all, {
      base: opts.src.appBase
    })
    .pipe($.if('*.jade', $.jade()))
    .pipe(gulp.dest(opts.dest.public))
    .pipe($.size({title: 'elements'}))
  })
}
