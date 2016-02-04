/**
 * Images: minify images in production for frontend.
 */

module.exports = function(gulp, opts, $) {
  gulp.task('images', function() {
    return gulp.src(opts.src.images, {
      base: opts.src.appBase
    })
    .pipe($.if(opts.prod, $.imagemin({
      progressive: true,
      interlaced: true
    })))
    .pipe(gulp.dest(opts.dest.public))
    .pipe($.size({title: 'images'}));
  });
}
