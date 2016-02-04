/**
 * Scripts: Compile and minify scripts for frontend.
 */

module.exports = function(gulp, opts, $) {
  gulp.task('scripts', function(cb) {
    if (opts.prod) {
      $.sequence(
        'uglify',
        cb
      );
    } else {
      $.sequence(
        'coffee',
        cb
      );
    }
  });

  // compile coffee (copy vanilla js)
  gulp.task('coffee', function() {
    return gulp.src(opts.src.scripts.app, {
      base: opts.src.appBase
    })
    .pipe($.if('*.coffee', $.coffee()))
    .pipe(gulp.dest(opts.dest.public))
    .pipe($.size({title: 'coffee'}));
  });

  // uglify scripts in production
  gulp.task('uglify', ['coffee'], function() {
    return gulp.src(opts.src.scripts.public, {
      base: opts.src.pubBase
    })
    .pipe($.uglify())
    .pipe(gulp.dest(opts.dest.public))
    .pipe($.size({title: 'uglify'}));
  });
}
