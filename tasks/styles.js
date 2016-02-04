/**
 * Styles: compile and minify styles for frontend.
 */

var nib = require('nib');

// css autoprefix versions
var AUTOPREFIXER_BROWSERS = [
  'last 3 versions'
];

module.exports = function(gulp, opts, $) {
  gulp.task('styles', function(cb) {
    if (opts.prod) {
      $.sequence(
        'cssmin',
        cb
      );
    } else {
      $.sequence(
        'autoprefix',
        cb
      );
    }
  });

  // compile stylus (copy vanilla css)
  gulp.task('stylus', function() {
    return gulp.src(opts.src.styles.app, {
      base: opts.src.appBase
    })
    .pipe($.if('*.styl', $.stylus({use: [nib()]})))
    .pipe(gulp.dest(opts.dest.public))
    .pipe($.size({title: 'stylus'}));
  });

  // autoprefix CSS
  gulp.task('autoprefix', ['stylus'], function(cb) {
    return gulp.src(opts.src.styles.public, {
      base: opts.src.pubBase
    })
    .pipe($.autoprefixer({
      browsers: AUTOPREFIXER_BROWSERS
    }))
    .pipe(gulp.dest(opts.dest.public))
    .pipe($.size({title: 'autoprefix'}));
  });

  // minimize CSS
  gulp.task('cssmin', ['autoprefix'], function() {
    return gulp.src(opts.src.styles.public, {
      base: opts.src.pubBase
    })
    .pipe($.cssmin())
    .pipe(gulp.dest(opts.dest.public))
    .pipe($.size({title: 'cssmin'}));
  });
}
