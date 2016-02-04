/**
 * Lint: Lint all frontend dependencies.
 */

var stylish = require('gulp-jscs-stylish');
var css_stylish = require('csslint-stylish');
var coffee_stylish = require('coffeelint-stylish');

var noop = function() {};

module.exports = function(gulp, opts, $) {
  gulp.task('lint', [
    'lint:server',
    'lint:views',
    'lint:styles',
    'lint:scripts',
    'lint:elements'
  ]);

  // lint server scripts
  gulp.task('lint:server', function() {
    return gulp.src(opts.src.server.all)
    .pipe($.jshint())
    .pipe($.jscs())
    .on('error', noop) // do nothing on error (i.e. keep running)
    .pipe(stylish.combineWithHintResults())
    .pipe($.jshint.reporter('jshint-stylish'))
    .pipe($.jshint.reporter('fail')); // fail and do not continue tasks
  });

  // lint all views
  gulp.task('lint:views', ['lint:jade']);

  // lint all client styles
  gulp.task('lint:styles', ['lint:stylus', 'lint:css']);

  // lint all client scripts
  gulp.task('lint:scripts', ['lint:coffee', 'lint:js']);

  // lint all polymer elements
  gulp.task('lint:elements', [
    'lint:elements:views',
    'lint:elements:styles',
    'lint:elements:scripts'
  ]);

  // lint client views
  gulp.task('lint:jade', function () {
    return lint_jade(opts.src.views);
  });

  // lint client styles
  gulp.task('lint:stylus', function () {
    return lint_stylus(opts.src.styles.all);
  });

  // lint client styles
  gulp.task('lint:css', function () {
    return lint_css(opts.src.styles.all);
  });

  // lint coffeescript
  gulp.task('lint:coffee', function () {
    return lint_coffee(opts.src.scripts.app);
  });

  // lint client scripts
  gulp.task('lint:js', function () {
    return lint_js(opts.src.scripts.app);
  });

  // lint element views
  gulp.task('lint:elements:views', [
    'lint:elements:views:jade',
    'lint:elements:views:html'
  ]);
  gulp.task('lint:elements:views:jade', function () {
    return lint_jade(opts.src.elements.views);
  });
  gulp.task('lint:elements:views:html', function () {
    return lint_html(opts.src.elements.views);
  });

  // lint element styles
  gulp.task('lint:elements:styles', [
    'lint:elements:styles:stylus',
    'lint:elements:styles:css'
  ]);
  gulp.task('lint:elements:styles:stylus', function () {
    return lint_stylus(opts.src.elements.styles);
  });
  gulp.task('lint:elements:styles:css', function () {
    return lint_css(opts.src.elements.styles);
  });

  // lint element scripts
  gulp.task('lint:elements:scripts', [
    'lint:elements:scripts:coffee',
    'lint:elements:scripts:js'
  ]);
  gulp.task('lint:elements:scripts:coffee', function () {
    return lint_coffee(opts.src.elements.scripts);
  });
  gulp.task('lint:elements:scripts:js', function () {
    return lint_js(opts.src.elements.scripts);
  });

  function lint_jade (files) {
    return gulp.src(files)
    .pipe($.ignore.exclude('*.html'))
    .pipe($.jadelint())
    .pipe($.size({title: 'jadelint'}));
  }

  function lint_html (files) {
    return gulp.src(files)
    .pipe($.ignore.exclude('*.jade'))
    .pipe($.htmlhint('.htmlhintrc'))
    .pipe($.htmlhint.reporter('htmlhint-stylish'))
    .pipe($.htmlhint.failReporter({ suppress: true }))
    .pipe($.size({title: 'htmlhint'}));
  }

  function lint_stylus (files) {
    return gulp.src(files)
    .pipe($.ignore.exclude('*.css'))
    .pipe($.stylint({
      reporter: {
        reporter: 'stylint-stylish',
        reporterOptions: {verbose: true}
      }
    }))
    .pipe($.stylint.reporter())
    .pipe($.if(!opts.bs.active, $.stylint.reporter('fail')))
    .pipe($.size({title: 'stylint'}));
  }

  function lint_css (files) {
    return gulp.src(files)
    .pipe($.ignore.exclude('*.styl'))
    .pipe($.csslint())
    .pipe($.csslint.reporter(css_stylish))
    .pipe($.if(!opts.bs.active, $.csslint.reporter('fail')))
    .pipe($.size({title: 'csslint'}));
  }

  function lint_coffee (files) {
    return gulp.src(files)
    .pipe($.ignore.exclude('*.js'))
    .pipe($.coffeelint())
    .pipe($.coffeelint.reporter(coffee_stylish))
    .pipe($.if(!opts.bs.active, $.coffeelint.reporter('fail')))
    .pipe($.size({title: 'coffeelint'}));
  }

  function lint_js (files) {
    return gulp.src(files)
    .pipe($.ignore.exclude('*.coffee'))
    .pipe($.jshint())
    .pipe($.jscs())
    .on('error', noop)
    .pipe(stylish.combineWithHintResults())
    .pipe($.jshint.reporter('jshint-stylish'))
    .pipe($.if(!opts.bs.active, $.jshint.reporter('fail')))
    .pipe($.size({title: 'jslint'}));
  }
}
