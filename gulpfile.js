'use strict';

/* --------------------------------------------------------------------------
 * Module Dependencies
 * -------------------------------------------------------------------------- */

// node modules
var path = require('path');

// NPM modules
var argv = require('yargs').argv;
var sequence = require('run-sequence');
var del = require('del');
var vpaths = require('vinyl-paths');
// TODO: use deprecated init because recommended doesnt reload
var bs = require('browser-sync').create();
var nib = require('nib');

// gulp modules
var gulp = require('gulp');
var $ = require('gulp-load-plugins')();
var stylish = require('gulp-jscs-stylish');
var css_stylish = require('csslint-stylish');
var coffee_stylish = require('coffeelint-stylish');

// local modules
var ports = require('./modules/ports');
// TODO: create a task-loader module for modular tasks (clean up gulpfile)

/* --------------------------------------------------------------------------
 * Config
 * -------------------------------------------------------------------------- */

// delay before reloading browser
var BS_RELOAD = 500;

// prefix all BS logs
var BS_LOG_PREFIX = 'MR';

// css autoprefix versions
var AUTOPREFIXER_BROWSERS = [
    'last 3 versions'
];

// no operation: empty function
var noop = function() {};

// stage option just turns on production build
if (argv.stage) {
    argv.prod = true;
}

/* --------------------------------------------------------------------------
 * Source and destination folders/files
 * -------------------------------------------------------------------------- */

var src = {
    // server files
    server: {
        main: 'server.js',
        all: [
            'server.js',
            'app.js',
            'routes/*.js'
        ]
    },
    // base
    appBase: 'app',
    pubBase: 'public',
    // all views
    views: ['views/*.jade'],
    // all styles
    styles: {
        all: ['app/styles/**/*.{styl,css}'],
        app: [
            'app/styles/**/*.{styl,css}',
            '!app/styles/theme/**/*'
        ],
        public: ['public/styles/**/*.css']
    },
    // all scripts
    scripts: {
        app: ['app/scripts/**/*.{coffee,js}'],
        public: ['public/scripts/**/*.js']
    },
    // all images
    images: ['app/images/**/*.{ico,jpg,png}'],
    // all elements
    elements: {
        all: ['app/elements/**/*'],
        views: ['app/elements/**/*.{jade,html}'],
        styles: ['app/elements/**/*.{styl,css}'],
        scripts: ['app/elements/**/*.{coffee,js}'],
        vulcanize: ['public/elements/index-elements.html']
    }
};

var dest = {
    public: 'public'
};

/* --------------------------------------------------------------------------
 * Task: Default
 * -------------------------------------------------------------------------- */

gulp.task('default', ['start']);

// start server
gulp.task('start', function(cb) {
    if (argv.stage) {
        sequence(
            'build',
            'serve',// serve in live reload mode
            cb
        );
    } else if (argv.prod) {
        sequence(
            'serve:prod',
            cb
        );
    } else {
        sequence(
            'build',
            'serve',
            cb
        );
    }
});

/* --------------------------------------------------------------------------
 * Task: Clean
 * -------------------------------------------------------------------------- */

gulp.task('clean', function() {
    return gulp.src(dest.public)
    .pipe(vpaths(del))
    .pipe($.size({title: 'clean'}));
});

/* --------------------------------------------------------------------------
 * Task: Build
 * -------------------------------------------------------------------------- */

gulp.task('build', function(cb) {
    sequence(
        'lint',// lint first
        'clean',
        ['styles','scripts','images', 'elements'],
        cb
    );
});

/* --------------------------------------------------------------------------
 * Task: Serve
 * -------------------------------------------------------------------------- */

gulp.task('serve', ['browser-sync']);

// start server in production mode
gulp.task('serve:prod', $.shell.task([
    'NODE_ENV=production node ' + src.server.main
]));

// live reload server
gulp.task('nodemon', function(cb) {
    var started = false;
    return $.nodemon({
        // nodemon the express server
        script: src.server.main,
        // dev mode
        env: {
            'NODE_ENV': 'development',
            'DEBUG': 'frontend'
        },
        // watch core server files
        watch: src.server.all,
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
            reload({
                stream: false
            });
        }, BS_RELOAD);
    });
});

// live reload browser with LAN
gulp.task('browser-sync', ['nodemon'], function() {
    bs.init({
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
    gulp.watch(src.views, ['watch-views', bs.reload]);
    gulp.watch(src.styles.all, ['watch-styles', bs.reload]);
    gulp.watch(src.scripts.app, ['watch-scripts', bs.reload]);
    gulp.watch(src.images, ['watch-images', bs.reload]);
    gulp.watch(src.elements.all, ['watch-elements', bs.reload]);
});

gulp.task('watch-views', function (cb) {
    sequence(
        'lint:views',
        cb
    );
});
gulp.task('watch-styles', function (cb) {
    sequence(
        'lint:styles',
        'styles',
        cb
    );
});
gulp.task('watch-scripts', function (cb) {
    sequence(
        'lint:scripts',
        'scripts',
        cb
    );
});
gulp.task('watch-images', ['images']);
gulp.task('watch-elements', function (cb) {
    sequence(
        'lint:elements',
        'elements',
        cb
    );
});

 /* --------------------------------------------------------------------------
 * Task: Lint
 * -------------------------------------------------------------------------- */

gulp.task('lint', [
    'lint:server',
    'lint:views',
    'lint:styles',
    'lint:scripts',
    'lint:elements'
]);

// lint server scripts
gulp.task('lint:server', function() {
    return gulp.src(src.server.all)
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
    return lint_jade(src.views);
});

// lint client styles
gulp.task('lint:stylus', function () {
    return lint_stylus(src.styles.all);
});

// lint client styles
gulp.task('lint:css', function () {
    return lint_css(src.styles.all);
});

// lint coffeescript
gulp.task('lint:coffee', function () {
    return lint_coffee(src.scripts.app);
});

// lint client scripts
gulp.task('lint:js', function () {
    return lint_js(src.scripts.app);
});

// lint element views
gulp.task('lint:elements:views', [
    'lint:elements:views:jade',
    'lint:elements:views:html'
]);
gulp.task('lint:elements:views:jade', function () {
    return lint_jade(src.elements.views);
});
gulp.task('lint:elements:views:html', function () {
    return lint_html(src.elements.views);
});

// lint element styles
gulp.task('lint:elements:styles', [
    'lint:elements:styles:stylus',
    'lint:elements:styles:css'
]);
gulp.task('lint:elements:styles:stylus', function () {
    return lint_stylus(src.elements.styles);
});
gulp.task('lint:elements:styles:css', function () {
    return lint_css(src.elements.styles);
});

// lint element scripts
gulp.task('lint:elements:scripts', [
    'lint:elements:scripts:coffee',
    'lint:elements:scripts:js'
]);
gulp.task('lint:elements:scripts:coffee', function () {
    return lint_coffee(src.elements.scripts);
});
gulp.task('lint:elements:scripts:js', function () {
    return lint_js(src.elements.scripts);
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
    .pipe($.if(!bs.active, $.stylint.reporter('fail')))
    .pipe($.size({title: 'stylint'}));
}

function lint_css (files) {
    return gulp.src(files)
    .pipe($.ignore.exclude('*.styl'))
    .pipe($.csslint())
    .pipe($.csslint.reporter(css_stylish))
    .pipe($.if(!bs.active, $.csslint.reporter('fail')))
    .pipe($.size({title: 'csslint'}));
}

function lint_coffee (files) {
    return gulp.src(files)
    .pipe($.ignore.exclude('*.js'))
    .pipe($.coffeelint())
    .pipe($.coffeelint.reporter(coffee_stylish))
    .pipe($.if(!bs.active, $.coffeelint.reporter('fail')))
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
    .pipe($.if(!bs.active, $.jshint.reporter('fail')))
    .pipe($.size({title: 'jslint'}));
}

function lint_json (files) {
    // TODO: lint package, bower, etc
}

/* --------------------------------------------------------------------------
 * Task: Styles
 * -------------------------------------------------------------------------- */

gulp.task('styles', function(cb) {
    if (argv.prod) {
        sequence(
            'cssmin',
            cb
        );
    } else {
        sequence(
            'autoprefix',
            cb
        );
    }
});

// compile stylus (copy vanilla css)
gulp.task('stylus', function() {
    return gulp.src(src.styles.app, {
        base: src.appBase
    })
    .pipe($.if('*.styl', $.stylus({use: [nib()]})))
    .pipe(gulp.dest(dest.public))
    .pipe($.size({title: 'stylus'}));
});

// autoprefix CSS
gulp.task('autoprefix', ['stylus'], function(cb) {
    return gulp.src(src.styles.public, {
        base: src.pubBase
    })
    .pipe($.autoprefixer({
        browsers: AUTOPREFIXER_BROWSERS
    }))
    .pipe(gulp.dest(dest.public))
    .pipe($.size({title: 'autoprefix'}));
});

// minimize CSS
gulp.task('cssmin', ['autoprefix'], function() {
    return gulp.src(src.styles.public, {
        base: src.pubBase
    })
    .pipe($.cssmin())
    .pipe(gulp.dest(dest.public))
    .pipe($.size({title: 'cssmin'}));
});

/* --------------------------------------------------------------------------
 * Task: Scripts
 * -------------------------------------------------------------------------- */

gulp.task('scripts', function(cb) {
    if (argv.prod) {
        sequence(
            'uglify',
            cb
        );
    } else {
        sequence(
            'coffee',
            cb
        );
    }
});

// compile coffee (copy vanilla js)
gulp.task('coffee', function() {
    return gulp.src(src.scripts.app, {
        base: src.appBase
    })
    .pipe($.if('*.coffee', $.coffee()))
    .pipe(gulp.dest(dest.public))
    .pipe($.size({title: 'coffee'}));
});

// uglify scripts in production
gulp.task('uglify', ['coffee'], function() {
    return gulp.src(src.scripts.public, {
        base: src.pubBase
    })
    .pipe($.uglify())
    .pipe(gulp.dest(dest.public))
    .pipe($.size({title: 'uglify'}));
});

/* --------------------------------------------------------------------------
 * Task: Images
 * -------------------------------------------------------------------------- */

gulp.task('images', function() {
    return gulp.src(src.images, {
        base: src.appBase
    })
    .pipe($.if(argv.prod, $.imagemin({
        progressive: true,
        interlaced: true
    })))
    .pipe(gulp.dest(dest.public))
    .pipe($.size({title: 'images'}));
});

/* --------------------------------------------------------------------------
 * Task: Elements
 * -------------------------------------------------------------------------- */

// vulcanize
gulp.task('elements', function() {
    return gulp.src(src.elements.all, {
        base: src.appBase
    })
    .pipe($.if('*.jade', $.jade()))
    .pipe(gulp.dest(dest.public))
    .pipe($.size({title: 'elements'}))
})
