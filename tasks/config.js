/**
 * Config: Provide src and dest files for all tasks.
 */

module.exports = {
  src: {
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
      vulcanize: [
        'public/elements/index-elements.html',
        'public/elements/component-elements.html'
      ]
    }
  },
  dest: {
    public: 'public'
  },
}
