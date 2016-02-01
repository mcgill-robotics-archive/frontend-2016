/* jshint unused: false */

/**
 * Handle any requests for a component and return the template
 * '[type]-component'; pass the component name to the template.
 */

// node modules
var path = require('path');
var fs = require('fs');

// npm modules
var express = require('express');

// vars
var router = express.Router();

/* GET home page. */
router.get('/:type/', function(req, res, next) {
  // Fetch the component type from the URL parameters
  var componentType = req.params.type + '-component';

  var elementTestPath = path.join(
    __dirname,
    '../',
    'public',
    'elements',
    'components',
    componentType,
    'test',
    'index.html'
  );

  fs.readFile(elementTestPath, 'utf8', function(err, data) {
    if (err) {
      return next(err);
    }
    var locals = {
      title: 'Frontend',
      description: '',
      component: componentType,
      testHTML: data
    };
    locals.dev = req.app.get('env') === 'development';
    res.render('component', locals);
  });
});

module.exports = router;
