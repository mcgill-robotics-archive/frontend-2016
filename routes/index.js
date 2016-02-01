/* jshint unused: false */

var express = require('express');
var router = express.Router();

var locals = {
  title: 'Frontend',
  description: ''
};

/* GET home page. */
router.get('/', function(req, res, next) {
  locals.dev = req.app.get('env') === 'development';
  res.render('index', locals);
});

module.exports = router;
