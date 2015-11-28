/* jshint unused: false */

var express = require('express');
var router = express.Router();

var locals = {
    dev: req.app.get('env') === 'development',
    title: 'Frontend',
    titleColor: '#f2f7fa',
    description: ''
};

/* GET home page. */
router.get('/', function(req, res, next) {
    res.render('index', locals);
});

module.exports = router;
