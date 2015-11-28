#!/usr/bin/env node
/* jshint latedef: false */

'use strict';

/* --------------------------------------------------------------------------
 * Module Dependencies
 * -------------------------------------------------------------------------- */

// node modules
var http = require('http');

// NPM modules
var debug = require('debug')('frontend');

// local modules
var app = require('./app');
var ports = require('./modules/ports');

/* --------------------------------------------------------------------------
 * Helper functions
 * -------------------------------------------------------------------------- */

/**
 * Normalize a port into a number, string, or false.
 */
function normalizePort(val) {
    var port = parseInt(val, 10);
    if (isNaN(port)) {
        // named pipe
        return val;
    }
    if (port >= 0) {
        // port number
        return port;
    }
    return false;
}

/**
 * Event listener for HTTP server "error" event.
 */
function onError(error) {
    if (error.syscall !== 'listen') {
        throw error;
    }
    var bind = typeof port === 'string' ?
        'Pipe ' + port
        : 'Port ' + port;
    // handle specific listen errors with friendly messages
    switch (error.code) {
        case 'EACCES':
            console.error(bind + ' requires elevated privileges');
            process.exit(1);
            break;
        case 'EADDRINUSE':
            console.error(bind + ' is already in use');
            process.exit(1);
            break;
        default:
            throw error;
    }
}

/**
 * Event listener for HTTP server "listening" event.
 */
function onListening() {
    var addr = server.address();
    var bind = typeof addr === 'string' ?
    'pipe ' + addr
    : 'port ' + addr.port;
    debug('Listening on ' + bind);
}

/* --------------------------------------------------------------------------
 * Config
 * -------------------------------------------------------------------------- */

// Get port from environment and store in Express
var port = normalizePort(process.env.PORT || ports.express);
app.set('port', port);

/* --------------------------------------------------------------------------
 * HTTP Server
 * -------------------------------------------------------------------------- */

// Create HTTP server
var server = http.createServer(app);

// Listen on provided port, on all network interfaces.
server.listen(port, function() {
    debug('Express server listening at address ' + server.address().address +
        ' on port ' + server.address().port);
});
server.on('error', onError);
server.on('listening', onListening);

/* --------------------------------------------------------------------------
 * HTTPS Server
 * -------------------------------------------------------------------------- */

/**
 * Create HTTPS server
 * 1. var fs = require('fs'); in Node Modules
 * 2. var https = require('https'); in Node modules
 * 3. generate keys
 * 4. Uncomment this code
 */
// var credentials = {
//     key: fs.readFileSync('app/keys/agent2-key.pem'),
//     cert: fs.readFileSync('app/keys/agent2-cert.pem')
// };
// var portHttps = normalizePort(process.env.PORT_HTTPS || ports.secure);
// app.set('portHttps', portHttps);
// var serverHttps = https.createServer(credentials, app);
// serverHttps.listen(portHttps, function() {
//     debug('Express server listening at address ' +
//         serverHttps.address().address +
//         ' on port ' + serverHttps.address().port);
// });
// serverHttps.on('error', onError);
// serverHttps.on('listening', onListening);
