/**
 * @file Defines behaviours for timer component.
 */

/*global Polymer, frontendInterface */

'use strict';

var timerComponent = Polymer({
  is: "timer-component",

  /*
   * Build an object with all Polymer properties for the component. Specify 
   * Polymer properties unique to the component defined, and set their
   * JavaScript types.
   */

  // properties needed and of what type they should be
  properties: frontendInterface.buildComponentPolymerProps({
    seconds: String,
    minutes: String
  }),

  /* 
	RUNNING THE CLOCK	
	Ask the server, what is the state and time on the timer?
        Depending on this: We set the time.
 	From here, we use setInterval to basically time our changes to the clock:
		so if we set the interval to one second, then every second, we run a method that updates the clock

	RESETTING THE CLOCK
	Tell the server to reset the clock, and respond when it's done resetting. 
	Once we hear back from the server, we will be given time, which we can simply use the same way we do when we run. 
		
 */
  attached : function () {
    var that = this;
    var g_startTime = new Date(); // This will eventually be requested from the server.
    
    function setupClientTimer () {
      // setInterval(updateTimer, 1000) call the update timer method every second
      setInterval(updateTimer(), 1000); 
    } 

    function updateTimer () {
      // Current time, new Date() : 
      var currentTime = Date.parse(new Date());
      // The time difference -> start time - current time
      var timeDiff = g_startTime - currentTime;
      // The time left, is 20 minutes minus the time difference
      var timeLeft = new Date(20 * 60 * 1000 - timeDiff);
      // 1 set the minutes and seconds based on the time. 
      seconds = Math.floor((timeLeft / 1000) % 60);
      that.seconds = seconds.toString();
      minutes = Math.floor((timeLeft / 1000 / 60) % 60);
      that.minutes = minutes.toString();
    }


    

/*
    // set up the end time
    var countdownMins = 20;
    var currentTime = Date.parse(new Date());
    var deadline = new Date(currentTime + countdownMins * 60 * 1000);

    // calculate the remaining time
    function timeRemain(deadline) {
      var totalTime = Date.parse(deadline) - Date.parse(new Date());
      seconds = Math.floor((totalTime / 1000) % 60);
      minutes = Math.floor((totalTime / 1000 / 60) % 60);
      return {
        'minutes': minutes,
        'seconds': seconds
      };
    }// end of timeRemain


    // initialize the timer
    function initializeTimer(id, deadline) {
      var timer = document.getElementById(id);
      var minutesSpan = timer.querySelector('.minutes'); 
      // avoid rebuilding the timer continuouslt
      var secondsSpan = timer.querySelector('.seconds');
      // function used to avoid delay

      function updateTimer() {
        var t = timeRemain(deadline);
        // add leading zeros
        minutesSpan.innerHTML = ('0' + t.minutes).slice(-2); 
        secondsSpan.innerHTML = ('0' + t.seconds).slice(-2);
        if (t.total <= 0) {
	  clearInterval(timeinterval);
        }
      }// end of updateTimer

      updateTimer(); // run function once at first to avoid delay
      var timeinterval = setInterval(updateTimer(), 1000);  
    }// end of initializeTimer

    initializeTimer('timer-component', deadline);
  },

*/

  /**
   * Polymer constructor. Copy initialization properties to the internal
   * component object.
   * @function
   * @param {Object} props - Contains initialization properties for component.
   */
  // Fix based on the changes you make to properties
  factoryImpl: function (props) {
    this.minutes = timeRemain(deadline).minutes;
    this.seconds = timeRemain(deadline).seconds;
  },
}); // end of polymer
