/**
 * @file Defines behaviours for timer component.
 */
'use strict';

var timerComponent = Polymer({
  is: "timer-component",

  // properties needed and of what type they should be
  properties: frontendInterface.buildComponentPolymerProps({
    seconds: Number,
    minutes: Number
  }),

  /**
   * Initializes a countdown with all its associated callbacks.
   * @function
   */
  attached: function () {
    // set up the end time
    var countdownMins = 20;
    var currentTime = Date.parse(new Date());
    var deadline = new Date(currentTime + countdownMins * 60 * 1000);

    // calculate the remaining time
    function timeRemain(deadline){
      var totalTime = Date.parse(deadline) - Date.parse(new Date());
      seconds = Math.floor((t / 1000) % 60 );
      minutes = Math.floor((t / 1000 / 60) % 60 );
      return {
        'minutes': minutes,
        'seconds': seconds
      };
    }

    // initialize the timer
    function initializeTimer(id, deadline){
      var timer = document.getElementById(id);
      var minutesSpan = timer.querySelector('.minutes');
      var secondsSpan = timer.querySelector('.seconds');

      // function used to avoid delay
      function updateTimer(){
        var t = timeRemain(deadline);
        minutesSpan.innerHTML = ('0' + t.minutes).slice(-2); // add leading zeros
        secondsSpan.innerHTML = ('0' + t.seconds).slice(-2);
        if(t.total<=0){
          clearInterval(timeinterval);
        }
      }
      updateTimer(); // run function once at first to avoid delay
      var timeinterval = setInterval(updateTimer,1000);
    }

    initializeTimer('timer-component', deadline);

  },

  /**
   * Polymer constructor. Copy initialization properties to the internal
   * component object.
   * @function
   * @param {Object} props - Contains initialization properties for component.
   */
  factoryImpl: function (props) {
    this.minutes = timeRemain(deadline).minutes;
    this.seconds = timeRemain(deadline).seconds;
  }
});






