\ timers.fs
\ Multiple callback timers using one single background task
\ Needs multi.fs

\ Temporary - for easier debugging
<<<board>>>
compiletoflash
include /Users/thomas/go/src/github.com/jeelabs/embello/explore/1608-forth/flib/mecrisp/multi.fs
\ End Temporary

compiletoflash

\ --------------------------------------------------
\  Configuration
\ --------------------------------------------------

8 constant num-timers \ maximum number of timers (using 4 cells each)

\ --------------------------------------------------
\  Internal Helpers
\ --------------------------------------------------

\ 4 cells per timer ( interval, last-run, callback, repeat )
num-timers 4 * cells buffer: timer-data

\ Calculate internal adresses
: tmr-inte-addr ( timer# - addr ) timer-data swap 4 *     cells + ;
: tmr-last-addr ( timer# - addr ) timer-data swap 4 *  1+ cells + ;
: tmr-call-addr ( timer# - addr ) timer-data swap 4 *  2+ cells + ;
: tmr-repe-addr ( timer# - addr ) timer-data swap 4 * 3 + cells + ;

\ Used by call-once/call-periodic
: call-internal ( callback when/interval repeat timer# )
  >r     r@ tmr-repe-addr !
         r@ tmr-inte-addr !
         r@ tmr-call-addr !
  millis r> tmr-last-addr !
;

\ Execute timer callback and clear if no repetition is required
: timer-exec ( timer# )
  >r     r@ tmr-call-addr @ execute
  millis r@ tmr-last-addr !
  r@ tmr-repe-addr @ NOT IF
    \ clear callback if no repetition needed
    false r@ tmr-call-addr !
  THEN 
  r> drop 
;

\ Check if a timer needs to be executed (checks if enabled and enough time passed)
: needs-run? ( timer# )
  dup tmr-call-addr @ IF 
    millis over tmr-last-addr @ -  ( timer# time_since_last_run )
           over tmr-inte-addr @ >  ( timer# true_if_time_to_run )
  ELSE
    false
  THEN nip ;

\ Check and execute all the timer
: timer-run ( -- #exec )
  0 \ return number of executed tasks
  num-timers 0 DO
  i needs-run? IF
    i timer-exec 1+
  THEN
LOOP ;

\ Go to sleep if we're the only task running
: sleep-if-alone ( -- )
  eint? IF \ Only enter sleep mode if interrupts have been enabled
    dint up-alone? IF sleep THEN eint
  THEN ;

\ Task which handles the timers in background
task: timertask
: timer& ( -- )
  timertask activate
    begin
      timer-run NOT IF sleep-if-alone THEN
      pause
    again
;

\ --------------------------------------------------
\  External API
\ --------------------------------------------------

\ Clear timer data structure
: clear-timers ( -- ) timer-data num-timers 4 * cells 0 fill ;

\ Register a callback or cancel a timer
: call-once     ( callback when     timer# ) false swap call-internal ; 
: call-periodic ( callback interval timer# ) true  swap call-internal ; 
: cancel-timer  ( timer# ) tmr-call-addr 0 swap !  ;

\ Show all timer data
: timers ( -- ) CR
  num-timers 0 do
    ." Timer #" i .
    ." Interval: " i tmr-inte-addr @ .
    ." Last-Run: " i tmr-last-addr @ .
    ." Callback: " i tmr-call-addr @ .
    ." Repeat: "   i tmr-repe-addr @ .
  CR loop ;

\ Initializes timer-data and starts multitasking
: timer-init ( -- )
  clear-timers
  timer& multitask
;

\ for testing only
: ping ( -- ) CR ." PING" CR ;

compiletoram

