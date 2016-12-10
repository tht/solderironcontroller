\ PID controller written in Forth
\ Based on the code presented here:
\ http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/


\ Setup variables for pid control
0,0 2variable kp           \ absolute value
0,0 2variable ki           \ scaled to sampling interval
0,0 2variable kd           \ scaled to sampling interval
0    variable interval     \ sampling interval (in ms)
0    variable out-limit    \ output limit (0 to `out-limit)
0    variable out-override \ output override (auto mode if -1)

\ Working variables while pid is running
0    variable set-val      \ current setpoint
0,0 2variable total-i      \ cummulative i error
0    variable last-input   \ last seen input


\ =============================================================================
\ Utility words

\ Fixed point to signed number (rounded)
: f2s ( f -- s ) swap 1 31 lshift and IF 1 + THEN ;

\ Signed number to fixed point
: s2f ( s -- f ) 0 swap ;

\ Minimum of double number (works also on fixed-point)
: dmin ( d1 d2 -- d_min )
  2over 2over ( d1 d2 d1 d2 )
  d< IF 2drop ELSE 2nip THEN
;

\ Maximum of double number (works also on fixed-point)
: dmax ( d1 d2 -- d_max )
  2over 2over ( d1 d2 d1 d2 )
  d> IF 2drop ELSE 2nip THEN
;

\ Make sure a double (fixed-point) number is in range
: drange ( d_val d_min d_max -- d_val ) 
  2rot ( d_min d_max d_val) dmin dmax
;

\ Make sure a number is in range
: range ( s_val s_min s_max -- s_val ) 
  rot ( s_min s_max s_val) min max
;

\ Output fixed point value
: f.000 3 f.n ;


\ =============================================================================
\ Main PID - internal definitions (do not call manually)

\ Calculate proportial error
: calc-p ( f_error -- f_correction )
  kp 2@ f*                 \ fetch k-value and scale error
  ." Pval:" 2dup f2s .
;


\ Calculate integral error
: calc-i ( f_error -- f_correction )
  ki 2@ f*                 \ apply ki factor
  total-i 2@ d+            \ sum up with running integral error
  0,0 out-limit @ s2f drange \ cap inside output range
  2dup total-i 2!          \ update running integral error
  ." Ival:" 2dup f2s .
;


\ Calculate differential error - actually use "derivative on input", not on error
: calc-d ( s_is -- f_correction )
  last-input @ -           \ substract last input from current input
  s2f kd 2@ f*             \ make fixed point, fetch k-value and multiply
  ." Dval:" 2dup f2s .
;


\ Do a PID calculation, returns correction value (aka duty-cycle)
: pid_compute ( s_is -- s_corr )
  CR ." SET:" set-val @ .  ." IS:"  dup . \ DEBUG

  \ feed error in p and i, current setpoint in d, sum up results
  dup dup set-val @ swap - s2f ( s_is s_is f_error )
  2dup  calc-p             ( s_is s_is f_error f_p )
  2swap calc-i d+          ( s_is s_is f_pi )
  rot   calc-d d-          ( s_is f_pid ) \ substract! derivate on input - not error

  f2s                      ( s_is s_corr )
  ." OUT:" dup .           \ DEBUG

  swap last-input !        \ Update variables for next run
  0 out-limit @ range      \ Make sure we return something inside range

  ." PWM:" dup .
;


\ =============================================================================
\ Main PID - external interface

\ Change setpoint on a running pid
: set ( s -- )
  set-val !
;


\ Change tuning-parameters on a running pid
: tuning  ( f_kp f_ki f_kd -- )
  \ depends on sampletime, so fetch it, move to fixed-point and change unit to seconds
  \ store on return stack for now
  interval @ s2f 1000,0 f/ 2>r

  2r@ f/ kd 2!             \ translate from 1/s to the sampletime
  2r> f* ki 2!             \ translate from 1/s to the sampletime
         kp 2!
;


\ Init PID
\ To use in a *reverse acting system* (bigger output value **reduced**
\ input value make sure `kp`, `ki` and `kd` are **all** negative.
\ Starts pid in manual mode (no setpoint set!). Set setpoint and call auto
\ to start the control loop.
: pid-init ( f_kp f_ki f_kd s_sampletime s_outlimit -- )
  out-limit !
  interval !
  tuning
  0 out-override !         \ Make sure we're in manual mode
  CR ." PID initialized - kp:" kp 2@ f.000 ." ki:" ki 2@ f.000 ." kd:" kd 2@ f.000
;


\ Returns calculated PID value or override value if in manual mode
: pid ( s_is -- s_corr )
  out-override @ -1 = IF   \ we're in auto-mode - do PID calculation
    pid_compute
  ELSE                     \ manual-mode! store input, return override value
    CR ." SET:" set-val @ .  ." IS:"  dup .
    last-input !
    out-override @
    ." PWM:" dup .
  THEN
;


\ Override output - switches PID into *manual mode*
: manual ( s -- )
  out-override !
;


\ Switches back to auto-mode after manual mode.
: auto ( -- )
  out-override @ <> -1 IF \ only do something if we'r in override mode
    \ store current output value as i to let it run smoothly
    out-override @
    0 out-limit @ range    \ Make sure we store something within PWM bounds
    s2f total-i 2!
    -1 out-override !
  THEN
;


\ Brings PID back to auto-mode after a manual override
\ set's last input reading as setpoint
: autohold ( -- )
  last-input @ set-val !   \ Use last input as setpoint (no bumps!)
  auto
;

