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

\ Calculates the difference to setpoint
: diff2set ( s_is -- s_diff ) set-val @ swap - ;


\ Change tuning-parameters on a running pid
: tuning  ( f_kp f_ki f_kd -- )
  \ depends on sampletime, so fetch it, move to fixed-point and change unit so seconds
  \ store on return stack for now
  interval @ s2f 1000,0 f/ 2>r

  2r@ f/ kd 2!             \ translate from 1/s to the sampletime
  2r> f* ki 2!             \ translate from 1/s to the sampletime
  kp 2!
;

\ Change setpoint on a running pid
: setpoint ( s -- )
  set-val !
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
  \ CR ." PID initialized - kp:" kp 2@ f.000 ." ki:" ki 2@ f.000 ." kd:" kd 2@ f.000
;


\ Calculate proportial error
\ DO NOT CALL DIRECTLY... see `pid` below
: calc-p ( s_is -- f_correction )
  diff2set s2f             \ linear error as fixed point value
  kp 2@ f*                 \ fetch k-value and scale error
  ." Pval:" 2dup f.000
;

\ Calculate integral error
\ DO NOT CALL DIRECTLY... see `pid` below
: calc-i ( s_is -- f_correction )
  diff2set s2f             \ linear error as fixed point
  ki 2@ f*                 \ apply ki factor
  total-i 2@ d+            \ sum up with running integral error
  0,0 out-limit @ s2f drange \ cap inside output range
  2dup total-i 2!          \ update running integral error
  ." Ival:" 2dup f.000
;

\ Calculate differential error - actually use "derivative on input", not on error
\ DO NOT CALL DIRECTLY... see `pid` below
: calc-d ( s_is -- f_correction )
  last-input @ -           \ substract last input from current input
  s2f kd 2@ f*             \ make fixed point, fetch k-value and multiply
  ." Dval:" 2dup f.000
;

\ Do a PID calculation, returns correction value (aka duty-cycle)
\ DO NOT CALL DIRECTLY... see `pid` below
: pid_compute ( s_is -- s_corr )
  dup >r \ store current value on return stack
         \ current value still on stack for later use

  CR ." SET:" set-val @ .  ." IS:"  dup . \ DEBUG

     calc-p
  r@ calc-i d+
  r@ calc-d d-             \ substract here as we're using derivative on input, not on error

  2dup ." OUT:" 4 f.n      \ DEBUG

  r> last-input !          \ Update variables for next run

  nip
  0 out-limit @ range      \ Make sure we return something inside range

  ." PWM:" dup .
;


\ Returns calculated PID value or override value if in manual mode
: pid ( s_is -- s_corr )
  out-override @ -1 = IF
    \ we're in auto-mode - do PID calculation
    pid_compute
  ELSE
    \ manual-mode! store input, return override value
    dup CR ." IS:" .
    last-input !
    out-override @
    dup ." PWM:" .
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

