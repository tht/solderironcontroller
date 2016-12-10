\ Define PID weights as fixed point
0,0 2variable kp \ absolute value
0,0 2variable ki \ scaled to sampling interval
0,0 2variable kd \ scaled to sampling interval
0    variable interval \ sampling interval (in ms)
0    variable out_limit \ output limit (0 to `out_limit`)
0    variable out_override \ output override (manual mode unless -1)

\ Working variables while pid is running
0    variable set_val    \ current setpoint
0,0 2variable total_i    \ cummulative i error
0    variable last_input \ last seen error

\ Fixed point to signed number
: f2s ( f -- s ) swap 1 31 lshift and IF 1 + THEN ;

\ Signed number to fixed point
: s2f ( s -- f ) 0 swap ;

: f.000 3 f.n ;


\ Change tuning-parameters on a running pid
: tuning  ( f_kp f_pi f_kd -- )
  \ depends on sampletime, so fetch it, move to fixed-point and change unit so seconds
  \ store on return stack for now
  interval @ s2f 1000,0 f/ 2>r

  2r@ f/ kd 2! \ translate from 1/s to the sampletime
  2r> f* ki 2! \ translate from 1/s to the sampletime
  kp 2!
;

\ Change setpoint on a running pid
: setpoint ( s -- )
  set_val !
;


\ Init PID
\ To use in a *reverse acting system* (bigger output value **reduced**
\ input value make sure `kp`, `ki` and `kd` are **all** negative.
\ Starts pid in manual mode (no setpoint set!). Set setpoint and call auto
\ to start the control loop.
: pid-init ( f_kp f_ki f_kd s_sampletime s_outlimit -- )
  out_limit !
  interval !
  tuning
  0 out_override !
  \ CR ." PID initialized - kp:" kp 2@ f.000 ." ki:" ki 2@ f.000 ." kd:" kd 2@ f.000
;

: diff2set ( s_is -- s_diff ) set_val @ swap - ;

\ Calculate proportial error
\ DO NOT CALL DIRECTLY... see `pid` below
: calc_p ( s_is -- f_correction )
  diff2set s2f \ linear error as fixed point value
  kp 2@ f* \ fetch k-value and scale error
  ." Pval:" 2dup f.000
;

\ Calculate integral error
\ DO NOT CALL DIRECTLY... see `pid` below
: calc_i ( s_is -- f_correction )
  diff2set s2f        \ linear error as fixed point
  ki 2@ f*            \ apply ki factor
  total_i 2@ d+       \ sum up with running integral
  2dup out_limit @ s2f d> IF \ check if we did run too high
    2drop out_limit @ s2f \ cap at out_limit
  THEN
  2dup 0,0 d< IF         \ check if we did run below output limit
    2drop 0,0
  THEN
  2dup total_i 2!     \ update running integral error
  ." Ival:" 2dup f.000
;

\ Calculate differential error - actually use "derivative on input", not on error
\ DO NOT CALL DIRECTLY... see `pid` below
: calc_d ( s_is -- f_correction )
  last_input @ -      \ substract last input from current input
  s2f kd 2@ f*        \ make fixed point, fetch k-value and multiply
  ." Dval:" 2dup f.000
;

\ Do a PID calculation, returns correction value (aka duty-cycle)
\ DO NOT CALL DIRECTLY... see `pid` below
: pid_compute ( s_is -- s_corr )
  dup >r \ store current value on return stack
         \ current value still on stack for later use

  \ DEBUG
  CR ." SET:" set_val @ .  ." IS:"  dup .

     calc_p
  r@ calc_i d+
  r@ calc_d d- \ substract here as we're using derivative on input, not on error

  \ DEBUG
  2dup ." OUT:" 4 f.n

  \ Update variables for next run
  r> last_input !

  nip
  out_limit @ min 0 max \ Make sure we return something inside PWM bounds

  ." PWM:" dup .
;

\ Returns calculated PID value or override value if in manual mode
: pid ( s_is -- s_corr )
  out_override @ -1 = IF
    \ we're in auto-mode - do PID calculation
    pid_compute
  ELSE
    \ manual-mode! store input, return override value
    dup CR ." IS:" .
    last_input !
    out_override @
    dup ." PWM:" .
  THEN
;

\ Override output - switches PID into *manual mode*
: manual ( s -- )
  out_override !
;

: auto ( -- )
  out_override @ <> -1 IF \ only do something if we'r in override mode
    \ store current output value as i to let it run smoothly
    out_override @
    out_limit @ min 0 max \ Make sure we store something within PWM bounds
    s2f total_i 2!
    -1 out_override !
  THEN
;

\ Brings PID back to auto-mode after a manual override
\ set's last input reading as setpoint
: autohold ( -- )
  last_input @ set_val ! \ Use last input as setpoint (no bumps!)
  auto
;


\ Temporary fix for pwm and adc

\ Make sure pwm is completly off at 0
: pwm ( u pin -- )
  over 0 = IF
    -pwm  \ disable pwm
    drop  \ drop output value
  ELSE
    \ enable pwm output on pin again
    dup dup p2cmp 4 * bit swap p2tim timer-base $20 + bis!
    pwm   \ set pwm output (duty cycle)
  THEN
;

\ always drop first value as this is still an old result
: adc dup adc drop adc ;




\ Set up for solder iron controller



\ Init pwm for output
PB6 constant PWM-OUT
120 PWM-OUT +pwm
0 PWM-OUT pwm

\ and the MOSFET for protecting the OpAmps input
PB5 constant OPPROT-OUT
OMODE-PP OPPROT-OUT io-mode!

: opamp-prot   ( -- ) OPPROT-OUT ios! ; \ protect OpAmp input by shorting input to Gnd
: opamp-unprot ( -- ) OPPROT-OUT ioc! ; \ disable protection


\ Init adc for input
PB0 constant ADC-IN
+adc
adc-calib
IMODE-ADC ADC-IN io-mode!

\ Some measured adc values and corresponding temp (not exact!)
\  100   0 K  ->  20°C (-20) 
\ 1400 130 K  -> 150°C (-20)
\ 3670 330 K  -> 350°C (-20)

: measure ( -- s_temp ) ADC-IN adc ADC-IN adc + 2/ 90 - 0 max 11 / 22 + ;


\ Control loop

\ what happens when 
\  ms action (next_step #)
\   0 disable pwm (0)
\   2 unprotect opamp (1)
\   9 measure adc value (2)
\     protect opamp, pid calculation
\  10 enable pwm with new value (3)
\ 100 repeat
0 variable next_step
0 variable wait_time

: 1mshandler ( -- )
  wait_time @ 0<> IF
    -1 wait_time +!
  ELSE
    next_step @ case
    0 of \ == 0 -> disable pwm
      0 PWM-OUT pwm
      1 next_step ! 1 wait_time ! \ 2 ms
      endof
    1 of \ == 1 -> unprotect opamp
      opamp-unprot
      2 next_step ! 7 wait_time ! \ 8 ms
      endof
    2 of \ == 2 -> do the work (measure, protect opamp, calculate pid)
      measure opamp-prot pid PWM-OUT pwm
      0 next_step ! 89 wait_time ! \ 90 ms
      endof
    endcase
  THEN
; 

\ add 1mshandler for pid control to systick handler
: ++ticks ( -- ) ++ticks 1mshandler ;

\ enable the new ++ticks implementation 
: enable_systick_pid ( -- )
  ['] ++ticks irq-systick !
;


\ Init PID
320,0 1,5 0,0075 100 10000 pid-init
150 setpoint

