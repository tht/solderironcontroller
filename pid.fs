\ Define PID weights as fixed point
0,0  2variable kp \ absolute value
0,0  2variable ki \ scaled to sampling interval
0,0  2variable kd \ scaled to sampling interval
1000  variable interval \ sampling interval
10000 variable out_limit \ output limit (0 to `out_limit`)
0     variable out_override \ output override (manual mode unless -1)

\ Working variables while pid is running
0    variable set_val    \ current setpoint
0,0 2variable total_i    \ cummulative i error
0    variable last_input \ last seen error

\ Fixed point to signed number
: f2s ( f -- s ) swap 1 31 lshift and IF 1 + THEN ;

\ Signed number to fixed point
: s2f ( s -- f ) 0 swap ;

: f.000 3 f.n ;

\ Init PID - use on *running* PID to change the tuning parameters
\ To use in a *reverse acting system* (bigger output value **reduced**
\ input value make sure `kp`, `ki` and `kd` are **all** negative.
: +pid ( f_kp f_ki f_kd s_set s_sampletime -- )
  dup s2f 1000,0 f/ 2>r \ store sampletime (in s) on return stack
  interval !
  set_val !
  2r@ f/ kd 2! \ translate from 1/s to the sampletime
  2r> f* ki 2! \ translate from 1/s to the sampletime
  kp 2!
  -1 out_override !
  CR ." kp:" kp 2@ f.000 ." ki:" ki 2@ f.000 ." kd:" kd 2@ f.000
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
  total_i 2@ d+ 2dup  \ sum up with running integral
  out_limit @ s2f d> IF \ check if we did run too high
    2drop out_limit @ s2f \ cap at out_limit
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
    last_input !
    out_override @
  THEN
;

\ Override output - switches PID into *manual mode*
: pid_override ( s -- )
  out_override !
;

\ Brings PID back to auto-mode after a manual override
\ set's last input reading as setpoint
: pid_auto ( -- )
  out_override @ <> -1 IF \ only do something if we'r in override mode
    \ store current output value as i to let it run smoothly
    out_override @
    out_limit @ min 0 max \ Make sure we store something within PWM bounds
    s2f total_i 2!
    last_input @ set_val ! \ Use last input as setpoint (no bumps!)
    -1 out_override !
  THEN
;


\ DEBUG - sample code
\ 120,0 2,2 0,01 350 100 +pid
\ 100 pid .
\ 200 pid .
\ 300 pid .
\ 400 pid .
\ 400 pid .
\ \ 1000 pid_override
\ 350 pid .
\ 350 pid .
\ \ pid_auto
\ 350 pid .



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

: onestep ( -- )
  0 PWM-OUT pwm     \ disable pwm to get a good measurement
  2 ms opamp-unprot \ wait 2ms before releasing OpAmp input protection
  7 ms measure      \ wait 7ms before measuring to let the OpAmp settle down
  opamp-prot 1 ms   \ start protection again, wait 1ms before ...
  pid PWM-OUT pwm   \ calculating new duty-type and enable pwm on output
  90 ms             \ let pwm run
;

\ Init PID
320,0 1,5 0,01 350 100 +pid

\ Define two loops for some seconds of runtime
: 10sec 100 0 DO onestep LOOP 0 PWM-OUT pwm ;
: 100sec 1000 0 DO onestep LOOP 0 PWM-OUT pwm ;
