\ Include PWM library
include pid.fs

\ Temporary fix for pwm and adc
include tempfix.fs


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


\ Decide based on adc input if tip is missing
: notip? ( s_adc -- flag )
  4050 >
;


\ Measures adc input and checks for some common error conditions
\ Switches to manual mode if there is an errror
: measure ( -- s_temp )
  ADC-IN adc ADC-IN adc + 2/
  dup notip? IF
    ." No tip connected - stopping pid"
    0 manual
    drop -1
  ELSE
    90 - 0 max 11 / 22 +
  THEN
;


\ Check if the temp goes up while heating
\ Sets an error if one sec passes without temp increase
0 variable heat_start
0 variable heat_temp
0 variable heat_state

: heatermonitor_on ( pwm -- pwm )
  millis heat_start @ - 1000 > IF \ at least 1sec passed
    0 heat_state !
    last-input @ heat_temp @ - 10 < IF
      \ less than 10°? There has to be an error
      ." Heatermonitor decided there is something wrong as temp is NOT rising - stopping"
      ." Temp at start:" heat_temp @ . ." now:" last-input @ .
      0 manual \ disable pid
      drop 0   \ force pwm out to 0
    ELSE
      ." heatmon is happy - stopped"
    THEN
  ELSE \ check if we're still needed here
    dup 6000 < IF \ pwm < 6000, disable myself
      0 heat_state !
      ." heatmon stopped"
    THEN
  THEN
;

: heatermonitor_off ( pwm -- pwm )
  dup 8000 > IF \ check if PWM > 8000
    1 heat_state !
    millis heat_start !
    last-input @ heat_temp !
    ." heatmon started"
  THEN
;

: heatermonitor ( pwm -- pwm )
  heat_state @ case
  0 of \ standby or not initialized
    heatermonitor_off
    endof
  1 of \ temp *should* go up
    heatermonitor_on
    endof
  endcase
;

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
      measure opamp-prot pid heatermonitor PWM-OUT pwm
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
150 set
enable_systick_pid
