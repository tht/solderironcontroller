\ Include PWM library
include pid.fs

\ Temporary fix for pwm
include tempfix.fs


\ Init pwm for output
PB6 constant PWM-OUT
120 PWM-OUT pwm-init
0 PWM-OUT pwm

\ and the MOSFET for protecting the OpAmps input
PB5 constant OPPROT-OUT
OMODE-PP OPPROT-OUT io-mode!

: opamp-prot   ( -- ) OPPROT-OUT ios! ; \ protect OpAmp input by shorting input to Gnd
: opamp-unprot ( -- ) OPPROT-OUT ioc! ; \ disable protection


\ Init adc for input
PB0 constant ADC-IN
adc-init adc-calib
IMODE-ADC ADC-IN io-mode!


\ Decide based on adc input if tip is missing
: notip? ( s_adc -- flag )
  4000 >
;


\ Measures adc input and checks for some common error conditions
\ Switches to manual mode if there is an errror
\ Result is in 1/5K (diff to environment)
: measure ( -- s_temp )
  ADC-IN adc ADC-IN adc + 2/
  dup notip? IF
    ." No tip connected - stopping pid"
    0 manual
    drop -1
  ELSE
    \ y=0.1098x+29.471 (*5 as we're working with 1/5K)
    s2f 0,549 f* 147,355 d+ f2s
  THEN
;


\ Check if the temp goes up while heating
\ Sets an error if one sec passes without temp increase
0 variable heat-start
0 variable heat-temp
0 variable heat-state

: heatermonitor-checkifok ( pwm -- pwm)
  0 heat-state !
  last-input @ heat-temp @ - 50 < IF
    \ less than 10K? There has to be an error
    ." Heatermonitor decided there is something wrong as temp is NOT rising - stopping"
    ." Temp at start:" heat-temp @ 5 / . ." now:" last-input @ 5 / .
    0 manual \ disable pid
    drop 0   \ force pwm out to 0
  ELSE
    ." heatmon is happy - stopped"
  THEN
;

: heatermonitor-on ( pwm -- pwm )
  millis heat-start @ - 1000 > IF \ at least 1sec passed
    heatermonitor-checkifok
  ELSE \ check if we're still needed here
    dup 6000 < IF \ pwm < 6000, disable myself
      0 heat-state !
      ." heatmon stopped"
    THEN
  THEN
;

: heatermonitor-off ( pwm -- pwm )
  dup 8000 > IF \ check if PWM > 8000
    1 heat-state !
    millis heat-start !
    last-input @ heat-temp !
    ." heatmon started"
  THEN
;

: heatermonitor ( pwm -- pwm )
  heat-state @ case
  0 of \ standby or not initialized
    heatermonitor-off
    endof
  1 of \ temp *should* go up
    heatermonitor-on
    endof
  endcase
;

\ Control loop

\ what happens when 
\  ms action (next-step #)
\   0 disable pwm (0)
\   2 unprotect opamp (1)
\   9 measure adc value (2)
\     protect opamp, pid calculation
\  10 enable pwm with new value (3)
\ 100 repeat
0 variable next-step
0 variable wait-time

: 1mshandler ( -- )
  wait-time @ 0<> IF
    -1 wait-time +!
  ELSE
    next-step @ case
    0 of \ == 0 -> disable pwm
      0 PWM-OUT pwm
      1 next-step ! 1 wait-time ! \ 2 ms
      endof
    1 of \ == 1 -> unprotect opamp
      opamp-unprot
      2 next-step ! 7 wait-time ! \ 8 ms
      endof
    2 of \ == 2 -> do the work (measure, protect opamp, calculate pid)
      measure opamp-prot pid heatermonitor PWM-OUT pwm
      0 next-step ! 89 wait-time ! \ 90 ms
      endof
    endcase
  THEN
; 

\ include pinchange.fs

\ add 1mshandler for pid control to systick handler
: ++ticks ( -- ) ++ticks 1mshandler ;

\ enable the new ++ticks implementation 
: enable-systick-pid ( -- )
  ['] ++ticks irq-systick !
;


\ Init PID
64,0 0,3 0,0075 100 10000 pid-init
1550 set \ about 330°C @20°C environment
enable-systick-pid
