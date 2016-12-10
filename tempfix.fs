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
