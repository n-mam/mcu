
Raspberry Pi Pico PWM notes

- There are 8 independent PWM engines, referred to as slices.
- Each slice has an A and B channel and each channel can output to several GPIOs.
- Each slice may have a different frequency. All GPIOs on a slice have the same frequency.
- The A and B channels may have different dutycycles.
- All GPIO on a slice/channel combination have the same dutycycle.

```
 - pico's default system clock runs at a frequency of 125MHz, i.e.
 - 125000000 cycles in 1 sec, implying
 - The duration of a single cycle of system clock is :
 - 1 / 125000000 = 0.000000008 = 8 nano seconds
 - we will refer to the duration of a single clock pulse as 1/Fc instead of 8ns
```

**Pico's PWM has 3 aspects:**

A PWM counter
- starts from 0
- Base on the either the default or the divided 125MHz sys clock
- Free running counter which counts from 0 to TOP's value (16 bit) and then wraps back again.

A TOP value (wrap)
- starts from 0
- pwm_set_wrap(), TOP register (16 bit)
- Signifies how many system clock cycles map to one (larger) pwm cycle
- For example we can set TOP(wrap) as 3. This would use 4 (0-3) system clock pulses as one PWM cycle.

So the "PWM counter" will count from 0 to 3(TOP) and then wrap back to 0 again.

A counter compare value (level)
- starts from 1
- set using pwm_set_chan_level(), CC register
- counter compare value determines the number of sys clock pulses (out of the total that make up one PWM pulse aka wrap) for which the output will remain high.
- In the abobe example we could set the level as 2 sys clock pulses which then works by comparing the current PWM counter value with the set PWM counter compare value

```
if (counter compare level for PWM) > (the current PWM counter value):
- Then the output is high
- Else it is low

|----|----|----|----|----|--------   sys clock pulses ----------------|----| 1 sec
|1100|1100|1100|1100|1100|............. PWM pulses ...................|1100| 1 sec

Note: PWM counter and TOP are both 16 bit wide:
Note: counter wrap (TOP) value sets the PWM  frequency and the level sets the PWM duty cycle.
```

**Calculating wrap from a given desired PWM frequency (f)**

```
           125,000,000 number of sys clock pulses in 1 sec
|-------------------------------------------------------------------| 1sec

             f number of PWM pulses
             each of (1/Fc * wrap) duration
             all in, again 1 sec
|--------|--------|--------|   --------  |--------|--------|--------| 1 sec
```

implying : (also since wrap actually start from 0, we use "wrap + 1"):

f * ((wrap + 1) * 1/Fc) = 1 sec

(f * (wrap + 1)) / Fc = 1

wrap + 1 = Fc / f
```
wrap = (Fc / f) - 1   ------ (1)
```
However, the wrap register (TOP i.e.) is only 16 bit wide. So we can not
use the above formula to compute the wrap needed for any arbitrary
desired PWM frequency.

For example, a 50 Hz desired PWM prequency would mean (as per the above formual)
a wrap value of (125000000 / 50) - 1 = 2,499,999. The 16 bit TOP reg is not wide
enough to store this value.

So with the highest wrap value of 65535 (2pow16 - 1) in the 16 bit wide TOP register, we have as per (1):

f = Fc / (wrap + 1)

  = 125000000 / 65536 = 1907.348 == 1.9 KHz

the lowest PWM frequence we can get (if nothing else is available) would be 1.9 KHz.
Note:
```
higher the wrap value lower the PWM cycle frequency
lower the wrap value higher the PWM cycle frequency
```
- lowest wrap(TOP) value = 0  i.e. one PWM cycle == one sys clock cycle
- counter compare (level) can only be 1 100% duty cycle
- PWM frequency same as sys clock frequency

**How to configure PWM frequencies lower than 1.9 KHz ... ?**

say we need 50Hz PWM at max wrap (lowest frequency)

The divider needed for a max wrap of (2 pow 16 - 1) given the desired PWM frequency is 50Hz, as per (1) would be

65534 = ((Fc/d ) / f) - 1

65535 * d * f = Fc

d = Fc / (f * 65535) for the desired PWM of 50Hz

d = 125000000 / 3276750 = 38.1475 need to take the ceil of this to avoid overflowing 16 bits

so this is the divider that would give us a new refrence clock frequency which when applied to our original formula would keep the wrap value under 16 bits for the desired 50 Hz frequency keeping the wrap maxed out