# raw

This application demonstrates how to retrieve accel and gyro data. ODR is configured at 50 Hz.

## Command interface

This application allows the following command to be sent through UART:
* `s`: to toggle print data in SI unit (enabled/disabled, enabled by default).
* `l`: to toggle print data in LSB (enabled/disabled, disabled by default).
* `f`: to toggle FIFO usage (enabled/disabled, enabled by default). Data will be read from sensor registers if FIFO is disabled.
* `i`: to toggle FIFO highres mode usage (enabled/disabled, enabled by default). Only apply if FIFO is enabled.
* `p`: to toggle selected power-mode (low-noise/low-power, low-noise by default).
* `c`: to print current configuration.
* `h`: to print helps screen.

## Terminal output

### Data format

Data are printed on the terminal as follow:

* When print in SI unit is enabled:
```
SI <timestamp> us Accel: <acc_x> <acc_y> <acc_z> g Gyro: <gyr_x> <gyr_y> <gyr_z> dps Temp: <temp> degC FIFO time: <fifo_time> us
```
* When print in LSB is enabled:
```
LSB <timestamp> us Accel: <raw_acc_x> <raw_acc_y> <raw_acc_z> Gyro: <raw_gyr_x> <raw_gyr_y> <raw_gyr_z> Temp: <raw_temp> FIFO time: <fifo_time> us
```

With:
* `timestamp`: Time in microsecond read from MCU clock when latest INT1 was fired
* `raw_acc_x|y|z`: Raw accel value
* `acc_x|y|z`: Accel value converted in g
* `raw_gyr_x|y|z`: Raw gyro value
* `gyr_x|y|z`: Gyro value converted in dps
* `raw_temp`: Raw temperature value
* `temp`: Temperature value converted in °C
* `fifo_time`: 16-bit timestamp field from FIFO.

### Example of output

```
[I] ###
[I] ### Example Raw
[I] ###
[I] SI        9363 us   Accel:       -        -        -     Gyro:       -        -        -       Temp:  21.63 degC   FIFO Time:  3232 us
[I] SI       29425 us   Accel:    0.07    -1.00     0.03 g   Gyro:       -        -        -       Temp:  21.59 degC   FIFO Time: 23232 us
[I] SI       49423 us   Accel:    0.07    -1.00     0.03 g   Gyro:       -        -        -       Temp:  21.72 degC   FIFO Time: 43232 us
[I] SI       69420 us   Accel:    0.07    -1.00     0.03 g   Gyro:       -        -        -       Temp:  21.74 degC   FIFO Time: 63232 us
[I] SI       89417 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.32    -0.06    -0.03 dps   Temp:  21.74 degC   FIFO Time: 83232 us
[I] SI      109414 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.34    -0.09     0.02 dps   Temp:  21.72 degC   FIFO Time: 103232 us
[I] SI      129412 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.31    -0.05     0.02 dps   Temp:  21.74 degC   FIFO Time: 123232 us
[I] SI      149409 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.34    -0.05    -0.02 dps   Temp:  21.76 degC   FIFO Time: 143232 us
[I] SI      169406 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.34    -0.05    -0.05 dps   Temp:  21.73 degC   FIFO Time: 163232 us
[I] SI      189403 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.34    -0.02     0.03 dps   Temp:  21.74 degC   FIFO Time: 183232 us
[I] SI      209401 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.31    -0.05     0.02 dps   Temp:  21.75 degC   FIFO Time: 203232 us
[I] SI      229398 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.34    -0.05     0.05 dps   Temp:  21.74 degC   FIFO Time: 223232 us
[I] SI      249395 us   Accel:    0.07    -1.00     0.02 g   Gyro:   -0.35    -0.05     0.05 dps   Temp:  21.72 degC   FIFO Time: 243232 us
[I] SI      269392 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.35    -0.05     0.09 dps   Temp:  21.73 degC   FIFO Time: 263232 us
[I] SI      289390 us   Accel:    0.07    -1.00     0.02 g   Gyro:   -0.31    -0.03     0.05 dps   Temp:  21.71 degC   FIFO Time: 283232 us
[I] SI      309387 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.34    -0.02     0.05 dps   Temp:  21.76 degC   FIFO Time: 303232 us
[I] SI      329384 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.34    -0.06     0.02 dps   Temp:  21.76 degC   FIFO Time: 323232 us
[I] SI      349381 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.32    -0.06     0.02 dps   Temp:  21.73 degC   FIFO Time: 343232 us
[I] SI      369379 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.37    -0.09     0.02 dps   Temp:  21.73 degC   FIFO Time: 363232 us
[I] SI      389376 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.35    -0.03     0.02 dps   Temp:  21.78 degC   FIFO Time: 383232 us
[I] SI      409373 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.35     0.00     0.05 dps   Temp:  21.75 degC   FIFO Time: 403232 us
[I] SI      429370 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.35     0.00     0.00 dps   Temp:  21.74 degC   FIFO Time: 423232 us
[I] SI      449367 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.32    -0.03     0.06 dps   Temp:  21.78 degC   FIFO Time: 443232 us
[I] SI      469365 us   Accel:    0.07    -1.00     0.03 g   Gyro:   -0.35    -0.03     0.06 dps   Temp:  21.75 degC   FIFO Time: 463232 us
```

