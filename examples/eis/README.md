# eis

This application demonstrates how to enable FSYNC feature required for EIS application. The FSYNC feature consists of an external signal provided to the IMU as input, which is used to tag sensor data closest to the rising edge of the signal, as well as the delay between the rising edge and the sensor data.

For this application, the FSYNC signal will be emulated by the MCU with a 30 Hz square signal.

## Command interface

This application allows the following command to be sent through UART:
* `f`: to toggle FIFO usage (enabled/disabled, enabled by default). Data will be read from sensor registers if FIFO is disabled.
* `c`: to print current configuration.
* `h`: to print helps screen.

## Terminal output

### Data format

Data are printed on the terminal as follow:
```
<timestamp> us Gyro: <gyr_x> <gyr_y> <gyr_z> dps [FSYNC occurred <fsync_delay> us ago]
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT1 was fired.
* `<gyr_x|y|z>`: Raw gyroscope value converted in dps.
* `<fsync_count>`: Only for sample tagged with FSYNC. Time in us between the FSYNC rising edge and the current sampling time.

### Example of output

```
[I] ###
[I] ### Example EIS
[I] ###
[I]       8190 us   Gyro:   -0.24    -0.06     0.00 dps   
[I]      13189 us   Gyro:   -0.37    -0.18     0.12 dps   
[I]      18189 us   Gyro:   -0.31    -0.18     0.00 dps   
[I]      23188 us   Gyro:   -0.31    -0.18     0.12 dps   
[I]      28187 us   Gyro:   -0.37    -0.12     0.00 dps   
[I]      33187 us   Gyro:   -0.31    -0.12     0.00 dps   
[I]      38186 us   Gyro:   -0.43    -0.12     0.06 dps   FSYNC occurred 3684 us ago
[I]      43185 us   Gyro:   -0.37    -0.06     0.00 dps   
[I]      48185 us   Gyro:   -0.31    -0.12    -0.06 dps   
[I]      53184 us   Gyro:   -0.37    -0.24     0.06 dps   
[I]      58183 us   Gyro:   -0.31    -0.12    -0.12 dps   
[I]      63183 us   Gyro:   -0.31    -0.12     0.00 dps   
[I]      68182 us   Gyro:   -0.31    -0.18     0.06 dps   FSYNC occurred 347 us ago
[I]      73181 us   Gyro:   -0.37    -0.06     0.00 dps   
[I]      78180 us   Gyro:   -0.37    -0.18    -0.06 dps   
[I]      83180 us   Gyro:   -0.31    -0.18     0.06 dps   
[I]      88179 us   Gyro:   -0.37    -0.06     0.06 dps   
[I]      93178 us   Gyro:   -0.31    -0.12     0.06 dps   
[I]      98178 us   Gyro:   -0.37    -0.18     0.06 dps   
[I]     103177 us   Gyro:   -0.43    -0.06     0.00 dps   FSYNC occurred 2011 us ago
[I]     108176 us   Gyro:   -0.37    -0.06     0.12 dps   
[I]     113176 us   Gyro:   -0.43    -0.12     0.12 dps   
 

```

We can verify the FSYNC signal frequency (expected to be 30 Hz) based on the last two FSYNC event detected above:

- fysnc_delta_time = (t1 - delay1) - (t0 - delay0)
- fsync_delta_time = (103177 - 2011) - (68182 - 347)
- fysnc_delta_time = 33331 us = 30.002 Hz
