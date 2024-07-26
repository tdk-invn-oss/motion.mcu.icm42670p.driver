# tilt

This application demonstrates how to configure and use tilt feature.

Once the example is running, tilting the board with an angle of 30 degrees or more will generate a tilt. By default, a tilt event is generated when the position is held for 4 seconds.

## Command interface

This application allows the following command to be sent through UART:
* `a`: to toggle Tilt configuration (High-Performance/Low-Power, High-Performance by default)
* `p`: to toggle Power-Save mode usage (enable/disable, disabled by default)
* `c`: to print current configuration.
* `h`: to print helps screen.

## Terminal output

### Data format

Data are printed on the terminal as follow:

```
<timestamp> us   TILT
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT1 was fired.

### Example of output

```
[I] ###
[I] ### Example Tilt
[I] ###
[I]       6374731 us   TILT
[I]      11683617 us   TILT
```

