# smd

This application demonstrates how to configure and use SMD feature. The algorithm detects when the user has moved significantly.

## Command interface

This application allows the following command to be sent through UART:
* `p`: to toggle Power-Save mode usage (enable/disable, disabled by default)
* `c`: to print current configuration.
* `h`: to print helps screen.

## Terminal output

### Data format

Data are printed on the terminal as follow:

```
<timestamp> us   SMD
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT1 was fired.

### Example of output

```
[I] ###
[I] ### Example SMD
[I] ###
[I]       6500645 us   SMD
[I]      14515148 us   SMD
```

