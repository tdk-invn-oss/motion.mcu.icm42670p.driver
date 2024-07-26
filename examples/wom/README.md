# wom

This application demonstrates how to configure and use wom (Wake-On-Motion) feature.

## Terminal output

### Data format

Data are printed on the terminal as follow:

```
<timestamp> us   WOM  X=<x> Y=<y> Z=<z>
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT1 was fired.
* `<x>`: 1 when motion through X axis exceeds configured threshold, 0 otherwise.
* `<y>`: 1 when motion through Y axis exceeds configured threshold, 0 otherwise.
* `<z>`: 1 when motion through Z axis exceeds configured threshold, 0 otherwise.

### Example of output

```
[I] ###
[I] ### Example WOM
[I] ###
[I]       1846092 us   WOM  X=0 Y=0 Z=1
[I]       1926230 us   WOM  X=1 Y=0 Z=1
[I]       2006377 us   WOM  X=1 Y=0 Z=1
[I]       2086516 us   WOM  X=1 Y=0 Z=1
[I]       2166653 us   WOM  X=1 Y=0 Z=1
[I]       2246786 us   WOM  X=1 Y=1 Z=0
[I]       2326920 us   WOM  X=1 Y=1 Z=1
[I]       2407051 us   WOM  X=1 Y=0 Z=1
[I]       2487187 us   WOM  X=0 Y=0 Z=1
[I]       2567319 us   WOM  X=0 Y=0 Z=1
[I]       2647450 us   WOM  X=1 Y=1 Z=1
[I]       2727583 us   WOM  X=1 Y=0 Z=1
[I]       2807717 us   WOM  X=0 Y=0 Z=1
```

