# pedometer

This application demonstrates how to configure and use Pedometer feature. The algorithm will detect steps and provide details such as the cadence and the estimated activity type (Walk, Run, Unknown).

## Command interface

This application allows the following command to be sent through UART:
* `a`: to toggle Pedometer configuration (Normal/Low-Power, Normal by default)
* `p`: to toggle Power-Save mode usage (enable/disable, disabled by default)
* `c`: to print current configuration.
* `h`: to print helps screen.

## Terminal output

### Data format

Data are printed on the terminal as follow:

```
<timestamp> us   STEP_DET count: <step_count> steps cadence: <cadence> steps/s activity: <activity_class>
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT1 was fired.
* `step_count`: Number of steps as reported by eDMP
* `cadence`: Number of steps per second
* `activity_class`: Type of activity detected amongst (Run, Walk, Unknown)

**Note:** Step count and cadency are valid only once the step count starts incrementing. `<activity_class>` will remain _Unknown_ meanwhile.

### Example of output

```
[I] ###
[I] ### Example Pedometer
[I] ###
[I]       2818306 us   STEP_DET     count:     0 steps  cadence: 2.0 steps/s  activity: Unknown
[I]       3177550 us   STEP_DET     count:     0 steps  cadence: 2.0 steps/s  activity: Unknown
[I]       3556702 us   STEP_DET     count:     6 steps  cadence: 2.0 steps/s  activity: Unknown
[I]       3915950 us   STEP_DET     count:     7 steps  cadence: 2.1 steps/s  activity: Walk
[I]       4255250 us   STEP_DET     count:     8 steps  cadence: 2.2 steps/s  activity: Walk
[I]       4594566 us   STEP_DET     count:     9 steps  cadence: 2.3 steps/s  activity: Walk
[I]       4933861 us   STEP_DET     count:    10 steps  cadence: 2.4 steps/s  activity: Walk
[I]       5273155 us   STEP_DET     count:    11 steps  cadence: 2.5 steps/s  activity: Run
[I]       5632388 us   STEP_DET     count:    12 steps  cadence: 2.5 steps/s  activity: Run
[I]       5991618 us   STEP_DET     count:    13 steps  cadence: 2.6 steps/s  activity: Run
[I]       6350879 us   STEP_DET     count:    14 steps  cadence: 2.6 steps/s  activity: Run
[I]       6710180 us   STEP_DET     count:    15 steps  cadence: 2.6 steps/s  activity: Run
[I]       7069450 us   STEP_DET     count:    16 steps  cadence: 2.7 steps/s  activity: Run
```

