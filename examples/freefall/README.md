# freefall

This application demonstrates how to configure and use FreeFall feature. Two types of events are detected by the algorithm:
* LOW-G: aims at detecting when the device is free-falling (before the end of the fall).
* FREEFALL: aims at detecting when the device felt and hit the ground.

## Terminal output

### Data format

Data are printed on the terminal as follow:

```
<timestamp> us   LOW_G
<timestamp> us   FREEFALL     length: <ff_length> cm (<ff_duration> ms)
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT1 was fired.
* `<ff_length>`: Length of the FreeFall in centimeters, computed based on the number of samples.
* `<ff_duration>`: Duration of the FreeFall in milliseconds, computed based on the number of samples.

### Example of output

```
[I] ###
[I] ### Example FreeFall
[I] ###
[I]       2030564 us   LOW_G
[I]       2033061 us   LOW_G
[I]       2035565 us   LOW_G
[I]       2038070 us   LOW_G
[I]       2040573 us   LOW_G
[I]       2043077 us   LOW_G
[I]       2045581 us   LOW_G
[I]       2048084 us   LOW_G
[I]       2050588 us   LOW_G
[I]       2053092 us   LOW_G
[I]       2055596 us   LOW_G
[I]       2058099 us   LOW_G
[I]       2060603 us   LOW_G
[I]       2063106 us   LOW_G
[I]       2065610 us   LOW_G
[I]       2068115 us   LOW_G
[I]       2070618 us   LOW_G
[I]       2073122 us   LOW_G
[I]       2075626 us   LOW_G
[I]       2078131 us   LOW_G
[I]       2080633 us   LOW_G
[I]       2083137 us   LOW_G
[I]       2085641 us   LOW_G
[I]       2088145 us   LOW_G
[I]       2090649 us   LOW_G
[I]       2093153 us   LOW_G
[I]       2095657 us   LOW_G
[I]       2098162 us   LOW_G
[I]       2100665 us   LOW_G
[I]       2103168 us   LOW_G
[I]       2105673 us   LOW_G
[I]       2108176 us   LOW_G
[I]       2110681 us   LOW_G
[I]       2113184 us   LOW_G
[I]       2115688 us   LOW_G
[I]       2118191 us   LOW_G
[I]       2120696 us   LOW_G
[I]       2123200 us   LOW_G
[I]       2125704 us   LOW_G
[I]       2128208 us   LOW_G
[I]       2130712 us   LOW_G
[I]       2133216 us   LOW_G
[I]       2135720 us   LOW_G
[I]       2138224 us   LOW_G
[I]       2140727 us   LOW_G
[I]       2143231 us   LOW_G
[I]       2145735 us   LOW_G
[I]       2148239 us   LOW_G
[I]       2150742 us   LOW_G
[I]       2153247 us   LOW_G
[I]       2155750 us   LOW_G
[I]       2158254 us   LOW_G
[I]       2160760 us   LOW_G
[I]       2163262 us   LOW_G
[I]       2170775 us   LOW_G
[I]       2173278 us   LOW_G
[I]       2175783 us   LOW_G
[I]       2178287 us   LOW_G
[I]       2180791 us   LOW_G
[I]       2183295 us   LOW_G
[I]       2185800 us   LOW_G
[I]       2188303 us   LOW_G
[I]       2190807 us   LOW_G
[I]       2193311 us   LOW_G
[I]       2195815 us   LOW_G
[I]       2198320 us   LOW_G
[I]       2215815 us   FREEFALL     length: 17.15 cm (75 ms)
```

