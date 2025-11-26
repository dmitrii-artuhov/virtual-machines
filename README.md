# L1 Cache properties calculation

The tool calculates cache size, associativity, and line length.

## Build & Run

```cmd
make && ./build/main [--debug] [--as-ratio x] [--ln-ratio y]
```

Parameters:
- `--debug` enables debug logs (does not expect any values)
- `--as-ratio` (default `0.3`) sets the minimal percentage difference between array reading times for cache associativity/size detection
- `--ln-ratio` (default `0.11`) sets the minimal percentage difference between array reading time for cache line detection

Might be useful to tweak them if the results are fluctuating.

Program will output L1 cache properties to stdout:
```
Detected L1 cache associativity: 8
Detected L1 cache size: 128KB
Detected L1 cache line size: 128B
```

## Results

### Mac M4 Max
```
Detected L1 cache associativity: 8
Detected L1 cache size: 128KB
Detected L1 cache line size: 128B
```

On mac associativity is not specified anywhere, but other properties are:

```
$> sysctl -a | grep hw.
hw.l1icachesize: 131072
hw.cachelinesize: 128
```

### Intel i7-8550U
```
Detected L1 cache associativity: 8
Detected L1 cache size: 32KB
Detected L1 cache line size: 64B
```

Which matches the specs:

```
$> cat /sys/devices/system/cpu/cpu0/cache/index0/ways_of_associativity 
8
$> cat /sys/devices/system/cpu/cpu0/cache/index0/size
32K
$> cat /sys/devices/system/cpu/cpu0/cache/index0/coherency_line_size
64
```