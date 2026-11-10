# Deviations

## Initialization time (~9.0–9.1 s vs plan ~8.6–8.7 s)

Straight-segment heading change is tracked as the net integrated gyro (`∫ ω_m dt`) over the B-window segment rather than `∫ |ω_m| dt`. With bias 0.01 rad/s this removes spurious restarts from the old integral-of-absolute test. Measured init time on seed 1: **9.1 s** (was 13.3 s with ∫|ω|).
