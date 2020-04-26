# Experiment log 2020-04-20

## SEHS

Settings:

```xml
<param name="map_topic" value="/obstacles/costmap/costmap" />
<param name="normal_frequency" value="0.25" />
<param name="max_frequency" value="10" />

<param name="throttle_levels" value="7" />
<param name="steering_levels" value="13" />

<param name="min_throttle" value="-1.0" />
<param name="max_throttle" value="1.0" />
<param name="safety_margin" value="$(arg planning_safety_margin)" />

<param name="time_step_s" value="0.02" />
```

- Average planning time: 291ms
- Minimum planning time: 115ms
- Maximum planning time: 640ms
- Replannings per lap: 9-10 (once every 4 seconds)
- Longest time being unable to replan: 2s

## DWA

Command: `roslaunch racer_simulator dwa.launch`

Settings:

```xml
<param name="frequency" value="50" />

<param name="distance_to_obstacle_weight" value="650.0" />
<param name="position_weight" value="500.0" />
<param name="heading_weight" value="250.0" />
<param name="velocity_weight" value="400.0" />

<param name="prediction_horizon_s" value="$(arg horizon)" />
<param name="integration_step_s" value="0.02" />
<param name="throttle_levels" value="20" />
<param name="steering_levels" value="33" />
<param name="min_throttle" value="0.1" />
<param name="max_throttle" value="1.0" />

<arg name="planning_safety_margin" default="0.3" />
```

- average lap time: 37.72s
- best lap time: 35.90s
- highest reached speed: 3.41m/s
- recording: `dwa-experiment-no-obstacles-2020-04-20_18.10.02.mp4`

| Lap number | Time [s]  | Distance travelled [m] | Average speed [m/s] | Maximum speed [m/s] |
|:----------:|:---------:|-----------------------:|--------------------:|--------------------:|
| #1         | 37.73     | 84.65                  | 2.29                | 2.90                |
| #2         | 36.65     | 86.65                  | 2.51                | 3.41                |
| #3         | 36.95     | 86.56                  | 2.40                | 3.22                |
| #4         | 36.35     | 86.23                  | 2.45                | 3.19                |
| #5         | 40.49     | 88.39                  | 2.24                | 3.04                |
| #6         | 41.00     | 88.20                  | 2.25                | 3.17                |
| #7         | 37.96     | 85.77                  | 2.31                | 2.87                |
| #8         | 36.57     | 86.39                  | 2.41                | 3.11                |
| #9         | 39.21     | 87.50                  | 2.28                | 2.96                |
| #10        | 36.43     | 86.43                  | 2.48                | 3.37                |
| #11        | 36.78     | 84.67                  | 2.35                | 3.13                |
| #12        | 38.47     | 85.85                  | 2.29                | 3.24                |
| #13        | 35.90     | 86.93                  | 2.47                | 3.18                |
| #14        | 37.62     | 85.49                  | 2.34                | 3.16                |
    
## Pure Pursuit

Command: `roslaunch racer_simulator pure_pursuit.launch`

Settings:

```xml
<param name="frequency" value="50" />

<param name="min_lookahead" value="0.5" />
<param name="max_lookahead" value="2.0" />

<arg name="planning_safety_margin" default="0.3" />
```

- average lap time: 36.44s
- best lap time: 33.86s
- highest reached speed: 3.42m/s
- recording: `pure-pursuit-experiment-no-obstacles-2020-04-20_18.45.57.mp4`

| Lap number | Time [s]  | Distance travelled [m] | Average speed [m/s] | Maximum speed [m/s] |
|:----------:|:---------:|-----------------------:|--------------------:|--------------------:|
| #1         | 37.71     | 88.11                  | 2.53                | 3.37                |
| #2         | 36.83     | 86.18                  | 2.39                | 3.21                |
| #3         | 38.06     | 87.10                  | 2.36                | 3.13                |
| #4         | 36.01     | 87.08                  | 2.48                | 3.21                |
| #5         | 35.36     | 85.75                  | 2.50                | 3.40                |
| #6         | 40.07     | 91.32                  | 2.36                | 3.16                |
| #7         | 36.66     | 85.28                  | 2.37                | 2.95                |
| #8         | 33.86     | 84.93                  | 2.57                | 3.22                |
| #9         | 36.17     | 87.19                  | 2.48                | 3.35                |
| #10        | 35.74     | 89.02                  | 2.55                | 3.19                |
| #11        | 35.24     | 86.04                  | 2.50                | 3.32                |
| #12        | 34.30     | 87.58                  | 2.62                | 3.39                |
| #13        | 36.72     | 88.89                  | 2.49                | 3.20                |
| #14        | 37.00     | 91.35                  | 2.53                | 3.16                |
| #15        | 36.92     | 89.68                  | 2.50                | 3.42                |













