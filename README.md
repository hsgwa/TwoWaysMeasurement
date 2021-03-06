Two Ways Test
====

# About
- Check Real-Time performance by following scenarios:
  - ping-pong
- ping-pong is implemented by 2 topics with following publisher and subscriber.
  - ping-publisher(ping-pub) sends ping periodically using ping-topic.
  - ping-subscriber(ping-sub) and pong-publisher(pong-pub)
    ping-sub subscribes ping-topic and optionally pong-pub sends pong by pong-topic.
  - pong-subscriber(pong-sub) subscribes pong-topic.
- There are some variations for the number of Executors and Nodes, e.g.
  - 1 Executor 1 Node
  - 1 Executor 2 Node
    - Node1 contains ping-pub + pong_sub
	- Node2 contains ping-sub + pong-pub
  - seperate above 2 Node into 2 Executors
- Service version ping-pong is also implemented but obsolute.

# Environment
- Ubuntu 18.04
  - If you want to run with scheduling option etc, you must run as root.
- ROS 2 Foxy

# Build

```
# This has many sub packages
colcon list

# build all
colcon build --symlink-install

# build what you want
colcon build --symlink-install --packages-up-to tw_ping_pong
```

# Run

```
# default option
./build/tw_topic/tw_ping_pong

# main-thread: RR98, child-thread: RR97, run-type 1e2n
./build/tw_topic/tw_ping_pong \
    --main-sched RR98 --child-sched RR97 \
	--run-type 1e2n \
    --ros-args --param num_loops:=1000 -param period_ns:=1000000
```

Available options

| type           | option            | default value | comment                                             |
|----------------|-------------------|--------------:|-----------------------------------------------------|
| command option | --sched-rrts      |           off | use RR-TS scheduler(OBSOLUTE)                       |
|                | --sched-rrrr      |           off | use RR-RR scheduler(OBSOLUTE)                       |
|                | --ipm             |           off | use intra process manager(communication)            |
|                | --round-ns <ns>   |          1000 | histogram bin width in [ns]                         |
|                | --num-skip <num>  |            10 | skip first N event to prevent initial overhead      |
|                | --static-executor |               | use StaticSingleThreadedExecutor                    |
|                | --main-sched      |            TS | scheduling policy of main  thread, use RR98/RR97/TS |
|                | --child-sched     |            TS | scheduling policy of child thread, use RR98/RR97/TS |
|                | --run-type        |          1e1n | executor/node setting. see below                    |
| parameter      | period_ns         |    10,000,000 | timer period[ns]                                    |
|                | num_loops         |        10,000 | number of loops                                     |
|                | debug_print       |         false | print debug message                                 |

run-type

| value   | meaning                     |
|---------|-----------------------------|
| 1e1n    | 1 executor, 1 node          |
| 1e2n    | 1 executor, 2 nodes         |
| 2e_ping | ping process in 2 executors |
| 2e_pong | pong process in 2 executors |

# Common Output
- Processes outputs following log to stdout as following.
  They output "recent values" and "average" of some metrics.

```
wakeup_jitters recent values: 
  202076 211784 216894 219512 233099 ...
wakeup_jitters average: 255350
recv_jitters recent values: 
  234106 178137 168598 168912 181125 ...
recv_jitters average: 178862
```

Here are meanings.
**unit are [ns], but chrono::system_clock is used internally.**
**As in https://cpprefjp.github.io/reference/chrono/system_clock.html, gcc has microseconds resolution.**

| name           | value                                                                | unit |
|----------------|----------------------------------------------------------------------|------|
| wakeup_jitters | ping-pub wake up time-diff between expected wake up time and actual. | [ns] |
| recv_jitters   | time-diff between ping-pub send and ping-sub recv.                   | [ns] |

