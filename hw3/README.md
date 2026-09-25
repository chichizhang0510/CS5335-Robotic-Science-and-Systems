# Homework 3 — ROS Actions

This workspace implements the four required parts of Project 3.  It uses two
ROS packages because a custom Action is an **interface**, while the client and
server are **nodes** that use that interface.

```text
action_interfaces  Task 1: MoveTurn.action
day3pkg            Task 2: ExecutiveNode (Action client)
                   Task 3: DrivingNode (Action server + /cmd_vel publisher)
                   Task 4: launch/day3.launch.py
scripts/             Extra Credit B: cancel_move_turn.bash
```

`MoveTurn.action` contains a distance in metres and an angle in degrees.  An
Action goal may request exactly one of them.  The driver rejects negative
values and requests where both fields are non-zero.  Degree input matches the
convention used in Homework 2; the Driving Node converts it to radians only
when calculating the required turn duration.

## Build

From this `hw3` directory, after sourcing the course ROS 2 installation:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Run the required demonstration

Start both nodes through the launch file:

```bash
ros2 launch day3pkg day3.launch.py
```

The Executive Node accepts these commands:

```text
move 0.50     # move forward 0.50 metres
turn 90       # turn counter-clockwise 90 degrees
figure8 0.20  # optional: trace two diamond-shaped loops, 0.20 m per side
p 0.20        # optional: trace a letter P, 0.20 m per side
quit
```

The default speeds are conservative (`0.10 m/s` and `0.30 rad/s`).  They are
ROS parameters on `DrivingNode`, so they can be adjusted after a safe physical
test if the actual TurtleBot travels slightly too far or too short.

## Extra Credit B — cancel an active goal

While a long movement is executing, run this in a second terminal (after
sourcing this workspace):

```bash
./scripts/cancel_move_turn.bash
```

The Driving Node has a reentrant callback group and a two-thread executor, so
it can receive the cancellation request while its execution callback is
publishing velocity commands.  It immediately publishes a zero-velocity
`TwistStamped` command and marks the goal cancelled.

## Extra Credit A — figure-eight and letter P commands

`figure8 <side-metres>` is implemented in the Executive Node as a sequence of
ordinary move/turn Action goals.  This deliberately reuses the required
Action instead of creating a separate direct `/cmd_vel` controller.  Stop the
sequence early by cancelling any goal with the Extra Credit B script.

`p <side-metres>` uses the same approach to trace a letter P.  Its vertical
line is twice the specified side length, and its top rectangle has sides equal
to the specified length.

## Important ROS distinction

An **Action is not a Node**.  It is the request/feedback/result contract and
the ROS communication mechanism for long-running work.  Here, the Executive
Node is the Action client and the Driving Node is the Action server.  A launch
file and an `.action` definition are not nodes themselves.
