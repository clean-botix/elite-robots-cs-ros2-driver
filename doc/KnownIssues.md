# Known Issues

## Late trajectory points can leak into the next trajectory

When trajectories are forwarded to the arm, a point that arrives more than `trajectory_point_timeout_s` late can run as part of the next trajectory. The arm would then move toward a target from the earlier trajectory.

This affects only trajectory forwarding, through `EliteDriver::writeTrajectoryControlAction()` and `writeTrajectoryPoint()`. The ROS2 driver never forwards trajectories itself, because it streams `servoj` targets instead. Only clients that use the Elite SDK directly are exposed.

### The larger problem

The trajectory port carries points with no trajectory or sequence ID. The script only counts them. A START action tells it the next *n* points belong to the new trajectory.

Once a point goes missing or arrives late, the count and the socket stream disagree. The script then has no way to tell which trajectory a point belongs to. Two failure modes pull in opposite directions:
- **Draining too little** leaves an old trajectory's points in the socket, and they run as the next trajectory's first waypoints.
- **Draining too much** swallows the next trajectory's own points, so the arm skips waypoints.

No choice of drain length avoids both. The protocol simply has no way to tell an old point from a new one.

### Current approach: one timeout

`trajectory_point_timeout_s` sets how long a point may be late before the script treats it as never coming. It defaults to 1.0 s. It is an `EliteDriverConfig` field and an optional driver hardware parameter, and the script receives it as `TRAJECTORY_POINT_TIMEOUT`.
- **A running trajectory** that waits longer than the timeout for its next point reports `FAILURE`. It stops its motion and abandons its remaining points, so the next START keeps every point of its own.
- **Replacing or cancelling a trajectory** drains its remaining points. Points the host already sent sit buffered and drain at once. A straggler gets up to the same timeout per silent gap.

**What the timeout guarantees:**
- The main loop never blocks forever on a missing point.
- A trajectory that times out never eats the next trajectory's points.
- A trajectory that finishes, fails, or is cancelled reports exactly one result. One replaced by a new START reports none, because a result at that moment could be read as the new trajectory's.

**What it doesn't guarantee:** a point later than the timeout is still read as part of the next trajectory.

A host that legitimately sends points further apart than 1 s gets a `FAILURE` and a stop. It must raise `trajectory_point_timeout_s` to fit its pacing.

### To do: unique trajectory IDs

Closing the issue for good needs a trajectory ID on every point. It is a significant change, spanning the SDK's writers, the script, and the tests.
- **START carries an ID.** `ReverseInterface::writeTrajectoryControlAction()` writes a new trajectory ID into an unused word of the reverse packet. Words `data[3]` through `data[6]` are always zero for that action today.
- **Every point carries the same ID.** `TrajectoryInterface::writeTrajectoryPoint()` writes it into one of the twelve words it now always zeroes, `data[6]` through `data[17]`.
- **The SDK tracks the current ID**, so `EliteDriver`'s public API need not change. START generates the ID, and each following `writeTrajectoryPoint()` stamps it.
- **The script discards any point whose ID doesn't match** the current trajectory. Stale points then never run, and the drain no longer needs a timeout to guess where one trajectory ends. The timeout would still detect a host that stopped sending.
- **The tests extend to match.** The harness's packet builders gain the ID, and new tests cover late points from earlier trajectories, out-of-order IDs, and ID reuse after wraparound.
