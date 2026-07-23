# RT Memory Locking — Test Plan

**Branch:** `ross/sw-933-investigate-mlockall` &nbsp;•&nbsp; **Ticket:** SW-933 &nbsp;•&nbsp; **PR:** #12

## What this change does

The control node (`eli_ros2_control_node`) locks its memory into RAM at startup so
the real-time control thread cannot be stalled by page faults, which would
otherwise starve the cyclic EtherCAT update and trip slave SM watchdogs
(observed as comm errors `0x0000` / `0x7500` / `0xFF0B`). Specifically it:

1. logs the `RLIMIT_MEMLOCK` it is running under,
2. calls `mlockall(MCL_CURRENT | MCL_FUTURE)`,
3. tunes glibc (`mallopt`) and pre-faults a heap reserve, and
4. periodically logs the control thread's page-fault counts.

**Critical dependency:** `mlockall()` only succeeds if the container grants a
sufficient `memlock` ulimit. The generated compose already grants
`ulimits.memlock: -1` (unlimited) to the drivers service **when
`REAL_TIME_KERNEL == 'true'`**, so for this test you force that flag via the
bot's `.env` (see "Getting this branch onto a bot"). Without the ulimit the lock
silently fails and the whole change is a no-op. Tests 1–2 below detect that
condition directly.

## Getting this (pre-merge) branch onto a bot

This driver is **not built directly**. It is a forked dependency vendored into the
MoveIt Pro workspace (`optimusclean/moveit`) via
[vendir](https://github.com/carvel-dev/vendir), compiled into the
`moveit-drivers` image, which the robot then runs. Because this branch is not yet
merged, you build a `moveit-drivers` image from it yourself and load it on the bot.

Two independent pieces must both be present on the bot for a valid test:

1. **The driver code** (this branch) — baked into the `moveit-drivers` image via
   vendir, per the steps below.
2. **The `memlock: -1` ulimit** — set `REAL_TIME_KERNEL="true"` in the bot's
   `.env` (see next section). The generated compose already grants
   `ulimits.memlock: -1` (and `rtprio: 99`) to the drivers service **when
   `REAL_TIME_KERNEL == 'true'`**, so forcing that flag is enough — no separate
   compose change is needed for the test. Without the ulimit, `mlockall()` fails
   and Test 2 will show it.

### Force the memlock ulimit via `.env`

`optimusclean-dev` resolves `REAL_TIME_KERNEL` as `${REAL_TIME_KERNEL:-<auto>}`
after sourcing the bot's `.env`, so a value set there overrides the auto-detected
(kernel-based) default. Setting it `true` makes the compose renderer emit the
`memlock`/`rtprio` ulimits for `moveit_drivers`.

Notes:
- This also wraps the drivers command in `chrt -r ${MOVEIT_DRIVERS_RT_PRIORITY:-99}`.
  That is intended (RT scheduling for the driver) and works on a stock kernel;
  `PREEMPT_RT` only improves latency guarantees, it is not required for `chrt`.
- The compose is **rendered from a template at bring-up**, so the `.env` change
  only takes effect after you re-run the `optimusclean-dev` bring-up (which
  re-renders `docker-compose.yml` and recreates containers). A bare
  `docker compose up` against the old rendered file will not pick it up.

The vendir pin lives in `optimusclean/moveit/vendir.yml` under
`src/external_dependencies/Elite_Robots_CS_ROS2_Driver` (`ref:`), and the resolved
SHA is recorded in `vendir.lock.yml`.

In an `optimusclean/moveit` workspace:

1. Point the pin at this branch (temporary — **do not commit** the vendir change):
   ```bash
   cd optimusclean/moveit
   # in vendir.yml, under Elite_Robots_CS_ROS2_Driver, set:
   #     ref: ross/sw-933-investigate-mlockall
   just deps-sync                 # vendir sync + perms fix; pulls the branch in
   ```
2. Confirm the branch's code actually landed in the workspace:
   ```bash
   ls src/external_dependencies/Elite_Robots_CS_ROS2_Driver/eli_cs_robot_driver/src/rt_memory.cpp
   grep -R "configure_realtime_memory" \
     src/external_dependencies/Elite_Robots_CS_ROS2_Driver/eli_cs_robot_driver/
   ```
3. Build the `moveit-drivers` image (dev compose):
   ```bash
   COMPOSE_FILE=docker-compose.dev.yml docker compose build drivers
   ```
4. Get the image onto the bot — either push it to GHCR under the tag the bot
   pulls (`ghcr.io/clean-botix/moveit-drivers:dev`), or copy it directly:
   ```bash
   docker save ghcr.io/clean-botix/moveit-drivers:dev | ssh <bot> 'docker load'
   ```
5. On the bot, make sure `REAL_TIME_KERNEL="true"` is in `.env` (above), then
   re-run the `optimusclean-dev` bring-up so the compose is re-rendered with the
   ulimit and the drivers container is recreated on the new image.

> After deploying, run Test 2 first — it confirms both pieces (driver code +
> ulimit) actually took effect before you invest in the rest.

---

## Test 2 — Lock is active on the running robot (the key check)

Validates that `mlockall()` actually took effect on this bot. ~30 seconds.

### 2a. Startup log

```bash
docker logs $C 2>&1 | grep -iE "RLIMIT_MEMLOCK|mlockall"
```

**Pass:**
- a line `RLIMIT_MEMLOCK: soft=unlimited hard=unlimited`, **and**
- **no** `mlockall failed ...` line.

**Fail:** `soft=0 MiB` / a small MiB value, or an `mlockall failed (...)` warning.
→ the `memlock` ulimit is missing (see Critical dependency above). Fix the
compose ulimit and redeploy before continuing.

### 2b. Locked memory (`VmLck`)

```bash
docker exec $C bash -c \
  'PID=$(pgrep -f eli_ros2_control_node | head -1); grep -E "VmLck|VmRSS" /proc/$PID/status'
```

**Pass:** `VmLck` is large — tens to hundreds of MB, on the order of `VmRSS`.
**Fail:** `VmLck: 0 kB` → nothing is locked; the change is not effective.

> Record the `VmLck` / `VmRSS` values in the results table.

---

## Test 3 — Mechanism demonstration (off-robot, deterministic)

Validates the kernel behavior the fix relies on: locked memory is not reclaimed.
No robot cycle, root, or swap required. Runs in the dev/drivers container or any
Linux box. Source: `test/manual/mlock_demo.cpp`.

```bash
g++ -O2 -std=c++17 test/manual/mlock_demo.cpp -o /tmp/mlock_demo
ulimit -l unlimited        # mirror the deployed drivers ulimit
/tmp/mlock_demo --nolock
/tmp/mlock_demo --lock
```

**Pass:**
- `--nolock` → `madvise ... succeeded` and a large fault count (~32768 pages),
  "unlocked memory was reclaimed".
- `--lock` → `madvise ... refused -- Invalid argument` and **0** faults,
  "locked memory stayed resident".

**Fail:** `--lock` reports `mlockall failed` (your shell's `ulimit -l` is too
low — re-run after `ulimit -l unlimited`) or a nonzero fault count.

---

## Test 4 — Page-fault self-logging (ongoing observability)

Validates the in-node monitor and gives the steady-state fault picture. The node
logs a `page-fault check (control thread): ...` line every ~30 s.

Let the robot run and perform normal cyclic work (ideally the operation that
historically threw the comm errors), then:

```bash
docker logs --since 10m $C 2>&1 | grep "page-fault check"
```

Example line:

```
page-fault check (control thread): +0 major, +4 minor over 30s (0.00 major/s); lifetime 2 major / 51230 minor
```

**Interpret:**
- **`major/s` ≈ 0 in steady state** → the control thread is not blocking on I/O
  faults. This is the healthy target and the outcome the lock is meant to hold.
- **`major/s` > 0 in steady state** (logged as a WARN) → the control thread is
  still taking blocking faults; investigate (lock not effective, or file-backed
  faults from an unexpected source).
- A burst of **minor** faults only right after startup, then quiet, is expected
  (that is the one-time working-set warm-up).

**Pass:** the `page-fault check` lines appear, and steady-state `major/s` is 0.

---

## Test 5 (optional) — A/B under memory pressure

Only needed to *quantify the benefit* (answers "were faults actually happening?").
Skippable if Tests 2–4 pass and field behavior is already good.

The idea: create memory pressure while the robot runs and compare the control
thread's major-fault growth with the lock working vs. defeated.

1. **Lock working (`REAL_TIME_KERNEL="true"`, `memlock: -1`):** run the robot
   under load, watch Test 4's `major/s` — expect it to stay ~0.
2. **Lock defeated (baseline):** set `REAL_TIME_KERNEL="false"` in `.env` and
   re-run the bring-up so the compose renders without the `memlock` ulimit,
   confirm Test 2a now shows `mlockall failed`, then repeat under the same load —
   expect `major/s` to climb. Restore `REAL_TIME_KERNEL="true"` afterward.

Generate pressure on the host (if `stress-ng` is available):

```bash
stress-ng --vm 2 --vm-bytes 90% --timeout 120s
```

**Pass:** major-fault rate is materially lower (ideally 0) with the lock working
than with it defeated, under the same load.

---

## Results

| Test | Result (pass/fail) | Notes / values |
|------|--------------------|----------------|
| 1 — unit tests | | |
| 2a — startup log | | `RLIMIT_MEMLOCK: soft=___ hard=___`, mlockall failed? |
| 2b — VmLck | | `VmLck=___`, `VmRSS=___` |
| 3 — mlock_demo | | nolock faults=___, lock faults=___ |
| 4 — self-logging | | steady-state major/s=___ |
| 5 — A/B (optional) | | locked major/s=___, defeated major/s=___ |

**Tester:** ___________  **Robot / host:** ___________  **Date:** ___________
**Kernel:** `uname -a` → ___________ (PREEMPT_RT?  yes / no)

## Notes for the tester

- **`REAL_TIME_KERNEL` controls the ulimit.** The compose only grants the drivers
  `memlock`/`rtprio` ulimits when `REAL_TIME_KERNEL == 'true'`. It auto-detects
  from the kernel (`true` only on `PREEMPT_RT`), which is why forcing it in `.env`
  is required for this test on a stock kernel. Record the kernel (`uname -a`) and
  confirm Test 2a regardless.
- **The strongest field signal** is the absence of the `0x7500` / `0xFF0B` comm
  errors during sustained cycle testing. Tests here confirm the *mechanism* is
  active so that field result can be correctly attributed to this change rather
  than to co-deployed changes (e.g. native EtherCAT, SW-901).
