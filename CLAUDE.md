# CLAUDE.md

This file guides Claude Code and other coding agents working in this repository. `AGENTS.md` is
a symlink to this file. Edit this file, not the link.

## Project Overview

This is Clean-Botix's fork of the Elite CS ROS 2 driver for ROS 2 Humble. It runs the Elite CS 612
arm in Optimus Clean, alongside MoveIt Pro. The default branch is `feature/moveit_pro`. The
`ros2app` repository consumes this driver and follows the same conventions.

### IMPORTANT: run format every time

C++ follows `.clang-format`. Format only the lines you changed with `git clang-format`, so
untouched upstream code keeps its layout.

### IMPORTANT: extend before you add, and shrink the diff

Before adding code, find the existing helper, constant, or pattern that already does part of
the job and extend it. Name it in the plan. If none exists, say so. Prefer a change that deletes
or replaces lines over one that only adds. A PR must read in one pass: no new abstraction for a
single caller, no new file for one function, no defensive branch for a case the types already
exclude. Run `/simplify` before opening a PR, or when a change exceeds about 100 net lines of
implementation code, tests not counted. Small fixes skip it.

### IMPORTANT: edit docs only where a change made a statement false

Documentation changes are corrections, not additions. Edit a doc only when your code change made
a statement in it incorrect or incomplete, and fix it in the same commit as the code. Do not
create markdown or CLAUDE.md files, add sections, or expand existing text unless asked. Markdown
edits follow the prose rules in `.claude/rules/prose.md`.

Every English doc has a Chinese twin: `README.md` and `README_CN.md`, `doc/ROS2Interface.md` and
`doc/ROS2Interface_CN.md`. A correction to one goes into both.

### Code comments and docstrings

Code carries the meaning. Names, types, small functions, and asserts say what a comment would
say. A comment exists only for what the code cannot express: a hardware fact, an external
contract, or a non-obvious why. Before writing one, rename or extract instead. A doc comment is
one sentence stating purpose. No `\param` or `\return` sections. The signature is the
documentation. When a comment is warranted, write **simplified technical english**: thesis
sentence first, then mechanism. One idea per sentence. Under 20 words per sentence. Active voice,
present tense, plain words. No connecting punctuation. No metaphors, no idioms, no synonyms for a
term the code already names. Describe the code as it is, never the change or the bug it fixed.
Never cite dates, ticket or PR numbers, commit hashes, or file line numbers. The commit message
carries that trace. Applies to tests too. A test file's header comment is one sentence naming the
invariant it pins, not the incident. Test cases carry no comments; the name says what is covered.
Delete comments that a change made redundant. Do not add them to untouched code. Use American
English.

### IMPORTANT: write the test first where it fits

Work test-first where the behavior can be pinned down before the code exists: helper logic,
state machines, parsing, protocol handling, failure paths. Write the failing test, watch it fail
for the right reason, then make it pass. Where a test-first loop does not fit (exploratory
hardware bring-up, a launch-file change, a pure refactor with existing coverage), say so rather
than retrofitting a test that only restates the implementation.

A test fake models the real collaborator, not the behavior the code under test expects. For
example, the controller manager fake in `test_controller_stopper.cpp` follows controller_manager
2.53.1's `STRICT` and `BEST_EFFORT` rules.

### IMPORTANT: a field bug gets a regression test that fails first

When fixing a bug observed on a real robot, the fix is not done until a test reproduces it:

1. Write a test that reproduces the field failure and **confirm it fails against the unfixed
   code**. A regression test that has never failed proves nothing.
2. Apply the fix and confirm the same test now passes.
3. Commit the test with the fix, and reference the Sentry issue or report in the commit message.

If the failure cannot be reproduced in a container (hardware-only timing, a controller firmware
state), say so explicitly in the PR and describe what was verified on the robot instead.

## Build and Test

A full build needs the Elite SDK and ROS 2 Humble. The `ghcr.io/clean-botix/moveit-drivers:dev`
image has both. Build from a copy of the source, so build artifacts stay out of the checkout:

```bash
docker run --rm -u root -v "$PWD":/src:ro -v driver_ws:/ws --entrypoint bash \
  ghcr.io/clean-botix/moveit-drivers:dev -c '
  unset CYCLONEDDS_URI; export ROS_LOCALHOST_ONLY=1
  source /opt/ros/humble/setup.bash
  rm -rf /ws/src/driver && mkdir -p /ws/src/driver
  (cd /src && tar --exclude .git -cf - .) | tar -xf - -C /ws/src/driver
  cd /ws && colcon build --packages-up-to eli_cs_robot_driver --cmake-args -DBUILD_TESTING=ON
  source install/setup.bash && ./build/eli_cs_robot_driver/test_controller_stopper'
```

The image's `CYCLONEDDS_URI` points at a file that exists only on the robot, so unset it.

## Git Workflow

- Feature branches should be named after the associated Linear issue where appropriate.
- Suggest a Conventional Commits message for each change (https://www.conventionalcommits.org/en/v1.0.0/)
