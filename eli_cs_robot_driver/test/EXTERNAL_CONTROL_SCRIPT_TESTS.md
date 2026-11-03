# Testing external_control.script

`external_control.script` runs on the Elite robot controller, but its logic is tested on any machine with Python 3 and pytest. A harness loads the script's function definitions into a sandbox. It replaces every robot builtin with a mock, so pytest can call the script's functions directly. Four test files build on the harness. They cover the script's functions, its servoj safety check, its wire-format contract with the Elite SDK, and the sync between its two copies.

A script change is ready when three things hold. The script tests pass. The ament linters pass on any changed test file. A run against the Elite simulator behaves as expected. The unit tests prove the logic, and the simulator proves the robot's interpreter accepts it.

## Quick start

The fastest loop runs the script tests directly with pytest. They need no ROS install and no build, and they finish in about a second.

```bash
python3 -m pytest src/external_dependencies/Elite_Robots_CS_ROS2_Driver/eli_cs_robot_driver/test/test_external_control_script_*.py
```

CI runs the same four files through `just unit-test-cov`, which `just ci-unit-test` calls. The recipe's package discovery skips everything under `src/external_dependencies`, so `_unit-test` names these files explicitly and runs them with pytest. Their results land in `build/external_control_script/pytest.xml`, and a failing test fails CI. The run deliberately skips the vendored package's ament linters, which fail on unmodified upstream code. Running `just unit-test` with no package argument reproduces the CI path locally.

```bash
just unit-test
```

The package-wide check runs every test in `eli_cs_robot_driver` through colcon, including the ament linters. Use it to lint a changed test file. Expect failures from the upstream launch files, `CMakeLists.txt`, and C++ sources. Only failures in files you changed matter.

```bash
just unit-test eli_cs_robot_driver
```

Line and branch coverage of the script itself is available through coverage.py. The harness compiles the script under its real path, so coverage attributes lines to the `.script` file.

```bash
cd src/external_dependencies/Elite_Robots_CS_ROS2_Driver/eli_cs_robot_driver
python3 -m coverage run --branch --include='*external_control.script' \
    -m pytest -q test/test_external_control_script_*.py
python3 -m coverage report -m
```

## Why the script is hard to test

Almost nothing about the script's environment supports testing. Each obstacle below shaped part of the rig.

**EliteScript is a proprietary dialect of Python with little public documentation.** It looks like Python, but its supported constructs are not fully specified anywhere this repo can reach. Nothing in the repo says whether it supports `return`, `not`, `break`, or `abs()`, for example. A construct that works in CPython can fail on the controller.

**No test framework exists for EliteScript.** There is no runner, no assertion library, and no mocking facility. The only native way to execute the script is on a controller or in the Elite simulator. Both are slow and manual, and they show motion rather than logic.

**The script must be a single file.** The driver sends it to the controller as one text blob. It cannot import modules of its own, so it cannot be split into testable units the usual way. Its only import is `threading`.

**The file is a template, not finished source.** It contains `{{NAME_REPLACE}}` placeholders that the SDK's `EliteDriver.cpp` substitutes before sending it. In CPython, `{{X}}` parses as a set inside a set, so the raw file runs but means the wrong thing.

**The script starts running as soon as it loads.** Its top level opens three sockets and enters an infinite control loop. Loading the whole file in a test would hang on the first socket.

**Every robot builtin is missing off the robot.** Calls such as `servoj()`, `socket_read_binary_integer()`, `textmsg()`, and `halt()` exist only in the controller's runtime.

**The threading model is undocumented.** The script runs several threads through `start_thread()`. Whether EliteScript threads are preemptive or cooperative is unknown, so no test can rely on real concurrency.

**The host contract lives in another language.** Packet layouts, enum values, and scaling factors are defined in the Elite SDK's C++ sources. The script repeats them as its own constants. Nothing at runtime checks that the two sides agree.

**Two copies of the script exist.** The driver's copy in `eli_cs_robot_driver/resources/` is the one sent at runtime. The SDK ships an identical copy in `Elite_Robots_CS_SDK/source/resources/`. Edits must land in both.

## How the rig answers each obstacle

Each obstacle has one dedicated countermeasure. The table maps them.

| Obstacle | Countermeasure |
|---|---|
| Undocumented dialect | A construct policy grounded in Elite's own upstream script, enforced by a static test |
| No test framework | pytest plus a harness that runs the script's functions under CPython |
| Single file | Header markers split definitions from the main loop, and loops delegate to one-iteration helpers |
| Template placeholders | The harness substitutes every placeholder before it runs the source |
| Runs on load | The harness runs only the header region, never the main loop |
| Missing builtins | Every robot builtin is a `MagicMock`, and a static test catches any that go unmocked |
| Unknown threading | A fake `start_thread()` records the call and never runs the target |
| Contract in C++ | Tests parse the SDK headers and build packets exactly as the C++ writers do |
| Two copies | A test requires the two copies to be byte-identical |

## Core components

### The script's layout

The script's structure makes it testable. Two markers divide it. Everything between `# HEADER_BEGIN` and `# HEADER_END` is constants and function definitions. Everything after `# HEADER_END` is the main loop, which opens sockets and runs forever.

Every infinite loop delegates its body to a one-iteration helper. `servoThread()` calls `runServoIteration()`, for example, and `scriptCommands()` calls `runScriptCommandIteration()`. The main loop is equally thin. It calls `openControlSockets()`, `initializeControlState()`, `runControlLoopIteration()` in a loop, and `shutdownControlThreads()`. Tests call the helpers directly and never need a loop to exit.

New logic must keep this shape. A loop body written inline cannot be tested without running the loop. A main-loop region that grows beyond thin calls escapes every test except the static ones.

### The harness

`external_control_script_harness.py` turns the script's header into a Python namespace. It is a plain helper module, so pytest does not collect it as a test. Tests call `load_control_script_namespace()` and then read functions, constants, and globals out of the returned dict.

The harness loads the script in three steps. `load_header_source()` reads the real file from disk and keeps only the text before `# HEADER_END`. It then replaces every `{{KEY}}` with a value from `DEFAULT_TEMPLATE_VALUES`, the same way `EliteDriver.cpp` does. Finally, `load_control_script_namespace()` runs that text with `exec()` into a dict pre-filled with mock builtins. Tests can override any placeholder through `template_overrides` and any builtin through `builtin_overrides`.

The mock builtins are `MagicMock` objects, with four deliberate exceptions.

- **`halt()` raises `ScriptHalted`.** A test can assert that the script halted without ending the test process. Passing `builtin_overrides={"halt": MagicMock()}` models a `halt()` that returns, which tests the script's defense against that case.
- **`start_thread()` never runs its target.** It returns an increasing fake handle. Tests drive thread bodies through their one-iteration helpers instead.
- **`norm()` returns the absolute value.** The script only passes it scalars.
- **`textmsg()` is reset after loading.** The header logs "Script started" at load time. The reset keeps that call out of every test's assertions.

The harness also reads the host contract straight from the SDK sources. `parse_sdk_enum()` extracts `enum class` values from the SDK headers. It feeds constants such as `CONTROL_MODE`, `SCRIPT_COMMAND`, and `TOOL_VOLTAGE`. These values come from C++, not from the script, so a test that uses them checks the script against the host.

Packet builders complete the harness. Each one mirrors a single SDK writer word for word, including its scaling and rounding. `reverse_servoj()` mirrors `EliteDriver::writeServoj()`, for example, and `script_command_set_payload()` mirrors `ScriptCommandInterface::setPayload()`. Every builder prepends the read count that `socket_read_binary_integer()` adds. That count shifts every host index up by one, which is the most common source of decoding bugs.

### The test files

Four test files divide the coverage by concern. Each has a module docstring that states its scope.

| File | Concern |
|---|---|
| `test_external_control_script_functions.py` | Each function's behavior, plus static checks of the script source |
| `test_external_control_script_safety.py` | The servoj start-target safety check, from unit level to an end-to-end replay of the snap-back incident |
| `test_external_control_script_wire_format.py` | Agreement with the SDK sources, and every packet type on all three sockets |
| `test_external_control_script_sync.py` | Byte identity of the driver and SDK copies |

The wire-format file has two halves. The first half checks static agreement with the SDK. It covers enum values, the placeholder set in `EliteDriver.cpp`, data sizes, zoom ratios, and the defaults for the configurable limits in the SDK header and the xacros. The second half feeds host-built packets through the reverse, trajectory, and script-command sockets. It then checks what the script does with each one.

### Static checks

Three tests inspect the script's source with Python's `ast` module instead of running it. They guard what the sandbox cannot reach.

- **Main-loop arity.** The main loop never runs under the harness. This test requires each call there to name a header function and pass the right number of arguments.
- **Mocked builtins.** Every call in the header must name a script function, a harness mock, or one of a few builtins both languages share (`int`, `str`, `range`). An unmocked robot builtin would otherwise resolve to a Python builtin of the same name.
- **Unproven constructs.** The header must not use `not`, `break`, `abs()`, `list.append()`, comprehensions, or lambdas. The next section explains why.

## Testing strategy

### Test behavior through helpers, not threads

Every behavior test calls a function synchronously and inspects mocks and globals afterward. No test starts a real thread or depends on scheduling. A test that needs a loop to run once gives a mock a side effect that ends the loop. `_stop_after()` in the functions file does this for the thread loops.

### Treat the host as the source of truth

Tests build packets from SDK enum values and SDK layouts, never from the script's own constants. A script constant that drifts from the SDK then fails a test instead of passing silently. The same rule covers placeholders. The test fails if the script, `EliteDriver.cpp`, and the harness defaults disagree on the set of placeholder names.

### Test the safety check at every level

The servoj start-target check guards against a real incident, so its tests are layered. Unit tests call `verifyServoStartSetpoint()` directly for the boundary, both directions, and the first offending joint. Mid-level tests go through `applyModeCommand()` and confirm a refused target never reaches `cmd_servo_joints`. End-to-end tests replay the stale-command incident through `runControlLoopIteration()` on a freshly initialized script. One test makes `halt()` return, and it proves the target is still refused.

### Admit only proven language constructs

A construct counts as proven when Elite's own upstream script uses it, or when a simulator run has exercised it. The upstream script is the best available evidence of what the interpreter supports, because Elite ships it for its own controllers. See [Elite-Robots/Elite_Robots_CS_SDK, `source/resources/external_control.script`](https://github.com/Elite-Robots/Elite_Robots_CS_SDK/blob/ca583c6a5e9d915ba0399147592c02cc95ab02b4/source/resources/external_control.script), pinned at commit `ca583c6`.

| Proven | Not proven, so avoided |
|---|---|
| `import threading`, `threading.Lock()`, `with` blocks | `not` |
| `return`, including inside a `with` block | `break` |
| `for i in range(...)`, `len()` | `abs()`, so use `norm()` |
| List element assignment | `list.append()` |
| `norm()`, `max()`, `min()`, `int()`, `str()` | List comprehensions and generator expressions |
| `str.format()` | `lambda` |
| `True`, `False`, `None`, `is None` | |
| Comma-separated `global a, b`, and `+=` | |

The static test enforces the right-hand column. A construct moves to the left column only with evidence. Update the static test in the same change.

### Prove each test catches its bug

Each fix in the script is checked by reintroducing its bug on purpose. The bug is applied to the script temporarily, the suite runs, and at least one test must fail before the file is restored. This practice is manual, not automated. It has caught weak tests more than once. Examples include a force-mode selection vector whose first element was zero, and a mutex test that could not see the lock. A reintroduced bug can also make a loop run forever. Give such a test a finite supply of mock reads, so it fails instead of hanging.

### Keep coverage complete

The header region has full line and branch coverage. A new branch without a test shows up immediately in the coverage report. The main-loop region stays uncovered by design, and the static arity test covers it instead.

## What the rig does not prove

The sandbox runs CPython, not EliteScript, so some risks stay outside it. A simulator run is the final check for each of them.

- **Interpreter acceptance.** A construct can pass every test and still fail to parse on the controller.
- **Timing and threading.** Real thread interleaving, servo cadence, and socket timeouts never run.
- **Motion.** The mocks record motion calls but cannot judge the motion that results.
- **Exact substitution text.** `DEFAULT_TEMPLATE_VALUES` mirrors the SDK's values, not always its exact formatting.
- **Deployment.** The tests read the script and SDK from source. A deployed robot can still run a stale SDK build, as the next section explains.

## Maintaining the rig

### Rules for changing the script

Most script changes need a matching harness or test change. The tests enforce the rules below, but knowing them up front saves a failing run.

- **Edit both copies.** Change the driver copy, then copy it byte for byte to `Elite_Robots_CS_SDK/source/resources/external_control.script`. The sync test fails otherwise.
- **Add a placeholder in three places.** Add the substitution to `EliteDriver.cpp`, a configuration field to `EliteDriverConfig` when the value is configurable, and a default to `DEFAULT_TEMPLATE_VALUES`. The placeholder test fails if any of them is missing.
- **Mock every new robot builtin.** Add it to `make_mock_builtins()`. The mocked-builtin test fails otherwise.
- **Extract every new loop body.** Give it a one-iteration helper, so tests can call it.
- **Check new constructs against upstream.** If the upstream script does not use a construct, avoid it, or prove it in the simulator first.

### Rules for adding tests

New test files follow the package's existing conventions. Missing any of them fails the colcon run, though not the direct pytest run.

- **Name the file `test_external_control_script_*.py`.** CI selects the script tests by that pattern. A file with any other name never runs in CI.
- **Register the file in CMake.** Add an `ament_add_pytest_test()` line in `eli_cs_robot_driver/CMakeLists.txt`. colcon only runs registered files.
- **Satisfy the ament linters.** `just unit-test eli_cs_robot_driver` runs flake8 with a 99-character line limit, and pep257. CI does not lint these files, so this check is manual. Docstring summaries start on the line after the opening quotes (D213) and end with a period.
- **Expect no repo-level lint.** The repo's ruff and pyright configurations exclude `src/external_dependencies`. Only the ament linters see these files.
- **Prefer the harness builders.** Build packets with the builders rather than by hand, so the test follows the host's real layout.

### Deployment coupling

The script and the SDK must ship together. The SDK fills in the script's placeholders at runtime. A new script with an old SDK reaches the robot with raw `{{...}}` text and fails to run.

The driver always builds and links against the SDK installed in `/usr/local`, not the workspace build. It links the SDK by bare library name, so the include and library paths come from the system defaults. The Docker image builds the SDK from this repo's source into `/usr/local`. Rebuild the image after any SDK change. In a running container, reinstall the workspace SDK into `/usr/local` instead.

## File map

| Path, relative to this directory | Role |
|---|---|
| [`../resources/external_control.script`](../resources/external_control.script) | The script sent at runtime |
| [`../../../Elite_Robots_CS_SDK/source/resources/external_control.script`](../../../Elite_Robots_CS_SDK/source/resources/external_control.script) | The SDK's identical copy |
| [`external_control_script_harness.py`](external_control_script_harness.py) | Loader, mocks, SDK contract parsing, packet builders |
| [`test_external_control_script_functions.py`](test_external_control_script_functions.py) | Function behavior and static checks |
| [`test_external_control_script_safety.py`](test_external_control_script_safety.py) | Servoj start-target safety check |
| [`test_external_control_script_wire_format.py`](test_external_control_script_wire_format.py) | SDK agreement and per-socket packet handling |
| [`test_external_control_script_sync.py`](test_external_control_script_sync.py) | Byte identity of the two copies |
| [`../CMakeLists.txt`](../CMakeLists.txt) | Test registration for colcon |
| [`../../../Elite_Robots_CS_SDK/source/Elite/EliteDriver.cpp`](../../../Elite_Robots_CS_SDK/source/Elite/EliteDriver.cpp) | Placeholder substitution |
