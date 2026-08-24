# Repository guidelines

## Scope

This repository implements the quadruped and wheel-legged robot control system.
It currently contains the dependency-free C++ core, motion runtime, MuJoCo backend,
Torch policy adapter, configuration loaders, and runnable simulation applications.

## Delivery priorities

- Prefer a working end-to-end control path over speculative completeness.
- Treat `rl_sar` as the behavior reference for policy observations, history, action
  conversion, and inference cadence. Treat `real_robot/mujoco` as the reference for
  MuJoCo scenes and interaction. Reuse verified behavior and only adapt language,
  dependency, and `RobotIO` boundaries.
- Keep the smallest implementation that satisfies the current request. Do not add a
  framework, abstraction layer, extension point, or configuration field for an
  unrequested future use case.
- Prefer modifying an existing source, test, or document. Add a file only when it has
  an independent responsibility; do not split code merely to satisfy a line-count goal.
- Keep one implementation for black and blackW differences, one primary test entry per
  module, and one current document per topic. Do not create milestone documents or test
  files for every small behavior.
- User requirements and the actual repository take precedence over stale plans or
  collaboration documents.

## Dependency boundaries

- `core/` must compile without ROS 2, Torch, MuJoCo, YAML libraries, or motor SDKs.
- Public core headers must not expose types owned by an external framework.
- Motion code may depend on `core`; backends and adapters may depend on `core`.
- `RobotIO` is the only robot-state and joint-command boundary used by motion code.
- Do not use ROS messages as `StateFrame`, `CommandFrame`, or other internal types.

## Data rules

- Use C++17.
- Use SI units: radians, radians per second, newton-metres, metres per second.
- Quaternion order is `w, x, y, z`.
- Timestamps are monotonic nanoseconds, not wall-clock timestamps.
- High-rate joint frames use fixed-capacity storage with `kMaxJoints == 16`.
- Every joint command has an explicit `ControlMode`; do not infer a mode from gains.
- Reject mismatched joint counts, expired commands, unknown enum values, and NaN/Inf
  before using a frame.
- Keep joint order explicit and validate the robot name and ordered joint names at startup.
  Do not infer the robot model from array length or carry model/calibration IDs in high-rate frames.
- Keep hardware calibration on the final execution side; motion code only uses normalized joint data.
- Treat `JointRole` as the normalized functional role. Do not infer it from an
  URDF/MJCF joint type, joint name, or an unusually large position range.
- Record whether a joint has position limits explicitly. Wheels normally use
  `JointRole::Wheel`, no position limit, and `JointImpedance` with zero KP plus
  target velocity and KD; the gains still do not select the control mode.
- Keep `MotionMode` coarse and stable. Select robot-specific motion features by
  `behavior_name`; do not add public enum values for a behavior's internal steps.

## Code style

### File headers

- Every C/C++ source/header, Python script, Shell script, and CMake file must begin
  with a short Chinese file header that states its purpose.
- Do not add author, creation date, or change history to file headers; Git already
  records that information.
- C/C++ files use this format before `#pragma once` or any `#include`:

  ```cpp
  /**
   * @file example.cpp
   * @brief 简要说明当前文件的作用。
   */
  ```

- Python, Shell, and CMake files use two short comment lines. A script shebang must
  remain on the first line:

  ```text
  # 文件：example.py
  # 作用：简要说明当前文件的作用。
  ```

### C and C++ formatting

- Use C++17 and follow the repository `.clang-format` file.
- Opening braces for namespaces, classes, structs, enums, functions, `if`, `else`,
  `for`, `while`, `switch`, and `try` blocks must be on a new line.
- Always use braces for control statements, including single-line bodies.
- Put only one statement on each line. Do not use comma expressions to hide steps.
- Prefer early returns to deeply nested conditionals.
- Keep lines at or below 100 characters when practical. Wrap long conditions and
  function arguments by logical group.
- Use `const` whenever a value should not change, and pass read-only non-trivial
  objects by `const&`.
- Use `auto` only when the type is obvious from the right-hand side or writing the
  type would reduce readability.
- Keep includes minimal and ordered: matching header, other project headers,
  C++ standard library, then third-party libraries. Do not rely on transitive includes.
- Use `snake_case` for functions and variables, `PascalCase` for types, and
  `kPascalCase` for constants.
- Prefer small types and functions with one clear responsibility.
- Keep public interfaces in `core/include/quadruped/core/` and implementation in
  `core/src/`.

### Comments and readability

- Write code comments in concise Chinese with normal Chinese punctuation.
- Comments should explain intent, constraints, units, ownership, or why a choice
  exists. Do not merely translate the next line of code into prose.
- Document non-obvious units, ranges, coordinate frames, sentinel values, ownership,
  or lifetimes where they affect correct use. Do not add comments to restate clear code.
- Add a short comment only before a genuinely non-obvious group of local variables.
- Public data structures should document every field whose meaning cannot be fully
  inferred from its name. Clearly distinguish similar fields such as online/valid,
  generated/applied sequence, and startup/session identifiers.
- Public interfaces should document behavior that callers cannot infer from the
  signature, especially units, lifetime, latest-value semantics, and failure rules.
- Do not keep commented-out code. Delete it and rely on Git history.
- TODO comments must state a concrete missing task or condition; avoid vague notes.
- Use meaningful names and named constants instead of unexplained numeric literals.

### Runtime behavior

- Avoid allocation, logging, file access, and blocking calls in future high-rate
  control paths.
- Return explicit status or validation results across module boundaries. Do not
  let exceptions cross `RobotIO` or periodic-control boundaries.

## Build and test

- Run the smallest affected build profile before handing off: `./scripts/build.sh` for
  shared core changes, `./scripts/build.sh --mujoco` for MuJoCo changes, and the RL
  profile when Torch behavior changes. Do not run every profile by default.
- Use `--clean` only when dependency discovery, profile options, generated configuration,
  or a suspected stale cache is part of the change.
- Prefer extending an existing integration test. Add focused tests for core state-machine
  behavior, numerical conversion, known bugs, and realistic safety failures that are not
  already covered. Simple field mapping, configuration wiring, and thin adapters do not
  require separate tests.
- Tests in M0 must not require network access or a third-party test framework.
- Do not commit `build/`, generated binaries, or `compile_commands.json`.

## Documentation style

- Keep `README.md` focused on project overview, setup, basic usage, and links to deeper documentation.
- Give each technical topic one authoritative document. Summarize and link elsewhere instead of repeating the same details.
- Describe the normal usage path first. Add failure cases only when they are useful for operation, debugging, or interface correctness.
- Reserve strong requirement words such as “必须”“不得” for real interface, data-integrity, concurrency, or safety invariants.
- Prefer concise factual descriptions over meta statements such as “本文只描述……”“当前范围不包括……”.
- Do not turn README sections into exhaustive configuration or API specifications; keep detailed field rules under `docs/`.
- Avoid documenting hypothetical edge cases that are neither implemented nor likely to help users.
- Keep status and validation notes short. Do not repeat architecture or usage instructions in status documents.
- When a code or configuration change affects existing documentation, update the authoritative document rather than adding another overlapping explanation.

## Change discipline

- Keep generated files and unrelated historical code out of this repository.
- Install pinned third-party binaries under `.deps/`; never commit that directory
  or hard-code a Python/Conda package path in CMake.
- Keep build, run, maintenance, and development helper scripts in `scripts/`.
- Keep design documents and their diagram sources/assets in `docs/`.
- Do not copy black and blackW implementations into separate source trees; use
  robot, controller, and policy configuration for their differences.
- Preserve the dependency direction instead of adding convenience includes from
  adapters or backends into the core.
