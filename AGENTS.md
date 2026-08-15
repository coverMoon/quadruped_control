# Repository guidelines

## Scope

This repository implements the quadruped and wheel-legged robot control system.
M0 is the dependency-free C++ core. M1 adds MuJoCo only to its own backend;
ROS 2, Torch, hardware SDKs, and application assembly belong to their own later
modules.

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
- Reject mismatched model IDs, calibration IDs, joint counts, expired commands,
  unknown enum values, and NaN/Inf before using a frame.
- Keep joint order explicit and validate names at startup. Do not infer the robot
  model from array length.
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
- When a domain-specific variable, field, constant, or state first appears, explain
  its meaning and role. Also document its unit, valid range, zero/sentinel meaning,
  coordinate frame, or lifetime when any of those are not obvious.
- Add short comments before non-obvious groups of local variables so a new reader
  can understand what is being prepared and why. Do not comment self-explanatory
  loop indices, temporary values, or standard-library operations one by one.
- Public data structures should document every field whose meaning cannot be fully
  inferred from its name. Clearly distinguish similar fields such as online/valid,
  generated/applied sequence, and model/calibration identifiers.
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

- Run `./scripts/build.sh` after source or build-system changes.
- Run `./scripts/build.sh --mujoco` for MuJoCo backend changes.
- Use `./scripts/build.sh --clean` when verifying changes to CMake configuration.
- Add or update tests for validation rules and public-interface behavior.
- Tests in M0 must not require network access or a third-party test framework.
- Do not commit `build/`, generated binaries, or `compile_commands.json`.

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
