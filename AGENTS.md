# Repository Instructions

## Project

This repository contains a C++17 2D mobile robot simulator. It consists of the
`robot_simulator` library, a console demonstration application, and GoogleTest
unit tests.

Keep the simulation deterministic unless a task explicitly introduces
controlled randomness.

## Repository Layout

- `include/robot_simulation/` contains public headers.
- `src/` contains library implementations.
- `tests/` contains GoogleTest tests.
- `app/` contains the console demo.
- `README.md` contains user-facing documentation.

Read only the files relevant to the requested task. Do not require a complete
repository review for a small, isolated change.

## Build and Test

Configure and build with:

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --parallel
```

Run the complete test suite with:

```bash
ctest --test-dir build --output-on-failure
```

The local tests use only local simulator data and are safe to run without
additional approval. For code changes, run tests proportional to the scope and
risk of the change. Documentation-only changes do not require a build unless
they modify documented commands or behavior.

## Domain Conventions

- Angles and angular velocities use radians.
- `Pose` describes an object's center and orientation.
- `Rectangle::length` lies along the object's local X axis.
- `Rectangle::width` lies along the object's local Y axis.
- The environment is centered at `(0, 0)`.
- Touching an obstacle counts as a collision.
- Touching the environment boundary is allowed.
- Length and time values must use a consistent unit system.

Do not change these semantics unintentionally. If a task explicitly changes a
domain rule, update the affected implementation, tests, and documentation.

## Distance Sensor Conventions

The following rules describe the current tested sensor model, not permanent
restrictions on future sensor designs:

- `ray_count` must be positive and odd, ensuring a central ray is present.
- A one-ray scan uses the sensor's central direction.
- `DetectionType::OBJECTDETECTED` corresponds to `object_detected == true`.
- Other detection states correspond to `object_detected == false`.

When intentionally changing sensor sampling or detection semantics, update the
relevant tests and documentation instead of preserving these rules blindly.

## C++ Conventions

- Use C++17 features only.
- Name types using `PascalCase` and new methods or functions using `camelCase`.
- Pass read-only objects by `const&` when a copy is unnecessary.
- Prefer returning computed values by value.
- Return by `const&` only when referring to an existing object with a guaranteed
  sufficient lifetime.
- Avoid unnecessary copies of expensive objects.
- Do not add top-level `const` to values returned by value.
- Follow the style of the touched component and avoid broad naming or formatting
  changes unless explicitly requested.

## Collaboration

This is also an educational project.

- When asked for an explanation or code review, prioritize understanding over
  rewriting code.
- Do not modify files when the user asks only a conceptual question or requests
  feedback.
- When suggesting a design decision, briefly explain important trade-offs.
- Do not implement additional features that were not requested.
- When asked to implement something, provide a complete working solution rather
  than deliberately withholding code.

## Change Discipline

- Match the scope of changes to the user's request.
- Avoid modifications unrelated to the task and speculative abstractions.
- Architectural improvements may be proposed when they provide a meaningful
  benefit, but do not implement them beyond the requested scope.
- Preserve existing user changes and unrelated worktree modifications.
- Do not change public APIs unintentionally.
- Add or update tests when observable behavior changes.
- Do not create commits, tags, or releases unless explicitly requested.

## Test Conventions

- Put geometry-level ray intersection tests in `tests/geometry_tests.cpp`.
- Put complete sensor behavior tests in `tests/distance_sensor_tests.cpp`.
- Put robot ownership and sensor attachment tests in `tests/robot_tests.cpp`.
- Use the shared tolerance from `tests/helpers.hpp` for floating-point checks.
- For `RaycastResult`, test the detection flag, detection type, and distance
  together when applicable.
- Prefer extending an existing scenario over adding a duplicate test.

## Completion

- Verify modified code using the relevant build and tests.
- Inspect the final diff for unintended changes.
- Summarize the changes and report the verification results.
- Never claim that tests passed if they were not executed.
- Report any remaining limitations directly related to the task.
