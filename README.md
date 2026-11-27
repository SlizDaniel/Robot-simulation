# Robot Simulator

A C++17 simulator for a rectangular mobile robot in a 2D world. It includes a reusable library, a terminal demo, an HTTP server, and a browser scene editor. The robot can drive straight or along an arc, sense obstacles and walls with simulated distance sensors, and use a reactive obstacle avoidance controller.

## Build

You need a C++17 compiler and CMake 3.16 or newer. CMake downloads nlohmann/json, cpp-httplib, and (when tests are enabled) GoogleTest during the first configuration.

From the repository root:

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --parallel
```

This builds the `robot_simulator` library, `robot_simulator_app` terminal demo, `robot_simulator_server`, and `robot_simulator_tests`. Use `-DBUILD_TESTING=OFF` if you do not need the C++ tests.

## Run the demos

Terminal demo:

```bash
./build/robot_simulator_app
```

It runs a fixed scene with three front distance sensors and the obstacle avoidance controller, printing each step and sensor reading. It takes no command line arguments.

Browser demo: start the server **from the repository root**, then open <http://127.0.0.1:8080/>.

```bash
./build/robot_simulator_server
```

The browser UI lets you edit or import a scene, initialize it, step or start the simulation, and inspect sensor readings and the trajectory. The server holds one session in memory; restarting it clears that session. The UI can export a scene to JSON for later import. See [frontend/README.md](frontend/README.md) for UI details.

## Run the tests

**C++:**

```bash
ctest --test-dir build --output-on-failure
```

`SimulationTests.StopsRobotWhenStepPassesThroughObstacle` is an intentionally failing TDD test: the current collision check misses an obstacle crossed between two step endpoints. Until that behavior is fixed, run the remaining C++ tests with:

```bash
ctest --test-dir build -E StopsRobotWhenStepPassesThroughObstacle --output-on-failure
```

**Python API:** You need Python 3.10 or newer, [uv](https://docs.astral.sh/uv/), a built `robot_simulator_server`, and a free `127.0.0.1:8080` port. Run from the repository root:

```bash
uv run pytest
```

`pyproject.toml` declares the test dependencies and configures pytest to run `tests/api` without unrelated system plugins. `uv.lock` records dependency versions. uv creates and updates `.venv` automatically; the API tests start and stop their own server.

**Browser:** You need Node.js, npm, and Playwright's Chromium. Build the C++ server first and leave port `127.0.0.1:8080` free for the real-server test.

```bash
cd frontend
npm ci
npx playwright install chromium
npm test
npm run test:e2e
```

`npm test` checks the UI with controlled API responses; `npm run test:e2e` starts its own C++ server and exercises the real endpoints.

## Simulation model

- `Pose{x, y, theta}` gives an object's **center** and heading. Angles and angular velocities use radians.
- `Rectangle{width, length}` has its **length along local X** and width along local Y. The environment is centered at `(0, 0)`, spanning `[-length/2, length/2]` on X and `[-width/2, width/2]` on Y.
- Use consistent length and time units throughout a scene. `Velocity` contains linear speed in length/time and angular speed in radians/time.
- `Simulation::update()` predicts the next pose, checks its corners against the world boundary and obstacles, then either accepts the step or stops the robot at its previous pose. Touching an obstacle counts as a collision; touching the world boundary is allowed. Accepted poses are recorded in the robot's trajectory.
- `Simulation::runSteps(n)` stops after `n` accepted updates or the first rejected update. `SimulationUpdate` reports whether a step was accepted, its stop reason, and one result per attached sensor.
- Distance sensors cast a positive, odd number of rays, including a central ray. They report the nearest obstacle or world boundary and distinguish in-range detections from out-of-range echoes. Sensor readings are deterministic and contain no simulated noise.
- `Environment::addObstacle()` assigns each stored obstacle an ID equal to its insertion index. `Obstacle::getObstacleId()` returns an optional ID; newly constructed obstacles have no ID. IDs are local to each environment.

The collision check currently tests the **end pose only**. A large step can pass through an obstacle and still be accepted; the failing C++ test above demonstrates this. The simulator does not calculate the time or point of contact. The obstacle avoidance controller reacts to sensor readings but does not plan a route.

Minimal library usage:

```cpp
#include <robot_simulation/environment.hpp>
#include <robot_simulation/robot.hpp>
#include <robot_simulation/simulation.hpp>

using namespace robot_simulation;
Environment world(Rectangle{20.0, 30.0});
Robot robot(Pose{0.0, 0.0, 0.0}, Velocity{2.0, 0.0}, Rectangle{2.0, 4.0});
Simulation simulation(0.5, robot, world);
SimulationUpdate result = simulation.update();
```

`Simulation` holds references to the robot and environment, so both must outlive it.

Public headers are in `include/robot_simulation/`, implementations in `src/`, C++ tests in `tests/`, and API tests in `tests/api/`. Example scene JSON is in [`scenarios/defaul.json`](scenarios/defaul.json) (the filename is spelled `defaul` in this repository).

## HTTP API

The server listens on `127.0.0.1:8080` and serves both the browser UI and these endpoints:

| Method and path | Purpose |
| --- | --- |
| `GET /api/health` | Server health |
| `POST /api/simulation/init` | Create or replace the session from scene JSON |
| `GET /api/simulation/config` | Current scene and attached sensor descriptions |
| `GET /api/simulation/state` | Current simulation snapshot |
| `POST /api/simulation/step` | Attempt one step |
| `POST /api/simulation/reset` | Restore the initialized scene |

For example, with the server running, initialize it from the included scene:

```bash
curl -H 'Content-Type: application/json' --data-binary @scenarios/defaul.json http://127.0.0.1:8080/api/simulation/init
```

`/config`, `/state`, `/step`, and `/reset` return HTTP 409 until a scene is initialized. After a rejected step, `/step` returns 409 until reset or reinitialization. Sensor IDs in step results match the descriptions returned by `/config`.

## License

This repository does not currently include a license file.
