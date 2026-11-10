import math

import pytest
import requests


def test_config_reports_active_scene_and_mounted_sensors(server, scene, fail_scene):
    response_init = requests.post(
        f"{server}/api/simulation/init", json=scene, timeout=5
    )
    assert response_init.status_code == 200

    response_config = requests.get(f"{server}/api/simulation/config", timeout=5)
    assert response_config.status_code == 200
    assert response_config.headers["Content-Type"].startswith("application/json")
    config = response_config.json()
    for key in ("environment", "robot", "obstacles", "simulation", "controller"):
        assert config[key] == scene[key]

    sensors = config["sensors"]
    assert [sensor["id"] for sensor in sensors] == [0, 1, 2]
    assert [sensor["role"] for sensor in sensors] == [
        "front_left", "front", "front_right"
    ]
    for sensor, theta in zip(sensors, (math.pi / 4, 0.0, -math.pi / 4)):
        assert sensor["relative_pose"] == {
            "x": 0.0, "y": 0.0, "theta": pytest.approx(theta)
        }
        assert sensor["min_distance"] == 0.0
        assert sensor["max_distance"] == 20.0
        assert sensor["field_of_view"] == pytest.approx(math.pi / 2)

    response_step = requests.post(f"{server}/api/simulation/step", timeout=5)
    assert response_step.status_code == 200
    results = response_step.json()["update"]["raycast_results"]
    assert [result["sensor_id"] for result in results] == [
        sensor["id"] for sensor in sensors
    ]
    assert requests.get(f"{server}/api/simulation/config", timeout=5).json() == config

    response_reset = requests.post(f"{server}/api/simulation/reset", timeout=5)
    assert response_reset.status_code == 200
    assert requests.get(f"{server}/api/simulation/config", timeout=5).json() == config

    response_reinit = requests.post(
        f"{server}/api/simulation/init", json=fail_scene, timeout=5
    )
    assert response_reinit.status_code == 200
    new_config = requests.get(f"{server}/api/simulation/config", timeout=5).json()
    assert new_config["environment"] == fail_scene["environment"]
    assert new_config["robot"] == fail_scene["robot"]
    assert new_config["sensors"] == sensors
