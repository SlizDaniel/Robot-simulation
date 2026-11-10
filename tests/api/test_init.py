import pytest
import requests
import json
from pathlib import Path

@pytest.fixture
def initialized_simulation(server):
    scene = json.loads(
        Path("scenarios/defaul.json").read_text()
    )

    response = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5
    )

    assert response.status_code == 200, response.text

    return response

def test_correct_initialization(initialized_simulation):
    snapshot = initialized_simulation.json()
    assert snapshot["robot_pose"]["x"] == -8.0
    assert snapshot["simulation_step"] == 0
    assert snapshot["simulation_time"] == 0.0
    assert snapshot["stopped"] is False
    assert snapshot["update"] is None

@pytest.fixture
def initialized_simulation_wrong_data(server):
    response = requests.post(
        f"{server}/api/simulation/init",
        data="{",
        timeout=5
    )

    return response

def test_initialization_with_invalid_data(initialized_simulation_wrong_data):
    assert initialized_simulation_wrong_data.status_code == 400
    snapshot = initialized_simulation_wrong_data.json()
    assert snapshot["error"] != None

@pytest.mark.parametrize("field, value",[
    ("dt", 0),
    ("ray_count", 4),
])
def test_initialization_rejects_invalid_arguments(server, field, value, scene):
    scene["simulation"][field] = value

    response = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5,
    )
    assert response.status_code == 400
    assert "error" in response.json()

def test_simulation_state_after_repeated_initialization(server, scene):
    response = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5,
    )
    assert response.status_code == 200

    make_step = requests.post(
        f"{server}/api/simulation/step",
        json=scene,
        timeout=5,
    )
    assert make_step.status_code == 200
    assert make_step.json()["simulation_step"] == 1

    response_reinitialization = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5,
    )
    body = response_reinitialization.json()
    assert body["simulation_step"] == 0
    assert body == response.json()
    assert response_reinitialization.status_code == 200

def test_failed_reinitialization_preserves_state(server, scene):
    response = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5,
    )
    assert response.status_code == 200

    make_step = requests.post(
        f"{server}/api/simulation/step",
        json=scene,
        timeout=5,
    )
    assert make_step.status_code == 200
    assert make_step.json()["simulation_step"] == 1

    initial_state = requests.get(
        f"{server}/api/simulation/state",
        timeout=5,
    )
    assert initial_state.status_code == 200
    scene["simulation"]["ray_count"] = 4

    response_wrong_scene = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5,
    )
    assert response_wrong_scene.status_code == 400
    assert "error" in response_wrong_scene.json()

    after_state = requests.get(
        f"{server}/api/simulation/state",
        timeout=5,
    )
    assert after_state.status_code == 200
    assert after_state.json() == initial_state.json()

def test_initialization_rejects_missing_robot(server, scene):
    del scene["robot"]

    response = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5,
    )

    assert response.status_code == 400
    assert "error" in response.json()
