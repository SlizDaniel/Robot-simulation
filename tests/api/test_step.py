import json
import pytest
import requests

def test_accepted_step(server, scene):
    response_init = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5,
    )
    assert response_init.status_code == 200
    assert response_init.json()["simulation_step"] == 0
    dt = scene["simulation"]["dt"]

    response_step = requests.post(
        f"{server}/api/simulation/step",
        timeout=5,
    )
    assert response_step.status_code == 200
    assert response_step.json()["simulation_step"] == 1
    assert response_step.json()["simulation_time"] == dt
    assert response_step.json()["update"]["step_accepted"] is True
    assert not response_step.json()["stopped"]

def test_two_steps_accepted(server, scene):
    response_init = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5,
    )
    assert response_init.status_code == 200
    assert response_init.json()["simulation_step"] == 0
    dt = scene["simulation"]["dt"]

    response_step = requests.post(
        f"{server}/api/simulation/step",
        timeout=5,
    )
    assert response_step.status_code == 200
    assert response_step.json()["simulation_step"] == 1
    assert response_step.json()["simulation_time"] == dt
    assert response_step.json()["update"]["step_accepted"] is True
    assert not response_step.json()["stopped"]

    response_step2 = requests.post(
        f"{server}/api/simulation/step",
        timeout=5,
    )
    assert response_step2.status_code == 200
    assert response_step2.json()["simulation_step"] == 2
    assert response_step2.json()["simulation_time"] == pytest.approx(dt*2)
    assert response_step2.json()["update"]["step_accepted"] is True
    assert not response_step2.json()["stopped"]

def test_rejected_step_stops_simulation(server, fail_scene):
    response_init = requests.post(
        f"{server}/api/simulation/init",
        json=fail_scene,
        timeout=5,
    )
    assert response_init.status_code == 200
    assert response_init.json()["simulation_step"] == 0
    response_step = requests.post(
        f"{server}/api/simulation/step",
        timeout=5,
    )
    assert response_step.status_code == 200
    assert response_step.json()["simulation_step"] == 0
    assert response_step.json()["simulation_time"] == 0.0
    assert response_step.json()["update"]["step_accepted"] is False
    assert response_step.json()["update"]["stop_reason"] == "OutOfEnvironment"
    assert response_step.json()["stopped"] is True

    response_step2 = requests.post(
        f"{server}/api/simulation/step",
        timeout=5,
    )
    assert response_step2.status_code == 409
    assert "error" in response_step2.json()
