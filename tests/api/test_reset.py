import json
import pytest
import requests

def test_reset_after_accepted_step(server, scene):
    response_init = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5,
    )
    assert response_init.status_code == 200
    initial_snapshot = response_init.json()

    response_step = requests.post(
        f"{server}/api/simulation/step",
        timeout=5,
    )
    assert response_step.status_code == 200
    assert response_step.json()["simulation_step"] == 1

    response_reset = requests.post(
        f"{server}/api/simulation/reset",
        timeout=5,
    )
    assert response_reset.status_code == 200
    assert response_reset.json() == initial_snapshot

    response_state = requests.get(
        f"{server}/api/simulation/state",
        timeout=5,
    )
    assert response_state.status_code == 200
    assert response_state.json() == initial_snapshot


def test_reset_after_stopped_step(server, fail_scene):
    response_init = requests.post(
        f"{server}/api/simulation/init",
        json=fail_scene,
        timeout=5,
    )
    assert response_init.status_code == 200
    initial_snapshot = response_init.json()

    response_step = requests.post(
        f"{server}/api/simulation/step",
        timeout=5,
    )
    assert response_step.status_code == 200
    assert response_step.json()["update"]["step_accepted"] is False
    assert response_step.json()["stopped"] is True

    response_reset = requests.post(
        f"{server}/api/simulation/reset",
        timeout=5,
    )
    assert response_reset.status_code == 200
    assert response_reset.json() == initial_snapshot
    assert response_reset.json()["stopped"] is False


def test_reset_uses_last_loaded_scene(server, scene, fail_scene):
    response_first_init = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5,
    )
    assert response_first_init.status_code == 200

    response_second_init = requests.post(
        f"{server}/api/simulation/init",
        json=fail_scene,
        timeout=5,
    )
    assert response_second_init.status_code == 200
    assert response_second_init.json() != response_first_init.json()

    response_step = requests.post(
        f"{server}/api/simulation/step",
        timeout=5,
    )
    assert response_step.status_code == 200
    assert response_step.json()["stopped"] is True

    response_reset = requests.post(
        f"{server}/api/simulation/reset",
        timeout=5,
    )
    assert response_reset.status_code == 200
    assert response_reset.json() == response_second_init.json()