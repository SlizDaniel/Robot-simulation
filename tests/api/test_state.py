import pytest
import requests
import json
from pathlib import Path

def test_after_init_state(server, scene):
    response = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5
    )
    assert response.status_code == 200

    state_response = requests.get(
        f"{server}/api/simulation/state",
        timeout=5
    )
    assert state_response.status_code == 200
    assert response.json() == state_response.json()
    
def test_state_doesnt_make_step(server, scene):
    response = requests.post(
        f"{server}/api/simulation/init",
        json=scene,
        timeout=5
    )
    assert response.status_code == 200

    step_response = requests.post(
        f"{server}/api/simulation/step",
        timeout=5
    )
    assert step_response.status_code == 200
    assert step_response.json()["simulation_step"] == 1

    state1_response = requests.get(
        f"{server}/api/simulation/state",
        timeout=5
    )
    assert state1_response.status_code == 200
    assert state1_response.json()["simulation_step"] == 1
    assert state1_response.json() == step_response.json()

    state2_response = requests.get(
        f"{server}/api/simulation/state",
        timeout=5
    )
    assert state2_response.status_code == 200
    assert state2_response.json()["simulation_step"] == 1
    assert state1_response.json() == state2_response.json()