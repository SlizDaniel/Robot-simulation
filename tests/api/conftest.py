import subprocess
import time
from pathlib import Path
import json
import pytest
import requests


BASE_URL = "http://127.0.0.1:8080"


@pytest.fixture(scope="session")
def server():
    process = subprocess.Popen(["./build/robot_simulator_server"])
    time.sleep(1)

    try:
        response = requests.get(f"{BASE_URL}/api/health", timeout=2)
        response.raise_for_status()
        assert response.status_code == 200

        state_response = requests.get(
            f"{BASE_URL}/api/simulation/state",
            timeout=5
        )
        assert state_response.status_code == 409
        assert state_response.json()["error"] != None

        yield BASE_URL

    except requests.RequestException as error:
        pytest.fail(f"Server failed: {error}")

    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)

@pytest.fixture
def scene():
    conf = json.loads(
        Path("scenarios/defaul.json").read_text()
    )
    return conf

@pytest.fixture
def fail_scene():
    conf = json.loads(
        Path("scenarios/out_of_environment.json").read_text()
    )
    return conf