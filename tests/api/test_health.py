import requests

def test_health_endpoint(server):
    response = requests.get(
        f"{server}/api/health",
        timeout=5
    )
    assert response.status_code == 200
    assert response.json()["status"] == "ok"
    assert response.json()["backend"] == "cpp"