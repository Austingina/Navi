import requests

def test_get_pose_names():
    url = "http://localhost:8000/tools/get_pose_names"
    resp = requests.post(url, json={})
    print("get_pose_names:", resp.status_code)
    print("Raw response:", repr(resp.text))
    try:
        print("JSON:", resp.json())
    except Exception as e:
        print("JSON decode error:", e)


def test_set_goal(target):
    url = "http://localhost:8000/tools/set_goal"
    resp = requests.post(url, json={"target": target})
    print("set_goal:", resp.status_code, resp.json())

def test_get_status():
    url = "http://localhost:8000/tools/get_status"
    resp = requests.post(url, json={})
    print("get_status:", resp.status_code, resp.json())

def test_get_nearest_point():
    url = "http://localhost:8000/tools/get_nearest_point"
    resp = requests.post(url, json={})
    print("get_nearest_point:", resp.status_code, resp.json())

if __name__ == "__main__":
    # test_get_pose_names()
    # 替换为你的目标点名称
    # test_set_goal("狗巢")
    test_get_status()
    # test_get_nearest_point()