from nodes.image_persistence import get_camera_variables

def test_get_camera_variables():
    camera_variables = get_camera_variables()
    print(camera_variables)
    assert False