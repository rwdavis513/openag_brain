from nodes.image_persistence import get_camera_variables


def test_get_camera_variables():
    camera_variables = get_camera_variables()
    print(camera_variables)
    assert camera_variables == ({'units': 'png', 'name': 'aerial_image', 'groups': 'camera',
                                 'description': 'Image from above the tray looking down on the plants'},
                                {'units': 'png', 'name': 'frontal_image', 'groups': 'camera',
                                 'description': 'Image from in front of the tray looking towards the plants'})
