from nodes.recipe_handler import get_constants


def test_get_constants():
    valid_variables, recipe_start, recipe_end = get_constants()
    assert 10 < len(valid_variables) < 1000
    assert all([type(name) == str for name in valid_variables])
