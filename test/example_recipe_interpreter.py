from test.example_recipe import recipe
from datetime import datetime


def interpret_recipe(recipe, current_time, start_time, variable):
    """
    Recipe Interpreter should read a recipe, current_time, start_time, variable and return a value.
       Determine the time since the beginning of the recipe.
       Determine what is the current step
       Calculate the remaining time left in this step.
       Look up the value within that step for that variable.
    """

    # Returns a list of the phases and step durations  [(duration_of_phase_1, duration_of_step_1), (duration_of_phase_2, duration_of_step_2), etc]
    duration_of_phases_steps = calc_duration_of_phases_steps(recipe['phases'])
    current_phase_number, duration_in_step = calc_phase_and_time_remaining(duration_of_phases_steps,
                                                                           current_time,
                                                                           start_time)
    current_phase = recipe['phases'][current_phase_number]
    variable_step_data = current_phase['step'][variable]
    value = determine_value_for_step(variable_step_data, duration_in_step)
    return value


def determine_value_for_step(variable_step_data, duration_in_step):
    """
    variable_step_data = [{"start_time": 0, "end_time": 6, "value": 20},
                          {"start_time": 6, "end_time": 18, "value": 23},
                          {"start_time": 18, "end_time": 24, "value": 19}]
    duration_in_step = 15
    Given the time within a step, what value is expected.

    """

    for row in variable_step_data:
        if row['start_time'] <= duration_in_step <= row['end_time']:
            return row['value']


def calculate_max_duration_from_step(step):
    """
    Determines the total duration of this step. Normally it is 24 hours.
       Could add other validation steps here as well.
       Convert to numpy and use argmax
    """
    max_time = 0
    for variable, start_end_times in step.items():
        for data in start_end_times:
            if data['start_time'] > data['end_time']:
                raise Exception("Start_time is after end time.")
            elif max_time < max(data['start_time'], data['end_time']):
                max_time = max(data['start_time'], data['end_time'])
    return max_time


def calc_duration_of_phases_steps(phases):
    """
    Returns a list with the duration of the step and the entire phase
    """
    duration_of_phases_steps = []
    for phase in phases:
        cycles = phase['cycles']
        max_duration = calculate_max_duration_from_step(phase['step'])
        duration_of_phases_steps.append((cycles*max_duration, max_duration))
    return duration_of_phases_steps


def calc_phase_and_time_remaining(duration_of_phases_steps, current_time, start_time):
    """
    duration_of_phases_steps == [(336, 24), (480, 24), (168, 24)]
    current_time : datetime : UTC of the current time
    start_time : datetime : UTC time the recipe started
    return: current_phase_number, duration_in_phase
    """
    time_since_start = current_time - start_time
    time_since_start = time_since_start.total_seconds() / 3600

    time_remaining = time_since_start
    for i, (total_duration, step_duration) in enumerate(duration_of_phases_steps):
        if time_remaining > total_duration:
            time_remaining -= total_duration
        else:
            duration_in_phase = time_remaining % step_duration
            current_phase_number = i
            break
    return current_phase_number, duration_in_phase


def test_calc_phase_and_time_remaining():
    duration_of_phases_steps = [(336, 24), (480, 24), (168, 24)]
    start_time = datetime.strptime("2017-04-17 14:00", "%Y-%m-%d %H:%S")
    current_time = datetime.strptime("2017-04-18 20:00", "%Y-%m-%d %H:%S")
    current_phase_number, duration_in_step = calc_phase_and_time_remaining(duration_of_phases_steps,
                                           current_time, start_time)
    assert (current_phase_number, duration_in_step) == (0, 20)

    start_time = datetime.strptime("2017-04-17 14:00", "%Y-%m-%d %H:%S")
    current_time = datetime.strptime("2017-04-18 03:00", "%Y-%m-%d %H:%S")
    current_phase_number, duration_in_step = calc_phase_and_time_remaining(duration_of_phases_steps,
                                           current_time, start_time)
    print(current_phase_number, duration_in_step)
    assert (current_phase_number, duration_in_step) == (1, 3)


def test_calculate_max_duration_from_step():
    step = { "air_temperature": [{"start_time": 0, "end_time": 6, "value": 20},
                                  {"start_time": 6, "end_time": 18, "value": 23},
                                  {"start_time": 18, "end_time": 24, "value": 19}],
              "nutrient_flora_duo_a": [{"start_time": 0, "end_time": 6, "value": 5},
                                       {"start_time": 6, "end_time": 18, "value": 2},
                                       {"start_time": 18, "end_time": 24, "value": 5}],
              "nutrient_flora_duo_b": [{"start_time": 0, "end_time": 6, "value": 2}],
              "light_illuminance": [{"start_time": 0, "end_time": 6, "value": 4},
                                    {"start_time": 18, "end_time": 24, "value": 3}],
        }
    max_time = calculate_max_duration_from_step(step)
    assert max_time == 24


def test_calc_duration_of_phases_steps():
    phases = [    # Previously operations,  Needs to be an ordered dictionary.
          { "name": "early",
            "cycles": 14,    # Add check for duration of a step to be a total of 24 hours. (Not a necessarity but valuable for consistency/simplicity)
            "step": { "air_temperature": [{"start_time": 0, "end_time": 6, "value": 20},
                                          {"start_time": 6, "end_time": 18, "value": 23},
                                          {"start_time": 18, "end_time": 24, "value": 19}],
                      "nutrient_flora_duo_a": [{"start_time": 0, "end_time": 6, "value": 5},
                                               {"start_time": 6, "end_time": 18, "value": 2},
                                               {"start_time": 18, "end_time": 24, "value": 5}],
                      "nutrient_flora_duo_b": [{"start_time": 0, "end_time": 6, "value": 2}],
                      "light_illuminance": [{"start_time": 0, "end_time": 6, "value": 4},
                                            {"start_time": 18, "end_time": 24, "value": 3}],
                },
          },
          { "name": "middle",
            "cycles": 20,
            "step": { "air_temperature": [{"start_time": 0, "end_time": 6, "value": 20},
                                          {"start_time": 6, "end_time": 18, "value": 23},
                                          {"start_time": 18, "end_time": 24, "value": 19}],
                      "nutrient_flora_duo_a": [{"start_time": 6, "end_time": 18, "value": 1.4}],
                      "nutrient_flora_duo_b": [{"start_time": 6, "end_time": 18, "value": 0.7}],
                      "light_illuminance": [{"start_time": 6, "end_time": 18, "value": 1}],
                },
          },
          { "name": "late",
            "cycles": 7,
            "step": { "air_temperature": [{"start_time": 6, "end_time": 24, "value": 23}],
                    },
          }
        ]
    duration_of_phases_steps = calc_duration_of_phases_steps(phases)
    assert duration_of_phases_steps == [(336, 24), (480, 24), (168, 24)]


def test_determine_value_for_step():
    variable_step_data = [{"start_time": 0, "end_time": 6, "value": 20},
                          {"start_time": 6, "end_time": 18, "value": 23},
                          {"start_time": 18, "end_time": 24, "value": 19}]
    duration_in_step = 15
    value = determine_value_for_step(variable_step_data, duration_in_step)
    assert value == 23
    duration_in_step = 0
    value = determine_value_for_step(variable_step_data, duration_in_step)
    assert value == 20


def test_interpret_recipe():
    """
    Recipe Interpreter should read a recipe, current_time, start_time, variable and return a value.
    """
    current_time = datetime.strptime("2017-04-18 20:00", "%Y-%m-%d %H:%S")
    start_time = datetime.strptime("2017-04-17 14:00", "%Y-%m-%d %H:%S")   # Need to figure out how to account for the start time not being midnight.
    variable = "air_temperature"
    value = interpret_recipe(recipe, current_time, start_time, variable)
    assert value == 19
