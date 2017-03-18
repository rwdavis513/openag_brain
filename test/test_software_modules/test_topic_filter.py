# from openag_brain.software_modules.topic_filter import EWMA
import numpy as np
import unittest
import random


class EWMA(object):
    """
    Calculate the Exponentially Weighted Moving Average (EWMA) for an input variable.
    EWMAi = alpha * Xi + (1 - alpha) * EWMAi-1
    Params:
        a : Alpha is the dapening coefficient. The lower the value the less damped. Should be between 0 and 1
    """
    def __init__(self, a, filter_results=True):
        self.a = a
        self.average = None
        self.IQR = None   # IQR = InterQuartile Range, https://simple.wikipedia.org/wiki/Interquartile_range
        self.recent_values = []
        self.IQR_MULTIPLE = 5
        self.filter_results = filter_results
        self.NUM_INITIAL_VALUES = 20

    def __call__(self, sample):
        # Update the average
        if self.average is None:
            self.average = sample
            return
        self.shift_recent_values(sample)
        self.calc_iqr(sample)
        if len(self.recent_values) == self.NUM_INITIAL_VALUES - 1 and self.filter_results:
            self.recalculate_ewma_excluding_fliers() # Uses median as a starting point
        if not self.is_outlier(sample):
            self.average = self.a * sample + (1 - self.a) * self.average
        else:
            print("Outlier Found! {:3f}".format(sample))

    def shift_recent_values(self, sample):
        self.recent_values.append(sample)
        if len(self.recent_values) > self.NUM_INITIAL_VALUES:
            self.recent_values.pop(0)

    def calc_iqr(self, sample):
        if self.IQR is None:
            self.IQR = self.average + 1    # Not the real IQR, but will work to ensure first samples aren't filtered out
                                           # + 1 in case the average is zero
            return
        if len(self.recent_values) > 10:
            self.IQR = (np.percentile(self.recent_values, 75) - np.percentile(self.recent_values, 25))/3
            # print("IQR: {:f}".format(self.IQR))

    def recalculate_ewma_excluding_fliers(self):
        """
        Recalculate ewma once there is enough data points to establish a reasonable confidence of distribution width
        Assumes calc_iqr was already run on the previous calls.
        :return:
        """
        self.average = np.median(self.recent_values)
        for value in self.recent_values:
            if not self.is_outlier(value):
                self.average = self.a * value + (1 - self.a) * self.average

    def is_outlier(self, sample):
        """
        Filter out values outside the Interquartile range (IQR)
        :param sample:
        :return:
        """
        if not self.filter_results:
            return False
        lower_limit = self.average - self.IQR_MULTIPLE*self.IQR
        upper_limit = self.average + self.IQR_MULTIPLE*self.IQR
        # print(list(map(lambda x: "{:0.1f}".format(x), (lower_limit, self.average, upper_limit))))
        return not (lower_limit <= sample <= upper_limit)


def compare_function_output(func, input, output_expected):
    results = []
    avg = []
    for sample, sample_avg_expected in zip(input, output_expected):
        func(sample)
        results.append(round(func.average, 3) == sample_avg_expected)
        avg.append(round(func.average, 3))
        if not results[-1]:
            print("{:4f} <> {:4f}".format(func.average, sample_avg_expected))
    print(avg)
    return results


class TestEWMA(unittest.TestCase):
    """
    Sometimes the sensors return erroneous values. These should get filtered out and not reported. Need to verify it
    works for all the appropriate use cases.
        Case 1: Steady stream of valid inputs to establish a baseline, then erroneous values over time
        Case 2: X percent erroneous values in the beginning
        Case 3: Extreme erroneous values
        Case 4: Steady stream of valid inputs, then a shift to all erroneous values at one time
        Case 5: Rapid shift in the input
    :return:
    """
    def test_ewma_unfiltered(self):
        """
        Tests the EWMA class to verify it calculates the exponentially weighted moving average for each new data point.
        :return: None
        """
        print("test_ewma_unfiltered\n------------------")
        ewma_func = EWMA(0.2, filter_results=False)
        data_stream = [3, 2, 6, 4, 3, 4, 7, 4, 8, 5, 4, 5, 3, 2, 3, 4, 2, 23, 4, 5, 4, 5, 2, 4, -90, 5, 7]
        ewma_unfiltered = [3, 2.8, 3.44, 3.552, 3.442, 3.553, 4.243, 4.194, 4.955, 4.964, 4.771, 4.817, 4.454, 3.963,
                           3.77, 3.816, 3.453, 7.362, 6.69, 6.352, 5.882, 5.705, 4.964, 4.771, -14.183, -10.346, -6.877]
        assert all(compare_function_output(ewma_func, data_stream, ewma_unfiltered))

    def test_ewma_filter_outliers(self):
        """
        Steady stream of valid inputs to establish a baseline, then erroneous values over time
        :return:
        """
        print("test_filter_outliers\n------------------")
        ewma_func = EWMA(0.2, filter_results=True)
        data_stream = [3, 2, 6, 4, 3, 4, 7, 4, 8, 5, 4, 5, 3, 2, 3, 4, 2, 23, 4, 7, 2, 45, -90, 23, 45]
        ewma_expected = [3, 2.8, 3.44, 3.552, 3.442, 3.553, 4.243, 4.194, 4.955, 4.964, 4.771, 4.817, 4.454, 3.963,
                         3.77, 3.816, 3.453, 3.453, 3.562, 4.814, 4.251, 4.251, 4.251]

        assert all(compare_function_output(ewma_func, data_stream, ewma_expected))

    def insert_random_fliers(self, data_stream):
        data = data_stream.copy()
        index_max = len(data) - 1
        # print(list(map(lambda x: "{:0.3f}".format(x), data)))
        outlier_locations = []
        for outlier in np.random.uniform(-1000, 1000, size=10):
            loc = random.randint(0, index_max)
            data.insert(loc, outlier)
            outlier_locations.append(loc)
        return data, outlier_locations

    def within_3sigma(self, value, mean, stdev):
        """
        Checks if the parameter is within 3 standard deviations of the mean
        :param value:
        :param mean:
        :param stdev:
        :return:
        """
        return (mean - 3*stdev) <= value <= (mean + 3*stdev)

    def test_ewma_filter_outliers_random_input(self):
        """
        Test random inputs with misc outliers. Creates a random normal distribution with a random mean and
        random standard deviation. Then 10 outliers are inserted into the list.
        This is done to simulate various inputs. The downside is that the fails could be intermittant. If this ever
        fails recommend saving those settings to repeat the issue.
        :return:
        """
        mean = random.randint(-100, 100)
        stdev = random.randint(5, 50)
        data_stream = list(map(lambda x: round(x, 3), np.random.normal(mean, stdev, size=100)))
        data_stream, outlier_locations = self.insert_random_fliers(data_stream)
        self.ewma_compare_results(data_stream, mean, stdev, outlier_locations)

    def ewma_compare_results(self, data_stream, mean, stdev, outlier_locations):
        ewma_func = EWMA(0.2, filter_results=True)
        ewma_unfiltered = EWMA(0.2, filter_results=False)
        print(list(map(lambda x: "{:0.3f}".format(x), data_stream)))
        for _, sample in enumerate(data_stream):
            ewma_func(sample)
            ewma_unfiltered(sample)
            if _ > 20:  # Only filter the first 10 results
                if not self.within_3sigma(ewma_func.average, mean, stdev):
                    print("Mean: {:f}, Stdev: {:3f}, ewma: {:3f}, sample: {:d}, {:3f}".format(mean, stdev,
                                                                                        ewma_func.average, _, sample))
                    assert False
                if _ in outlier_locations and not self.within_3sigma(sample, mean, stdev):
                    # if it is an outlier then ewma unfiltered should be different than filtered
                    print("filtered: {:0.3f}  unfiltered: {:0.3f}".format(ewma_func.average, ewma_unfiltered.average))
                    print(sample)
                    print(_)
                    assert round(ewma_func.average, 3) != round(ewma_unfiltered.average, 3)

    def test_ewma_filter(self):
        data_stream = [-953.060, 82.414, 75.627, 101.725, 92.747, 86.824, 106.127, 104.136, 63.223, 97.402,
                       94.617, 85.263, 87.337, 90.000, 85.662, 88.297, 74.880, 110.812, 70.485, 85.928, 508.925,
                       78.585, 100.868, 107.894, 112.247, 82.002, 101.238, 62.958, 96.318, 90.225, 86.997,
                       93.158, 110.167, 191.598, 102.279, 90.973, 78.831, -63.813, 84.553, 104.157, 80.185,
                       82.832, 90.811, 69.352, 101.181, 34.019, 97.826, 96.848, 82.841, 96.128, 105.761, 96.689,
                       122.107, 78.107, 81.107, 76.118, 102.961, 99.852, 89.967, 98.029, 78.895, 464.843,
                       89.245, 76.949, 105.847, 94.632, 95.001, 82.801, 381.858, 75.820, 108.987, 117.088,
                       93.587, 89.832, 89.401, 87.222, 92.310, 110.758, 86.953, 87.652, 79.859, 97.795, 100.966,
                       109.980, 95.535, 89.615, 69.682, 81.639, 81.679, -156.417, 94.325, 77.825, 851.553,
                       82.412, 88.118, 24.525, 93.778, 108.072, 95.802, 113.294, 96.925, 94.488, 91.306, 82.509,
                       92.780, 82.017, 76.723, 85.777, 96.075, 82.439]
        mean, stdev = 94, 12
        # ewma: -953.059732
        self.ewma_compare_results(data_stream, mean, stdev, [0, 38, 88, 91])

if __name__ == '__main__':
    unittest.main()
