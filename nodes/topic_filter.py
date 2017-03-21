#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Float64
from openag.var_types import EnvVar, GROUP_ENVIRONMENT, WATER_LEVEL_HIGH

# Filter a list of environmental variables that are specific to environment
# sensors and actuators
ENVIRONMENT_VARIABLES = tuple(
    var for var in EnvVar.items.values()
    if GROUP_ENVIRONMENT in var.groups
)


class RollingMedian(object):
    """
    Calculates the median for the last n points. Could be a potential alternative to EWMA
    """
    def __init__(self, num_initial_values=100, filter_results=True):
        self.average = None
        self.recent_values = []
        self.FILTER_RESULTS = filter_results
        self.NUM_INITIAL_VALUES = num_initial_values

    def __call__(self, sample):
        # Update the average
        if self.average is None:
            self.average = sample
            return
        self.shift_recent_values(sample)
        self.average = np.median(self.recent_values)

    def shift_recent_values(self, sample):
        self.recent_values.append(sample)
        if len(self.recent_values) > self.NUM_INITIAL_VALUES:
            self.recent_values.pop(0)


class EWMA(object):
    """
    Calculate the Exponentially Weighted Moving Average (EWMA) for an input variable.
    EWMAi = alpha * Xi + (1 - alpha) * EWMAi-1
    Highly dependent on the number of values used to calculate the percentile. Need to be enough to capture the baseline distribution.
      # Todo: Explore other ways to ensure this is long enough.
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
        self.NUM_INITIAL_VALUES = 100

    def __call__(self, sample):
        # Update the average
        if self.average is None:
            self.average = sample
            return
        self.shift_recent_values(sample)
        self.calc_iqr()
        if len(self.recent_values) == self.NUM_INITIAL_VALUES - 1 and self.filter_results:
            self.recalculate_ewma_excluding_fliers()  # Uses median as a starting point
        if not self.is_outlier(sample):
            self.average = self.a * sample + (1 - self.a) * self.average

    def shift_recent_values(self, sample):
        self.recent_values.append(sample)
        if len(self.recent_values) > self.NUM_INITIAL_VALUES:
            self.recent_values.pop(0)

    def calc_iqr(self):
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


def filter_topic(src_topic, dest_topic, topic_type):
    """
    Publishes a measured data point given the raw value by applying the EWMA function to the data.
    :param src_topic: String? - Source topic signal to be filtered
    :param dest_topic: String? - Output topic to publish new data to
    :param topic_type: The data type of the topic, aka Float64, Int32, etc
    :return: sub, pub : subscribed topic (raw measured value) and published topic (filtered (smoothed) value)
    """
    rospy.loginfo("Filtering topic {} to topic {}".format(
        src_topic, dest_topic
    ))
    pub = rospy.Publisher(dest_topic, topic_type, queue_size=10)
    f = EWMA(0.3)
    def callback(src_item):
        f(src_item.data)
        dest_item = topic_type(f.average)
        pub.publish(dest_item)
    sub = rospy.Subscriber(src_topic, topic_type, callback)
    return sub, pub


def forward_topic(src_topic, dest_topic, topic_type):
    rospy.loginfo("Forwarding topic {} to topic {}".format(
        src_topic, dest_topic
    ))
    pub = rospy.Publisher(dest_topic, topic_type, queue_size=10)
    def callback(src_item):
        dest_item = topic_type(src_item.data)
        pub.publish(dest_item)
    sub = rospy.Subscriber(src_topic, topic_type, callback)
    return sub, pub


def filter_all_variable_topics(variables):
    """
    Given an iterator publishers, where each publisher is a two-tuple
    `(topic, type)`, publishes a filtered topic endpoint.
    """
    for env_var in variables:
        src_topic = "{}/raw".format(env_var)
        dest_topic = "{}/measured".format(env_var)
        # Ignore type associated with environmental variable type and
        # coerce to Float64

        # @FIXME this is a short-term fix for preventing boolean values from
        # being filtered by the EWMA filter.
        #
        # Explanation: right now all topics under `/environment/<id>` are
        # float64 type, with Boolean being 1 or 0. This was judged to be a
        # simpler architecture at the time. Values from sensors may be any
        # type, but are coerced to Float64. The same is true for actuators.
        # However, this assumption breaks down for filtering boolean values,
        # since the EWMA will produce fractional numbers that don't coerce
        # correctly back to boolean.
        #
        # In future, we should change the architecture of the system to support
        # standard ros types under `/environment/<id>`.
        if env_var == WATER_LEVEL_HIGH:
            forward_topic(src_topic, dest_topic, Float64)
        else:
            filter_topic(src_topic, dest_topic, Float64)

if __name__ == '__main__':
    rospy.init_node("topic_filter")
    # Make sure that we're under an environment namespace.
    filter_all_variable_topics(ENVIRONMENT_VARIABLES)
    rospy.spin()
