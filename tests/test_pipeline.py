import numpy as np

import matplotlib.pyplot as plt

# with open("build/float_array.bin", "rb") as file:
#     # Read the data as numpy.float32
#     sine = np.fromfile(file, dtype=np.float32)
#
# plt.plot(sine)
# plt.show()
#
import unittest
import numpy as np
import matplotlib.pyplot as plt

import unittest

from build import config
from build.lib import pipeline


class Pipeline(unittest.TestCase):
    def test_creation(self):
        pipe = pipeline.CyPipeline()
        self.assertIsNot(pipe, None)
        # print(pipe)

    def test_recent(self):
        pipe = pipeline.CyPipeline()
        recent = pipe.mostRecent()
        self.assertEqual(recent, 0)

    def test_latest(self):
        pipe = pipeline.CyPipeline()
        latest = pipe.getRingBuffer()
        # print(latest.shape)
        # self.assertEqual(recent, 0)


#
#
# class Tester(unittest.TestCase):
#     def test_data(self):
#         data = np.fromfile("pipeline.bin", dtype=np.float32)
#         data = data.reshape((config.N_SENSORS, config.BUFFER_LENGTH))
#         print(data)
#         # plt.plot(data[140])
#         plt.specgram(data[140])
#
#         plt.show()


# class TestPipe(unittest.TestCase):
#     def test_receive(self):
#         with open("../build/float2_array.bin", "rb") as file:
#             sine = np.fromfile(file, dtype=np.float32)
#
#         print(sine)
#         # plt.specgram(sine)
#         plt.plot(sine)
#         plt.show()
#         print("sdhjfksjd")
