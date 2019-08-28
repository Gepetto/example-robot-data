#!/usr/bin/env python2

import unittest
import example_robot_data


class RobotTestCase(unittest.TestCase):
    ROBOT = None
    NQ = None
    NV = None

    def test_nq(self):
        model = self.ROBOT.model
        self.assertEqual(model.nq, self.NQ, "Wrong nq value.")

    def test_nv(self):
        model = self.ROBOT.model
        self.assertEqual(model.nv, self.NV, "Wrong nv value.")

    def test_q0(self):
        self.assertTrue(hasattr(self.ROBOT, "q0"), "It doesn't have q0")


class TalosArmTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadTalosArm()
    RobotTestCase.NQ = 7
    RobotTestCase.NV = 7


class TalosArmFloatingTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadTalosArm()
    RobotTestCase.NQ = 14
    RobotTestCase.NV = 13


class TalosTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadTalos()
    RobotTestCase.NQ = 39
    RobotTestCase.NV = 38


class TalosLegsTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadTalosLegs()
    RobotTestCase.NQ = 19
    RobotTestCase.NV = 18


class HyQTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadHyQ()
    RobotTestCase.NQ = 19
    RobotTestCase.NV = 18


class TiagoTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadTiago()
    RobotTestCase.NQ = 50
    RobotTestCase.NV = 48


class TiagoNoHandTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadTiagoNoHand()
    RobotTestCase.NQ = 14
    RobotTestCase.NV = 12


class ICubTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadICub(reduced=False)
    RobotTestCase.NQ = 39
    RobotTestCase.NV = 38


if __name__ == '__main__':
    unittest.main()
