#!/usr/bin/env python2

import sys
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


class ANYmalTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadANYmal()
    RobotTestCase.NQ = 19
    RobotTestCase.NV = 18


class ANYmalKinovaTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadANYmal(withArm="kinova")
    RobotTestCase.NQ = 27
    RobotTestCase.NV = 24


class HyQTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadHyQ()
    RobotTestCase.NQ = 19
    RobotTestCase.NV = 18


class TalosTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadTalos()
    RobotTestCase.NQ = 39
    RobotTestCase.NV = 38


class TalosArmTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadTalosArm()
    RobotTestCase.NQ = 7
    RobotTestCase.NV = 7


class TalosArmFloatingTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadTalosArm()
    RobotTestCase.NQ = 14
    RobotTestCase.NV = 13


class TalosLegsTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadTalosLegs()
    RobotTestCase.NQ = 19
    RobotTestCase.NV = 18


class ICubTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadICub(reduced=False)
    RobotTestCase.NQ = 39
    RobotTestCase.NV = 38


class SoloTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadSolo()
    RobotTestCase.NQ = 15
    RobotTestCase.NV = 14


class Solo12Test(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadSolo(False)
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


class UR5Test(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadUR()
    RobotTestCase.NQ = 6
    RobotTestCase.NV = 6


class UR5LimitedTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadUR(limited=True)
    RobotTestCase.NQ = 6
    RobotTestCase.NV = 6


class UR5GripperTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadUR(gripper=True)
    RobotTestCase.NQ = 6
    RobotTestCase.NV = 6


class KinovaTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadKinova()
    RobotTestCase.NQ = 9
    RobotTestCase.NV = 6


class RomeoTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadRomeo()
    RobotTestCase.NQ = 62
    RobotTestCase.NV = 61


class PandaTest(RobotTestCase):
    RobotTestCase.ROBOT = example_robot_data.loadPanda()
    RobotTestCase.NQ = 9
    RobotTestCase.NV = 9


if __name__ == '__main__':
    test_classes_to_run = [
        ANYmalTest, ANYmalKinovaTest, HyQTest, TalosTest, TalosArmTest, TalosArmFloatingTest, TalosLegsTest, ICubTest,
        SoloTest, Solo12Test, TiagoTest, TiagoNoHandTest, UR5Test, UR5LimitedTest, UR5GripperTest, KinovaTest,
        RomeoTest, PandaTest
    ]
    loader = unittest.TestLoader()
    suites_list = []
    for test_class in test_classes_to_run:
        suite = loader.loadTestsFromTestCase(test_class)
        suites_list.append(suite)
    big_suite = unittest.TestSuite(suites_list)
    runner = unittest.TextTestRunner()
    results = runner.run(big_suite)
    sys.exit(not results.wasSuccessful())
