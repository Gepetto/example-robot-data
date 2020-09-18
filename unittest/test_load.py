#!/usr/bin/env python

import unittest

from example_robot_data import load


class RobotTestCase(unittest.TestCase):
    def check(self, name, expected_nq, expected_nv):
        """Helper function for the real tests"""
        robot = load(name, display=False)
        self.assertEqual(robot.model.nq, expected_nq)
        self.assertEqual(robot.model.nv, expected_nv)
        self.assertTrue(hasattr(robot, "q0"))

    def test_anymal(self):
        self.check('anymal', 19, 18)

    def test_anymal_kinova(self):
        self.check('anymal_kinova', 25, 24)

    def test_double_pendulum(self):
        self.check('double_pendulum', 2, 2)

    def test_hector(self):
        self.check('hector', 7, 6)

    def test_hyq(self):
        self.check('hyq', 19, 18)

    def test_icub(self):
        self.check('icub', 39, 38)

    def test_icub_reduced(self):
        self.check('icub_reduced', 36, 35)

    def test_iris(self):
        self.check('iris', 7, 6)

    def test_kinova(self):
        self.check('kinova', 9, 6)

    def test_panda(self):
        self.check('panda', 9, 9)

    def test_romeo(self):
        self.check('romeo', 62, 61)

    def test_solo(self):
        self.check('solo', 15, 14)

    def test_solo12(self):
        self.check('solo12', 19, 18)

    def test_talos(self):
        self.check('talos', 39, 38)

    def test_talos_box(self):
        self.check('talos_box', 39, 38)

    def test_talos_full(self):
        self.check('talos_full', 51, 50)

    def test_talos_full_box(self):
        self.check('talos_full_box', 51, 50)

    def test_talos_arm(self):
        self.check('talos_arm', 7, 7)

    def test_talos_legs(self):
        self.check('talos_legs', 19, 18)

    def test_tiago(self):
        self.check('tiago', 50, 48)

    def test_tiago_no_hand(self):
        self.check('tiago_no_hand', 14, 12)

    def test_ur5(self):
        self.check('ur5', 6, 6)

    def test_ur5_gripper(self):
        self.check('ur5_gripper', 6, 6)

    def test_ur5_limited(self):
        self.check('ur5_limited', 6, 6)


if __name__ == '__main__':
    unittest.main()
