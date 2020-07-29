#!/usr/bin/env python

import unittest

import example_robot_data as erd


class RobotTestCase(unittest.TestCase):
    def check(self, robot, expected_nq, expected_nv):
        """Helper function for the real tests"""
        self.assertEqual(robot.model.nq, expected_nq)
        self.assertEqual(robot.model.nv, expected_nv)
        self.assertTrue(hasattr(robot, "q0"))

    def test_anymal(self):
        self.check(erd.loadANYmal(), 19, 18)

    def test_anymal_kinova(self):
        self.check(erd.loadANYmal(withArm="kinova"), 25, 24)

    def test_hyq(self):
        self.check(erd.loadHyQ(), 19, 18)

    def test_talos(self):
        self.check(erd.loadTalos(), 39, 38)

    def test_talos_arm(self):
        self.check(erd.loadTalos(arm=True), 7, 7)

    def test_talos_legs(self):
        self.check(erd.loadTalos(legs=True), 19, 18)

    def test_icub(self):
        self.check(erd.loadICub(reduced=False), 39, 38)

    def test_solo(self):
        self.check(erd.loadSolo(), 15, 14)

    def test_solo12(self):
        self.check(erd.loadSolo(False), 19, 18)

    def test_tiago(self):
        self.check(erd.loadTiago(), 50, 48)

    def test_tiago_no_hand(self):
        self.check(erd.loadTiago(hand=False), 14, 12)

    def test_ur5(self):
        self.check(erd.loadUR(), 6, 6)

    def test_ur5_limited(self):
        self.check(erd.loadUR(limited=True), 6, 6)

    def test_ur5_gripper(self):
        self.check(erd.loadUR(gripper=True), 6, 6)

    def test_kinova(self):
        self.check(erd.loadKinova(), 9, 6)

    def test_romeo(self):
        self.check(erd.loadRomeo(), 62, 61)

    def test_panda(self):
        self.check(erd.loadPanda(), 9, 9)


if __name__ == '__main__':
    unittest.main()
