#!/usr/bin/env python

import unittest

try:
    import pybullet
except ImportError:
    pybullet = False

from example_robot_data import load_full


class RobotTestCase(unittest.TestCase):
    def check(self, name, expected_nq, expected_nv, one_kg_bodies=[]):
        """Helper function for the real tests"""
        robot, _, urdf, _ = load_full(name, display=False, verbose=True)
        self.assertEqual(robot.model.nq, expected_nq)
        self.assertEqual(robot.model.nv, expected_nv)
        self.assertTrue(hasattr(robot, "q0"))
        self.assertTrue(hasattr(robot, "urdf"))
        if pybullet:
            self.check_pybullet(urdf, one_kg_bodies)

    def check_pybullet(self, urdf, one_kg_bodies):
        client_id = pybullet.connect(pybullet.DIRECT)
        robot_id = pybullet.loadURDF(urdf, physicsClientId=client_id)
        for joint_id in range(pybullet.getNumJoints(robot_id, client_id)):
            dynamics = pybullet.getDynamicsInfo(robot_id, joint_id, client_id)
            if dynamics[0] == 1:
                joint = pybullet.getJointInfo(robot_id, joint_id, client_id)
                # uncomment on python >= 3.4 to get full list of wrong bodies at once
                # with self.subTest():
                self.assertIn(joint[12].decode(), one_kg_bodies)
        pybullet.disconnect(client_id)

    def test_b1(self):
        self.check("b1", 19, 18)

    def test_go1(self):
        self.check("go1", 19, 18)

    def test_a1(self):
        self.check("a1", 19, 18)

    def test_anymal(self):
        self.check("anymal", 19, 18)

    def test_anymal_c(self):
        self.check("anymal_c", 19, 18)

    def test_anymal_kinova(self):
        self.check("anymal_kinova", 25, 24)

    def test_baxter(self):
        self.check("baxter", 19, 19)

    def test_cassie(self):
        try:
            self.check("cassie", 29, 28)
        except ImportError:
            import pinocchio

            pin_version = tuple(int(i) for i in pinocchio.__version__.split("."))
            self.assertLess(pin_version, (2, 9, 1))

    def test_double_pendulum(self):
        self.check("double_pendulum", 2, 2)

    def test_double_pendulum_continuous(self):
        self.check("double_pendulum_continuous", 4, 2)

    def test_double_pendulum_simple(self):
        self.check("double_pendulum_simple", 2, 2)

    def test_asr(self):
        self.check("asr_twodof", 2, 2, one_kg_bodies=["ground"])

    def test_hector(self):
        self.check("hector", 7, 6)

    def test_hyq(self):
        self.check("hyq", 19, 18)

    def test_icub(self):
        self.check("icub", 39, 38)

    def test_icub_reduced(self):
        self.check("icub_reduced", 36, 35)

    def test_iris(self):
        self.check("iris", 7, 6)

    def test_kinova(self):
        self.check("kinova", 9, 6)

    def test_panda(self):
        self.check("panda", 9, 9)

    def test_allegro_right(self):
        self.check("allegro_right_hand", 16, 16)

    def test_allegro_left(self):
        self.check("allegro_left_hand", 16, 16)

    def test_quadruped(self):
        self.check("quadruped", 15, 14)

    def test_romeo(self):
        self.check("romeo", 62, 61)

    def test_simple_humanoid(self):
        self.check(
            "simple_humanoid", 36, 35, one_kg_bodies=["LARM_LINK3", "RARM_LINK3"]
        )

    def test_simple_humanoid_classical(self):
        self.check(
            "simple_humanoid_classical",
            36,
            35,
            one_kg_bodies=["LARM_LINK3", "RARM_LINK3"],
        )

    def test_bolt(self):
        self.check("bolt", 13, 12)

    def test_solo8(self):
        self.check("solo8", 15, 14)

    def test_solo12(self):
        self.check("solo12", 19, 18)

    def test_finger_edu(self):
        self.check("finger_edu", 3, 3, one_kg_bodies=["finger_base_link"])

    def test_talos(self):
        self.check("talos", 39, 38)

    def test_laikago(self):
        self.check("laikago", 19, 18)

    def test_pr2(self):
        self.check("pr2", 41, 36)

    def test_talos_box(self):
        self.check("talos_box", 39, 38)

    def test_talos_full(self):
        self.check("talos_full", 51, 50)

    def test_talos_full_box(self):
        self.check("talos_full_box", 51, 50)

    def test_talos_arm(self):
        self.check("talos_arm", 7, 7)

    def test_talos_legs(self):
        self.check("talos_legs", 19, 18)

    def test_tiago(self):
        self.check("tiago", 50, 48)

    def test_tiago_dual(self):
        self.check("tiago_dual", 111, 101)

    def test_tiago_no_hand(self):
        self.check("tiago_no_hand", 14, 12)

    def test_ur3(self):
        self.check("ur3", 6, 6)

    def test_ur3_gripper(self):
        self.check("ur3_gripper", 6, 6)

    def test_ur3_limited(self):
        self.check("ur3_limited", 6, 6)

    def test_ur5(self):
        self.check("ur5", 6, 6)

    def test_ur5_gripper(self):
        self.check("ur5_gripper", 6, 6)

    def test_ur5_limited(self):
        self.check("ur5_limited", 6, 6)

    def test_ur10(self):
        self.check("ur10", 6, 6)

    def test_ur10_limited(self):
        self.check("ur10_limited", 6, 6)


if __name__ == "__main__":
    unittest.main()
