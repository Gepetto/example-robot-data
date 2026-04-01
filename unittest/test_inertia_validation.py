#!/usr/bin/env python

import unittest

import numpy as np
import pinocchio as pin

from example_robot_data import ROBOTS, load


def _get_inertia_attribute(inertia, *names):
    for name in names:
        if hasattr(inertia, name):
            value = getattr(inertia, name)
            return value() if callable(value) else value
    joined_names = ", ".join(names)
    raise AttributeError(f"Unsupported inertia API, expected one of: {joined_names}")


def _validate_rigid_body_inertia(inertia, context, atol=1e-12):
    """Validate one rigid-body inertia for trusted-model execution."""
    mass = float(_get_inertia_attribute(inertia, "mass"))
    com = np.asarray(_get_inertia_attribute(inertia, "com", "lever"), dtype=float)
    inertia_com = np.asarray(
        _get_inertia_attribute(inertia, "inertiaCom", "inertia"), dtype=float
    )

    if not np.isfinite(mass):
        raise ValueError(f"{context} has non-finite mass {mass}")
    if not np.isfinite(com).all():
        raise ValueError(f"{context} has non-finite center of mass {com}")
    if not np.isfinite(inertia_com).all():
        raise ValueError(f"{context} has non-finite inertia matrix entries")
    if mass < -atol:
        raise ValueError(f"{context} has negative mass {mass}")

    sym_inertia = 0.5 * (inertia_com + inertia_com.T)
    if not np.allclose(inertia_com, sym_inertia, atol=atol, rtol=0.0):
        raise ValueError(f"{context} inertia matrix must be symmetric")

    principal_moments = np.linalg.eigvalsh(sym_inertia)
    if principal_moments[0] < -atol:
        raise ValueError(f"{context} inertia matrix must be positive semidefinite")

    if mass <= atol:
        if np.max(np.abs(sym_inertia)) > atol:
            raise ValueError(f"{context} is massless but has non-zero inertia")
        return

    i0, i1, i2 = principal_moments
    if i0 + i1 < i2 - atol or i0 + i2 < i1 - atol or i1 + i2 < i0 - atol:
        raise ValueError(
            f"{context} violates rigid-body inertia triangle inequalities"
        )


def _pinocchio_version():
    return tuple(int(part) for part in pin.__version__.split("."))


class InertiaValidationTestCase(unittest.TestCase):
    def _check_robot(self, name):
        robot = load(name, display=False, verbose=False)
        model = robot.model

        self.assertEqual(
            len(model.inertias),
            len(model.names),
            f"{name} exposes inconsistent joint/inertia metadata",
        )

        for joint_id, inertia in enumerate(model.inertias):
            joint_name = model.names[joint_id]
            context = f"{name}:{joint_name} (joint {joint_id})"
            with self.subTest(robot=name, joint=joint_name, joint_id=joint_id):
                _validate_rigid_body_inertia(inertia, context)

    def test_registered_robots_have_physically_valid_inertias(self):
        for name in sorted(name for name in ROBOTS if name != "cassie"):
            with self.subTest(robot=name):
                self._check_robot(name)

    def test_cassie_has_physically_valid_inertias(self):
        try:
            self._check_robot("cassie")
        except ImportError:
            if _pinocchio_version() >= (2, 9, 1):
                self.skipTest(
                    "Cassie requires Pinocchio SDF support in this environment."
                )
            raise


if __name__ == "__main__":
    unittest.main()
