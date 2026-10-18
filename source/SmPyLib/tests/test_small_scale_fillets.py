# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression for surface-cache convergence when filleting small boxes (issue #8)."""

import subprocess
import sys
import textwrap
import unittest


class TestSmallScaleFillets(unittest.TestCase):
    def test_default_tolerance_does_not_prevent_cache_convergence(self):
        # A child process keeps a stalled native call from hanging the test suite.
        script = textwrap.dedent("""
            import math
            import sys
            import _omni_solid as sm

            side = float(sys.argv[1])
            rotated = bool(int(sys.argv[2]))
            brep = sm.create_box((0, 0, 0), side, side, side)
            if rotated:
                sm.rotate(brep, (0, 0, 0), (1, 2, 3), 37)
                sm.translate(brep, (7 * side, -3 * side, 2 * side))

            # Fillet at the actual input scale and default tolerance to expose the reported bug.
            sm.circular_fillet(brep, side / 10)
            assert brep.is_manifold_solid()
            # Rounded cube: six planar faces, twelve edge fillets and eight corner patches.
            assert (brep.face_count(), brep.edge_count(), brep.vertex_count()) == (26, 48, 24)

            # Normalize only the finished result for accurate volume and bounds comparisons.
            normalized = brep.copy()
            if rotated:
                sm.translate(normalized, (-7 * side, 3 * side, -2 * side))
                sm.rotate(normalized, (0, 0, 0), (1, 2, 3), -37)
            sm.scale(normalized, (1 / side,) * 3)

            # Unit cube: radius 0.1 leaves a straight span of 0.8; sum its analytic components.
            expected_volume = (
                0.8**3                       # central cube
                + 6 * 0.8**2 * 0.1           # six face slabs
                + 3 * math.pi * 0.8 * 0.1**2  # twelve quarter-cylinders
                + 4 * math.pi * 0.1**3 / 3    # eight sphere octants
            )
            assert abs(normalized.volume(relative_accuracy=1e-8) - expected_volume) < 1e-6
            lo, hi = normalized.bounding_box(tight=True)
            assert all(abs(x - y) < 1e-6 for x, y in zip(lo + hi, (0, 0, 0, 1, 1, 1)))
        """)

        # Repeat the reported case three times, then check other scales and rigid transforms.
        cases = [(0.001, False)] * 3 + [(s, False) for s in (0.002, 0.01, 0.1, 10)]
        cases += [(0.001, True), (0.1, True)]
        for side, rotated in cases:
            with self.subTest(side=side, rotated=rotated):
                result = subprocess.run(
                    [sys.executable, "-c", script, str(side), str(int(rotated))],
                    capture_output=True,
                    text=True,
                    timeout=10,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
