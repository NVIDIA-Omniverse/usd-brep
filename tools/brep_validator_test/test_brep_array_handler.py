# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""brep_array_handler.py, which validate_directory.sh runs. It exits 0 whatever it finds, so check its output."""

import subprocess
import sys
from pathlib import Path

TEST_ROOT = Path(__file__).resolve().parent
if str(TEST_ROOT) not in sys.path:
    sys.path.insert(0, str(TEST_ROOT))
try:
    from .utils.base_test_case import BrepValidatorBaseTestCase
except ImportError:
    from utils.base_test_case import BrepValidatorBaseTestCase

# brep_validator_cli sits next to brep_validator_test in the source tree and in the release package.
HANDLER = TEST_ROOT.parent / "brep_validator_cli" / "brep_array_handler.py"


class BrepArrayHandlerTestCase(BrepValidatorBaseTestCase):
    def run_handler(self, fixture_name):
        result = subprocess.run(
            [sys.executable, str(HANDLER), str(self.test_files_dir / fixture_name)],
            capture_output=True,
            text=True,
            timeout=300,
            check=True,
        )
        # The handler prints a traceback and carries on when validation raises.
        self.assertNotIn("Traceback", result.stdout + result.stderr)
        return result.stdout

    def test_valid_fixture_passes(self):
        self.assertIn("Validation passed: No issues found.", self.run_handler("CubeBrepArray.usda"))

    def test_invalid_fixture_reports_its_requirement(self):
        output = self.run_handler("Test_BA_115_Invalid_faceuse_faceIndex__out_of_range.usda")
        self.assertIn("Requirement: BA.115", output)
        self.assertNotIn("Validation passed", output)
