# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/scripts"))
from usdview_hdusdbrep import release_paths, usdview_environment

parser = argparse.ArgumentParser()
parser.add_argument("--gpu-image", type=Path, help="also run native-GPU usdview smoke test and save a viewport PNG")
args = parser.parse_args()

build, _, _, schema = release_paths()
env, interpreter, usdview = usdview_environment()
env["OMNISOLID_PLUGIN_PATH"] = str(schema)

with tempfile.TemporaryDirectory(prefix="hdUsdBrep-tests-") as temp:
    fixture = Path(temp) / "box.usda"
    multiple = Path(temp) / "two-bodies.usda"
    subprocess.run(
        [
            str(interpreter),
            "-c",
            "import os, sys; "
            "dll_directories = [os.add_dll_directory(os.path.abspath(p)) for p in os.environ['PATH'].split(os.pathsep) "
            "if os.path.isdir(p)] if sys.platform == 'win32' else []; "
            "import _omni_solid as sm; "
            "box = sm.create_box((0,0,0),10,10,10); "
            "sm.usd.export_brep(box, sys.argv[1]); "
            "sm.usd.export_breps([box, sm.create_box((20,0,0),5,5,5)], sys.argv[2])",
            str(fixture),
            str(multiple),
        ],
        env=env,
        check=True,
    )
    command = [
        str(build / ("HYDRA_USD_BREP_test_app.exe" if sys.platform == "win32" else "HYDRA_USD_BREP_test_app")),
        str(schema),
        str(fixture),
        str(build / "plugins/hdUsdBrep"),
        str(multiple),
    ]
    subprocess.run(command, env=env, check=True)

    if args.gpu_image:
        image = args.gpu_image.resolve()
        image.parent.mkdir(parents=True, exist_ok=True)
        env["HDUSDBREP_GPU_IMAGE"] = str(image)
        result = subprocess.run(
            [
                str(interpreter),
                str(usdview.with_name("testusdview")),
                "--renderer",
                "Storm",
                "--testScript",
                str(Path(__file__).with_name("test_usdview.py")),
                str(multiple),
            ],
            env=env,
            capture_output=True,
            text=True,
            timeout=120,
        )
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        result.check_returncode()
        assert "PASS: native-GPU Storm rendering" in result.stdout, "GPU callback did not finish"
        for message in ("_PopulateVertexPrimvars", "Coding Error", "Runtime Error"):
            assert message not in result.stdout + result.stderr, f"renderer diagnostic: {message}"
