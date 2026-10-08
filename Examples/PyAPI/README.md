<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Public Python API Examples

These examples compose the stable `_omni_solid` Python API. Run them after
building the repository in an environment where `_omni_solid` is importable.

| Example | Demonstrates | Qualification |
|---|---|---|
| [round_corner_pipe.py](round_corner_pipe.py) | OCP-style spherical corners on a bounded rectangular pipe, composed from planar sweep, primitives, transforms, and exact Booleans | [test_round_corner_pipe_example.py](../../source/SmPyLib/tests/test_round_corner_pipe_example.py) |

For example:

```bash
python Examples/PyAPI/round_corner_pipe.py --output round_corner_pipe.smb
```
