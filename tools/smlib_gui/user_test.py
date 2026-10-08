# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""User-managed SMLib GUI tests.

Fill ``test_N()`` bodies with
``sm`` / ``smdev`` commands, then pick that number in the GUI and press Run.

The GUI injects ``sm`` (``_omni_solid``) and ``smdev`` (``_smlib_dev``) at
playback. Do not import those modules here. Edit this file, press Reload if
the GUI is already open, and switch numbers without restarting.

Kernel-direct C++ lives in ``source/SmPyDevLib/src/SmPyDevUserTests.cpp``.
``test_4`` calls ``smdev.user_test()``; rebuild ``_smlib_dev`` after editing
that C++ file. ``test_3`` calls ``smdev.user_test_example()``, the kernel analog of
``test_2``.

Local case bodies do not need to be committed.
"""

# Names injected by the GUI at playback. Placeholders keep this file importable.
sm = None
smdev = None


def test_0():
    """Empty."""
    pass


def test_1():
    """Filleted box union sphere."""
    box = sm.create_box((-5, -5, 0), 10, 10, 10)
    sm.fillet_edges(box, box.edges(), radius=1.0)
    sph = sm.create_sphere((0, 0, 12), 2.0)
    result = sm.boolean_union(box, sph)
    smdev.assert_valid(result)


def test_2():
    """Box with kernel dump and AssertValid."""
    box = sm.create_box((0, 0, 0), 10, 10, 10)
    smdev.dump(box)
    smdev.assert_valid(box)


def test_3():
    """Kernel-direct C++ example (SmPyDevUserTests.cpp)."""
    result = smdev.user_test_example()

def test_4():
    """Kernel-direct C++ stub (SmPyDevUserTests.cpp)."""
    result = smdev.user_test()


def test_5():
    pass


def test_6():
    pass


def test_7():
    pass


def test_8():
    pass


def test_9():
    pass
