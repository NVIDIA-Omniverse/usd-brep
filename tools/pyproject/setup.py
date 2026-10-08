# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from setuptools import setup
from setuptools.dist import Distribution


class BinaryDistribution(Distribution):
    """Force a platform-specific wheel tag.

    The staged tree holds a compiled extension and its libraries, but no source
    setuptools can see, so it would otherwise be tagged `py3-none-any` and served
    to every platform.
    """

    def has_ext_modules(self):
        return True


setup(distclass=BinaryDistribution)
