# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Select license texts for the native libraries vendored into the wheel."""

import os
from typing import List


# Explicit ownership: Windows stages third-party DLLs into the build directory too.
_FIRST_PARTY_LIBRARIES = {"smlib", "sm_api", "sm_api_usd", "brep_sm_usd", "brep_usd_data", "usd_brep_utils"}

# Texts in the OpenUSD dependency's PACKAGE-LICENSES, and the libraries they cover.
_LICENSE_TEXTS = (
    ("usd-license.txt", ("usd_",)),
    ("onetbb-LICENSE.txt", ("tbb",)),
    ("hwloc-COPYING.txt", ("tbbbind", "hwloc")),  # hwloc is built into Windows tbbbind
)


def license_texts(libs: List[str], build_dir: str) -> List[str]:
    """Recognize first-party libraries by name and location; reject unattributed dependencies."""
    names = {"pybind11-LICENSE.txt"}  # compiled into _omni_solid
    build_dir = os.path.normcase(os.path.normpath(build_dir))
    for lib in libs:
        stem = os.path.basename(lib).lower().removeprefix("lib")
        local = os.path.normcase(os.path.normpath(os.path.dirname(lib))) == build_dir
        if local and stem.split(".", 1)[0] in _FIRST_PARTY_LIBRARIES:
            continue
        covering = [text for text, prefixes in _LICENSE_TEXTS if stem.startswith(prefixes)]
        if not covering:
            raise RuntimeError(f"no license text known for vendored library {os.path.basename(lib)}")
        names.update(covering)
    return sorted(names)
