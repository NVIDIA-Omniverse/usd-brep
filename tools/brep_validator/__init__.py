# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

try:
    # Import the validator before our submodules. Its import starts every installed plugin,
    # including ours (plugin.py), which imports brep_validator.brep_validator; starting it
    # from inside that module's own import would find BrepValidator not yet defined.
    import usd_validation_nvidia  # noqa: F401
except ImportError:
    pass  # brep_validator.py raises the install hint.

from .brep_validator import BrepValidator
from .plugin import BrepValidatorPlugin, register_all, unregister_all
from .validate import BrepValidationResult, register_omnisolid_schema, validate_file

__all__ = [
    "BrepValidator",
    "BrepValidatorPlugin",
    "BrepValidationResult",
    "register_all",
    "register_omnisolid_schema",
    "unregister_all",
    "validate_file",
]

