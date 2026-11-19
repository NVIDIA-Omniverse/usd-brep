# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

try:
    # Import the validator before our submodules. Its import starts every installed plugin,
    # including ours (plugin.py), which imports brep_usdschema_validator.brep_usdschema_validator; starting it
    # from inside that module's own import would find BrepUsdSchemaValidator not yet defined.
    import usd_validation_nvidia  # noqa: F401
except ImportError:
    pass  # brep_usdschema_validator.py raises the install hint.

from .brep_usdschema_validator import BrepUsdSchemaValidator
from .plugin import BrepUsdSchemaValidatorPlugin, register_all, unregister_all
from .validate import UsdSchemaValidationResult, register_omnisolid_schema, usdschema_validate_file

__all__ = [
    "BrepUsdSchemaValidator",
    "BrepUsdSchemaValidatorPlugin",
    "UsdSchemaValidationResult",
    "register_all",
    "register_omnisolid_schema",
    "unregister_all",
    "usdschema_validate_file",
]

