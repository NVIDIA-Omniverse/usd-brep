# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""usd-validation-nvidia plugin that registers the BrepArray schema rules.

The usd-brep wheel declares ``BrepUsdSchemaValidatorPlugin`` as an ``omni.asset_validator`` entry
point, so ``nvidia_usd_validate`` and ``ValidationEngine`` pick up ``BrepUsdSchemaValidator`` once
the wheel and ``usd-validation-nvidia`` are installed (``pip install usd-brep[validation]``).
Callers outside the plugin system can call ``register_all()`` themselves.
"""

from usd_validation_nvidia import CategoryRuleRegistry, register_rule

from .brep_usdschema_validator import BrepUsdSchemaValidator
from .validate import register_omnisolid_schema


def register_all():
    """Register the omniSolid schema, then the BrepArray schema rules.

    The schema is registered before any stage is validated: without it, BrepArray prims have
    no resolved type and BrepUsdSchemaValidator reports false failures.
    """
    register_omnisolid_schema()
    if CategoryRuleRegistry().get_category(BrepUsdSchemaValidator) is None:
        register_rule("Omni:Geometry")(BrepUsdSchemaValidator)


def unregister_all():
    """Unregister the BrepArray schema rules from usd-validation-nvidia."""
    registry = CategoryRuleRegistry()
    if registry.get_category(BrepUsdSchemaValidator) is not None:
        registry.remove(BrepUsdSchemaValidator)


class BrepUsdSchemaValidatorPlugin:
    """usd-validation-nvidia entry point for the BrepArray schema rules."""

    def on_startup(self):
        register_all()

    def on_shutdown(self):
        unregister_all()
