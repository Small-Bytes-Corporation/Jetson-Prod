"""
Patch typing.TypeAlias for Python 3.9 compatibility with fusion-engine-client.

fusion-engine-client uses TypeAlias which was added in Python 3.10. On Python 3.9,
we inject TypeAlias from typing_extensions into the typing module so that
"from typing import TypeAlias" works when fusion_engine_client is loaded.

Import this module at the very start of main.py, before any code that imports
fusion_engine_client (e.g. device_discovery, rtk_controller).
"""

import sys

if sys.version_info < (3, 10):
    import typing
    try:
        from typing_extensions import TypeAlias
        setattr(typing, "TypeAlias", TypeAlias)
    except ImportError:
        # typing_extensions not installed - use type as fallback
        # (TypeAlias is used for type hints; type works as a no-op placeholder)
        setattr(typing, "TypeAlias", type)
