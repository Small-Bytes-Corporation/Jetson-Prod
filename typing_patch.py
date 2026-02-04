"""
Patch typing.TypeAlias for Python 3.9 compatibility.

TypeAlias was added in Python 3.10. On Python 3.9, we inject TypeAlias from
typing_extensions into the typing module so that "from typing import TypeAlias"
works for any dependency that requires it.

Import this module at the very start of main.py.
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
