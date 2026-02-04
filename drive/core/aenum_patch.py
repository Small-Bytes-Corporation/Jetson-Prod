"""
Patch aenum.extend_enum for Python 3.6 compatibility with IntEnum/IntFlag (Jetson).

On Python 3.6, fusion_engine_client's use of enum_bitmask(SatelliteType) triggers
  TypeError: object.__new__(SatelliteTypeMask) is not safe, use int.__new__()
because aenum's extend_enum can call object.__new__ for dynamically created
IntEnum-derived classes. This module patches extend_enum so that for enum
classes that are subclasses of int, we use int.__new__ when creating new members.

Import this module before any code that imports fusion_engine_client (main.py
does this at startup). If RTK still fails on Jetson, use Python 3.8+ in a venv:
  sudo apt install python3.8
  python3.8 -m venv venv && source venv/bin/activate
  pip install -r requirements.txt
"""

import sys

if sys.version_info >= (3, 7):
    # No patch needed on Python 3.7+
    _patch_applied = False
else:
    _patch_applied = False

    try:
        import aenum
    except ImportError:
        pass
    else:
        _orig_extend_enum = aenum.extend_enum

        def _safe_int_new_member(cls, *args, **kwds):
            """Create IntEnum/IntFlag member with int.__new__ (Python 3.6 safe)."""
            value = args[0] if args else 0
            return int.__new__(cls, value)

        def _patched_extend_enum(enumeration, name, *args, **kwds):
            if issubclass(enumeration, int):
                orig_new_member = getattr(enumeration, "_new_member_", None)
                try:
                    enumeration._new_member_ = staticmethod(_safe_int_new_member)
                    return _orig_extend_enum(enumeration, name, *args, **kwds)
                finally:
                    if orig_new_member is not None:
                        enumeration._new_member_ = orig_new_member
                    else:
                        try:
                            del enumeration._new_member_
                        except AttributeError:
                            pass
            return _orig_extend_enum(enumeration, name, *args, **kwds)

        aenum.extend_enum = _patched_extend_enum
        _patch_applied = True
