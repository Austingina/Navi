"""vrr_vpr — Visual Place Recognition engine for C++ integration.

Exposes a stateful VPREngine class that wraps global descriptor extraction
(MegaLoc or SALAD-indoor), FAISS retrieval, and optional re-ranking stages
(geometric and/or color). Designed for C++ embedding via pybind11.
"""

from .engine import VPREngine

__all__ = ["VPREngine"]
__version__ = "1.1.0"
