"""
basic algorithmic methods
"""
from __future__ import annotations
__all__ = ['box_filter', 'marching_cubes', 'trilinear_interpolate']
def box_filter(grid_values: ..., width: int = 3) -> ...:
    """
    apply a box filter of given width (typically 3) -- use multiple times to approximate Gaussian filter
    """
def marching_cubes(grid_values: ..., size: arr) -> tuple[arr, uintA]:
    """
    use Lewiner's original Marching Cubes algorithm compute a zero-levelset mesh from a 3D tensor. Returned vertices are centered and assume given total box size
    """
def trilinear_interpolate(pts: arr, grid_values: ..., grid_res: arr) -> ...:
    """
    use trilinear interpolation to sample values from a 3D grid of values
    """
