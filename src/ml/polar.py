"""
Polar Diagram Module  
====================

Loads and interpolates the yacht's polar performance diagram.
Used to calculate target speeds and VMG optimization.
"""

import json
from dataclasses import dataclass
from typing import Optional, Tuple
import math
import logging

logger = logging.getLogger(__name__)


@dataclass
class PolarData:
    """Polar diagram data structure."""
    name: str
    tws: list[float]    # True wind speeds (knots)
    twa: list[float]    # True wind angles (degrees)
    stw: list[list[float]]  # Speed through water [twa_idx][tws_idx]


class Polar:
    """
    Yacht polar diagram for performance calculations.
    
    The polar provides target boat speed for given TWA and TWS,
    enabling VMG optimization and performance monitoring.
    """
    
    def __init__(self, polar_data: Optional[PolarData] = None):
        self._data = polar_data or self._default_polar()
        
    @classmethod
    def from_json(cls, filepath: str) -> 'Polar':
        """Load polar from JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        polar_data = PolarData(
            name=data.get('name', 'unknown'),
            tws=data['tws'],
            twa=data['twa'],
            stw=data['stw']
        )
        return cls(polar_data)
        
    @classmethod
    def pogo_1250(cls) -> 'Polar':
        """Return the Pogo 1250 polar (built-in)."""
        return cls(cls._pogo_1250_polar())
        
    @staticmethod
    def _pogo_1250_polar() -> PolarData:
        """Built-in Pogo 1250 polar data."""
        return PolarData(
            name='pogo1250',
            tws=[0, 4, 6, 8, 10, 12, 14, 16, 20, 25, 30, 35, 40, 45, 50, 55, 60],
            twa=[
                0, 5, 10, 15, 20, 25, 32, 36, 40, 45, 52, 60,
                70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180
            ],
            stw=[
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0.4, 0.6, 0.8, 0.9, 1, 1, 1, 1.1, 1.1, 1.1, 1.1, 0.1, 0.1, 0.1, 0, 0],
                [0, 0.8, 1.2, 1.6, 1.8, 2, 2, 2.1, 2.1, 2.2, 2.2, 2.2, 0.5, 0.2, 0.2, 0, 0],
                [0, 1.2, 1.8, 2.4, 2.7, 2.9, 3, 3.1, 3.2, 3.3, 3.3, 3.3, 1.2, 0.5, 0.3, 0, 0],
                [0, 1.4, 2.1, 2.7, 3.1, 3.4, 3.5, 3.6, 3.6, 3.7, 3.8, 3.7, 1.7, 0.7, 0.4, 0, 0],
                [0, 1.7, 2.5, 3.2, 3.7, 4, 4.1, 4.3, 4.3, 4.4, 4.5, 4.4, 2.6, 1.1, 0.4, 0, 0],
                [0, 2.8, 4.2, 5.4, 6.2, 6.7, 6.9, 7.1, 7.2, 7.4, 7.5, 7.4, 5.6, 2.2, 0.7, 0, 0],
                [0, 3.1, 4.7, 5.9, 6.7, 7, 7.2, 7.4, 7.6, 7.8, 7.9, 7.9, 6.5, 2.6, 0.8, 0, 0],
                [0, 3.5, 5.1, 6.3, 7, 7.3, 7.5, 7.7, 7.9, 8.1, 8.2, 8.3, 7.4, 2.9, 1.2, 0, 0],
                [0, 3.8, 5.6, 6.7, 7.3, 7.6, 7.8, 8, 8.2, 8.4, 8.5, 8.6, 8.2, 3, 1.3, 0, 0],
                [0, 4.2, 6, 7, 7.7, 8, 8.2, 8.3, 8.6, 8.9, 9, 9.1, 8.9, 3.2, 1.4, 0, 0],
                [0, 4.6, 6.3, 7.3, 8, 8.3, 8.5, 8.7, 9, 9.3, 9.5, 9.6, 9.6, 3.8, 1.9, 0, 0],
                [0, 4.8, 6.6, 7.5, 8.2, 8.6, 8.9, 9.1, 9.5, 9.8, 10.1, 10.4, 10.4, 4.2, 2.1, 0, 0],
                [0, 5, 6.9, 7.9, 8.3, 8.8, 9.2, 9.4, 9.9, 10.4, 10.9, 11.3, 11.3, 4.5, 2.3, 0, 0],
                [0, 5.3, 7.1, 8.1, 8.6, 8.9, 9.3, 9.7, 10.4, 11.1, 11.8, 12.5, 12.5, 5.6, 3.1, 0.6, 0.6],
                [0, 5.4, 7.1, 8.2, 8.8, 9.2, 9.5, 9.9, 10.9, 11.9, 12.8, 14.1, 14.1, 7.1, 4.2, 0.7, 0.7],
                [0, 5.3, 7, 8.1, 8.8, 9.4, 9.8, 10.3, 11.2, 12.7, 14.3, 15, 15, 8.3, 5.3, 1.5, 1.5],
                [0, 5, 6.8, 7.8, 8.6, 9.4, 10, 10.6, 11.8, 13.2, 14.9, 15.7, 15.7, 9.4, 6.3, 1.6, 1.6],
                [0, 4.5, 6.3, 7.4, 8.3, 9, 9.8, 10.6, 12.3, 14.4, 15.6, 16.6, 16.6, 10.8, 7.5, 2.5, 2.5],
                [0, 3.8, 5.6, 6.9, 7.8, 8.5, 9.2, 10, 12.2, 15, 16.3, 17.6, 17.6, 13.2, 9.7, 3.5, 2.6],
                [0, 3.2, 4.8, 6.1, 7.1, 7.9, 8.6, 9.3, 10.9, 14.4, 16.8, 18.6, 18.6, 14.9, 11.2, 3.7, 3.7],
                [0, 2.7, 4.1, 5.3, 6.4, 7.3, 8, 8.7, 10, 12.4, 15.4, 17.9, 17.9, 15.2, 11.6, 4.5, 3.6],
                [0, 2.4, 3.6, 4.8, 5.9, 6.8, 7.6, 8.2, 9.4, 11.4, 14.3, 16.6, 16.6, 15.8, 12.5, 5, 4.2],
                [0, 2.2, 3.3, 4.4, 5.5, 6.4, 7.2, 7.9, 9, 10.6, 12.8, 15.4, 15.4, 15.4, 12.3, 4.6, 3.9],
            ]
        )
        
    def _default_polar(self) -> PolarData:
        """Default polar (Pogo 1250)."""
        return self._pogo_1250_polar()
        
    def get_target_speed(self, twa: float, tws: float) -> float:
        """
        Get target boat speed from polar.
        
        Args:
            twa: True wind angle (degrees, 0-180)
            tws: True wind speed (knots)
            
        Returns:
            Target speed through water (knots)
        """
        # Normalize TWA to 0-180
        twa = abs(twa)
        if twa > 180:
            twa = 360 - twa
            
        # Find surrounding indices for interpolation
        twa_idx = self._find_indices(self._data.twa, twa)
        tws_idx = self._find_indices(self._data.tws, tws)
        
        # Bilinear interpolation
        return self._bilinear_interpolate(twa, tws, twa_idx, tws_idx)
        
    def _find_indices(self, arr: list[float], val: float) -> Tuple[int, int, float]:
        """Find bracketing indices and interpolation factor."""
        if val <= arr[0]:
            return (0, 0, 0.0)
        if val >= arr[-1]:
            return (len(arr) - 1, len(arr) - 1, 0.0)
            
        for i in range(len(arr) - 1):
            if arr[i] <= val <= arr[i + 1]:
                frac = (val - arr[i]) / (arr[i + 1] - arr[i]) if arr[i + 1] != arr[i] else 0
                return (i, i + 1, frac)
                
        return (len(arr) - 1, len(arr) - 1, 0.0)
        
    def _bilinear_interpolate(self, twa: float, tws: float,
                              twa_idx: Tuple[int, int, float],
                              tws_idx: Tuple[int, int, float]) -> float:
        """Bilinear interpolation of polar speed."""
        i0, i1, twa_frac = twa_idx
        j0, j1, tws_frac = tws_idx
        
        # Get corner values
        v00 = self._data.stw[i0][j0]
        v01 = self._data.stw[i0][j1]
        v10 = self._data.stw[i1][j0]
        v11 = self._data.stw[i1][j1]
        
        # Interpolate along TWS axis
        v0 = v00 + (v01 - v00) * tws_frac
        v1 = v10 + (v11 - v10) * tws_frac
        
        # Interpolate along TWA axis
        return v0 + (v1 - v0) * twa_frac
        
    def get_optimal_vmg_upwind(self, tws: float) -> Tuple[float, float, float]:
        """
        Find optimal upwind VMG angle.
        
        Args:
            tws: True wind speed (knots)
            
        Returns:
            (optimal_twa, target_stw, vmg)
        """
        best_vmg = 0.0
        best_twa = 45.0
        best_stw = 0.0
        
        # Search upwind angles (30-60 degrees typical)
        for twa_deg in range(25, 70):
            stw = self.get_target_speed(twa_deg, tws)
            vmg = stw * math.cos(math.radians(twa_deg))
            if vmg > best_vmg:
                best_vmg = vmg
                best_twa = twa_deg
                best_stw = stw
                
        return (best_twa, best_stw, best_vmg)
        
    def get_optimal_vmg_downwind(self, tws: float) -> Tuple[float, float, float]:
        """
        Find optimal downwind VMG angle.
        
        Args:
            tws: True wind speed (knots)
            
        Returns:
            (optimal_twa, target_stw, vmg)
        """
        best_vmg = 0.0
        best_twa = 150.0
        best_stw = 0.0
        
        # Search downwind angles (120-180 degrees typical)
        for twa_deg in range(110, 181):
            stw = self.get_target_speed(twa_deg, tws)
            # VMG downwind is speed * cos(180 - twa)
            vmg = stw * math.cos(math.radians(180 - twa_deg))
            if vmg > best_vmg:
                best_vmg = vmg
                best_twa = twa_deg
                best_stw = stw
                
        return (best_twa, best_stw, best_vmg)
        
    def get_performance_ratio(self, twa: float, tws: float, actual_stw: float) -> float:
        """
        Calculate performance as percentage of polar target.
        
        Args:
            twa: True wind angle (degrees)
            tws: True wind speed (knots)
            actual_stw: Actual speed through water (knots)
            
        Returns:
            Performance ratio (0.0 - 1.2 typical, >1 means above polar)
        """
        target = self.get_target_speed(twa, tws)
        if target <= 0:
            return 0.0
        return actual_stw / target
