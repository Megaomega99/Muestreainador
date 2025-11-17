"""
Web Interface for Arduino IIR Filter Control
Megaomega engineering for well-being © 2025
"""

__version__ = "2.0.0"
__author__ = "ToMuProRa Project"

from .filter_calculator import FilterCalculator
from .serial_communication import ArduinoController

__all__ = ['FilterCalculator', 'ArduinoController']
