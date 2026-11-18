"""
This module provides utility functions for data manipulation and logging
as a class-based structure.

Class:
    DataUtils: Provides methods to convert dictionaries and strings to dataclass
               instances and extract point coordinates from strings.

Modules:
    os: Provides a way of using operating system-dependent functionality.
    traceback: Provides utilities for extracting, formatting, and printing stack traces.
    typing: Provides support for type hints.
    rclpy.impl.rcutils_logger: Provides logging functionality for ROS 2.

Attributes:
    logger (RcutilsLogger): Logger instance for logging errors and messages.
"""
import os
import traceback
from dataclasses import MISSING, fields, is_dataclass
from typing import TypeVar

from rclpy.impl.rcutils_logger import RcutilsLogger

T = TypeVar('T')


class DataUtils:
    """Utility class for data conversion and logging."""

    logger = RcutilsLogger(os.path.basename(__file__))

    @staticmethod
    def from_dict(input_class: type[T], data: dict) -> T:
        """Recursively converts a dictionary into an instance of the given dataclass."""
        try:
            if not is_dataclass(input_class):
                raise ValueError(f"{input_class} must be a dataclass")

            init_data = {}
            for f in fields(input_class):
                if f.name not in data:
                    if f.default is MISSING and f.default_factory is MISSING:
                        raise KeyError(f.name)
                    continue  # a field with a default may be missing from the file (an older config)
                value = data[f.name]
                if is_dataclass(f.type) and isinstance(f.type, type):
                    value = DataUtils.from_dict(f.type, value if value is not None else {})
                init_data[f.name] = value

            return input_class(**init_data)
        except Exception as e:
            DataUtils.logger.error(
                f"Error in from_dict: {e}\n{traceback.format_exc()}")
            raise

    @staticmethod
    def str_to_model(s: str, data_class: type[T]) -> T:
        """
        Convert a comma-separated string into an instance of a given class.

        Example:
            >>> @dataclass
            ... class Point:
            ...     x: float
            ...     y: float
            ...     z: float
            >>> DataUtils.str_to_model("(1.0, 2.0, 3.0)", Point)
            Point(x=1.0, y=2.0, z=3.0)
        """
        parts = s.strip("()").split(",")
        parts = [float(p.strip()) for p in parts if p.strip()]
        return data_class(*parts[:len(data_class.__annotations__)])
