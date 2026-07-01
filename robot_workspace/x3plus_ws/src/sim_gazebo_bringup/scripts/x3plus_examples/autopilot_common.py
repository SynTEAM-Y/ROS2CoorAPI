#!/usr/bin/env python3
"""Common utilities for vision-based autonomous manipulation.

Ported from ROS1 arm_autopilot package to ROS2.
Provides PID control and helper functions for vision-guided robot control.
"""

import time
import numpy as np


class SimplePID:
    """Discrete PID controller for smooth motion control.
    
    Adapted from ROS1 arm_autopilot simplePID class.
    Supports vector-valued targets for multi-axis control.
    
    Example:
        pid = SimplePID(target=[0.0, 0.0], P=[1.0, 1.0], I=[0.0, 0.0], D=[0.1, 0.1])
        control = pid.update(current_value=[0.5, -0.3])
    """
    
    def __init__(self, target, P, I, D):
        """Create a discrete PID controller.
        
        Args:
            target: Target value(s) - scalar or numpy array
            P: Proportional gain(s)
            I: Integral gain(s)
            D: Derivative gain(s)
        """
        # Convert to numpy arrays for vector operations
        self.setPoint = np.array(target, dtype=float)
        self.Kp = np.array(P, dtype=float)
        self.Ki = np.array(I, dtype=float)
        self.Kd = np.array(D, dtype=float)
        
        # Validate shapes
        if not (np.size(self.Kp) == np.size(self.Ki) == np.size(self.Kd)):
            raise TypeError('P, I, D must have the same shape')
        
        if np.size(self.setPoint) != 1 and np.size(self.Kp) != 1:
            if np.size(self.Kp) != np.size(self.setPoint):
                raise TypeError('PID gains must match target shape')
        
        # State variables
        self.last_error = 0.0
        self.integrator = 0.0
        self.timeOfLastCall = None
        self.integrator_max = float('inf')
    
    def update(self, current_value):
        """Update PID controller and return control signal.
        
        Args:
            current_value: Current measured value(s) - scalar or numpy array
            
        Returns:
            Control signal - same shape as target
        """
        current_value = np.array(current_value, dtype=float)
        
        if np.size(current_value) != np.size(self.setPoint):
            raise TypeError('current_value and target must have the same shape')
        
        # First call - initialize timing
        if self.timeOfLastCall is None:
            self.timeOfLastCall = time.perf_counter()
            return np.zeros(np.size(current_value))
        
        # Calculate error
        error = self.setPoint - current_value
        
        # Proportional term
        P = error
        
        # Calculate time delta
        currentTime = time.perf_counter()
        deltaT = currentTime - self.timeOfLastCall
        
        # Integral term (with anti-windup)
        self.integrator = self.integrator + (error * deltaT)
        self.integrator = np.clip(self.integrator, -self.integrator_max, self.integrator_max)
        I = self.integrator
        
        # Derivative term
        D = (error - self.last_error) / deltaT if deltaT > 0 else 0.0
        self.last_error = error
        self.timeOfLastCall = currentTime
        
        # Return control signal
        return self.Kp * P + self.Ki * I + self.Kd * D
    
    def reset(self):
        """Reset PID controller state."""
        self.last_error = 0.0
        self.integrator = 0.0
        self.timeOfLastCall = None


def linear(point1, point2):
    """Calculate linear interpolation coefficients.
    
    Given two points (x1, y1) and (x2, y2), returns [slope, intercept]
    for the line y = slope*x + intercept.
    
    Args:
        point1: [x1, y1]
        point2: [x2, y2]
        
    Returns:
        [slope, intercept] as list
    """
    x1, y1 = point1
    x2, y2 = point2
    
    if x2 == x1:
        return [0.0, y1]  # Vertical line - return constant
    
    slope = (y2 - y1) / (x2 - x1)
    intercept = y1 - slope * x1
    
    return [slope, intercept]


def clamp(value, min_val, max_val):
    """Clamp value to range [min_val, max_val].
    
    Args:
        value: Input value (scalar or array)
        min_val: Minimum value
        max_val: Maximum value
        
    Returns:
        Clamped value
    """
    return np.clip(value, min_val, max_val)
