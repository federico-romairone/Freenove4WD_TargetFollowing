"""
Unit tests for PID Controller
"""

import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from pid_controller import PIDController


class TestPIDController(unittest.TestCase):
    """Test cases for PID controller."""
    
    def test_initialization(self):
        """Test PID controller initialization."""
        controller = PIDController(1.0, 0.1, 0.01, output_limit=10.0)
        
        self.assertEqual(controller.Kp, 1.0)
        self.assertEqual(controller.Ki, 0.1)
        self.assertEqual(controller.Kd, 0.01)
        self.assertEqual(controller.output_limit, 10.0)
        self.assertEqual(controller.integral, 0.0)
        self.assertEqual(controller.prev_error, 0.0)
    
    def test_proportional_only(self):
        """Test pure proportional control."""
        controller = PIDController(Kp=2.0, Ki=0.0, Kd=0.0, output_limit=100.0)
        
        error = 5.0
        output = controller.compute(error, dt=0.1)
        
        # Output should be Kp * error
        expected = 2.0 * 5.0
        self.assertAlmostEqual(output, expected, places=5)
    
    def test_integral_accumulation(self):
        """Test integral term accumulation."""
        controller = PIDController(Kp=0.0, Ki=1.0, Kd=0.0, output_limit=100.0)
        
        # Apply constant error for multiple steps
        error = 2.0
        dt = 0.1
        
        output1 = controller.compute(error, dt)
        output2 = controller.compute(error, dt)
        output3 = controller.compute(error, dt)
        
        # Integral should accumulate
        self.assertAlmostEqual(output1, 2.0 * 0.1, places=5)
        self.assertAlmostEqual(output2, 2.0 * 0.2, places=5)
        self.assertAlmostEqual(output3, 2.0 * 0.3, places=5)
    
    def test_derivative_term(self):
        """Test derivative term calculation."""
        controller = PIDController(Kp=0.0, Ki=0.0, Kd=1.0, output_limit=100.0)
        
        # Apply changing error
        dt = 0.1
        output1 = controller.compute(1.0, dt)  # error = 1.0
        output2 = controller.compute(2.0, dt)  # error = 2.0, change = 1.0
        
        # Derivative term should be Kd * (change in error / dt)
        expected = 1.0 * (1.0 / 0.1)  # Kd * derivative
        self.assertAlmostEqual(output2, expected, places=5)
    
    def test_output_limiting(self):
        """Test output limiting."""
        controller = PIDController(Kp=10.0, Ki=0.0, Kd=0.0, output_limit=50.0)
        
        # Large error should be limited
        error = 100.0
        output = controller.compute(error, dt=0.1)
        
        self.assertEqual(output, 50.0)
        
        # Negative large error
        error = -100.0
        output = controller.compute(error, dt=0.1)
        
        self.assertEqual(output, -50.0)
    
    def test_anti_windup(self):
        """Test integral anti-windup."""
        controller = PIDController(Kp=0.0, Ki=1.0, Kd=0.0, output_limit=10.0)
        
        # Apply large error for many steps
        error = 100.0
        dt = 0.1
        
        for _ in range(100):
            output = controller.compute(error, dt)
        
        # Integral should be limited
        self.assertLessEqual(abs(controller.integral), 10.0 / 1.0)
    
    def test_reset(self):
        """Test controller reset."""
        controller = PIDController(Kp=1.0, Ki=1.0, Kd=1.0, output_limit=10.0)
        
        # Accumulate some state
        controller.compute(5.0, dt=0.1)
        controller.compute(5.0, dt=0.1)
        
        # Reset
        controller.reset()
        
        self.assertEqual(controller.integral, 0.0)
        self.assertEqual(controller.prev_error, 0.0)
    
    def test_set_gains(self):
        """Test setting PID gains."""
        controller = PIDController(Kp=1.0, Ki=0.1, Kd=0.01, output_limit=10.0)
        
        controller.set_gains(Kp=2.0, Ki=0.2, Kd=0.02)
        
        Kp, Ki, Kd = controller.get_gains()
        self.assertEqual(Kp, 2.0)
        self.assertEqual(Ki, 0.2)
        self.assertEqual(Kd, 0.02)
    
    def test_partial_gain_update(self):
        """Test updating only some gains."""
        controller = PIDController(Kp=1.0, Ki=0.1, Kd=0.01, output_limit=10.0)
        
        controller.set_gains(Kp=2.0)
        
        Kp, Ki, Kd = controller.get_gains()
        self.assertEqual(Kp, 2.0)
        self.assertEqual(Ki, 0.1)  # Unchanged
        self.assertEqual(Kd, 0.01)  # Unchanged


if __name__ == '__main__':
    unittest.main()
