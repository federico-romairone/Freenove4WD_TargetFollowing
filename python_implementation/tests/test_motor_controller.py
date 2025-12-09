"""
Unit tests for Motor Controller
"""

import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from motor_controller import MotorController, SimulatedMotorController


class TestMotorController(unittest.TestCase):
    """Test cases for motor controller."""
    
    def test_initialization(self):
        """Test motor controller initialization."""
        controller = MotorController(max_speed=100, wheel_base=0.15)
        
        self.assertEqual(controller.max_speed, 100)
        self.assertEqual(controller.wheel_base, 0.15)
        self.assertEqual(controller.current_left_speed, 0)
        self.assertEqual(controller.current_right_speed, 0)
    
    def test_set_motor_speeds(self):
        """Test setting motor speeds."""
        controller = MotorController(max_speed=100)
        
        controller.set_motor_speeds(50, 60)
        
        left, right = controller.get_current_speeds()
        self.assertEqual(left, 50)
        self.assertEqual(right, 60)
    
    def test_speed_clamping(self):
        """Test that speeds are clamped to max_speed."""
        controller = MotorController(max_speed=100)
        
        # Test positive clamping
        controller.set_motor_speeds(150, 120)
        left, right = controller.get_current_speeds()
        self.assertEqual(left, 100)
        self.assertEqual(right, 100)
        
        # Test negative clamping
        controller.set_motor_speeds(-150, -120)
        left, right = controller.get_current_speeds()
        self.assertEqual(left, -100)
        self.assertEqual(right, -100)
    
    def test_stop(self):
        """Test stopping motors."""
        controller = MotorController(max_speed=100)
        
        controller.set_motor_speeds(50, 60)
        controller.stop()
        
        left, right = controller.get_current_speeds()
        self.assertEqual(left, 0)
        self.assertEqual(right, 0)
    
    def test_move_forward(self):
        """Test forward movement."""
        controller = MotorController(max_speed=100)
        
        controller.move_forward(50)
        
        left, right = controller.get_current_speeds()
        self.assertEqual(left, 50)
        self.assertEqual(right, 50)
    
    def test_move_backward(self):
        """Test backward movement."""
        controller = MotorController(max_speed=100)
        
        controller.move_backward(50)
        
        left, right = controller.get_current_speeds()
        self.assertEqual(left, -50)
        self.assertEqual(right, -50)
    
    def test_turn_left(self):
        """Test left turn."""
        controller = MotorController(max_speed=100)
        
        controller.turn_left(30)
        
        left, right = controller.get_current_speeds()
        self.assertEqual(left, -30)
        self.assertEqual(right, 30)
    
    def test_turn_right(self):
        """Test right turn."""
        controller = MotorController(max_speed=100)
        
        controller.turn_right(30)
        
        left, right = controller.get_current_speeds()
        self.assertEqual(left, 30)
        self.assertEqual(right, -30)


class TestSimulatedMotorController(unittest.TestCase):
    """Test cases for simulated motor controller."""
    
    def test_command_history(self):
        """Test that command history is recorded."""
        controller = SimulatedMotorController(max_speed=100)
        
        controller.set_motor_speeds(50, 60)
        controller.set_motor_speeds(40, 70)
        
        history = controller.get_command_history()
        
        self.assertEqual(len(history), 2)
        self.assertEqual(history[0]['left_speed'], 50)
        self.assertEqual(history[0]['right_speed'], 60)
        self.assertEqual(history[1]['left_speed'], 40)
        self.assertEqual(history[1]['right_speed'], 70)
    
    def test_history_timestamps(self):
        """Test that timestamps are recorded in history."""
        controller = SimulatedMotorController(max_speed=100)
        
        controller.set_motor_speeds(50, 60)
        
        history = controller.get_command_history()
        
        self.assertIn('time', history[0])
        self.assertIsInstance(history[0]['time'], float)


if __name__ == '__main__':
    unittest.main()
