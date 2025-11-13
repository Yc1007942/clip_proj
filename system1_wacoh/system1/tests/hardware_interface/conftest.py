import sys
from unittest.mock import MagicMock
sys.modules['rtde_control'] = MagicMock()
sys.modules['rtde_receive'] = MagicMock()