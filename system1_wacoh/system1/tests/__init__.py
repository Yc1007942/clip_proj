import sys
import types


def _ensure_segment_task_pkg_stub() -> None:
	"""Provide lightweight stand-ins for segment_task_pkg messages and actions during tests."""
	if 'segment_task_pkg' in sys.modules:
		return

	segment_pkg = types.ModuleType('segment_task_pkg')
	msg_module = types.ModuleType('segment_task_pkg.msg')
	action_module = types.ModuleType('segment_task_pkg.action')

	class _Pose:
		def __init__(self) -> None:
			self.position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
			self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)

	class _PoseStamped:
		def __init__(self) -> None:
			self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0))
			self.pose = _Pose()

	class PoseWithStiffness:
		def __init__(self) -> None:
			self.pose = types.SimpleNamespace()
			self.pose.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0))
			self.pose.pose = _Pose()
			self.stiffness_id = 0

	class SegmentTaskCmd:
		OPEN_END_EFFECTOR = 12
		CLOSE_END_EFFECTOR = 13

	class SegmentTaskStatus:
		SUCCESS = 1
		CONTROLLER_PROBLEMS = 2
		STOPPED = 3

	class _BaseAction:
		class Goal:  # Minimal placeholders required by type checks
			pass

		class Result:
			pass

	class ExecuteMoveHandWithStiffness(_BaseAction):
		pass

	class ExecuteSegmentTask(_BaseAction):
		pass

	msg_module.PoseWithStiffness = PoseWithStiffness
	msg_module.SegmentTaskCmd = SegmentTaskCmd
	msg_module.SegmentTaskStatus = SegmentTaskStatus
	action_module.ExecuteMoveHandWithStiffness = ExecuteMoveHandWithStiffness
	action_module.ExecuteSegmentTask = ExecuteSegmentTask

	segment_pkg.msg = msg_module
	segment_pkg.action = action_module

	sys.modules['segment_task_pkg'] = segment_pkg
	sys.modules['segment_task_pkg.msg'] = msg_module
	sys.modules['segment_task_pkg.action'] = action_module


_ensure_segment_task_pkg_stub()
