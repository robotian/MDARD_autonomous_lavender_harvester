#!/usr/bin/env python3
"""The robot's manipulator_action_server (status_interfaces/action/ManipulatorTask): the arm commands
husky_operations_manager sends during a harvesting operation.

  GO STOW / GO READY / GO DROP   move_group to the SRDF state stow_state / ready_state / drop_state (the SRDF is
                                 /etc/clearpath/robot.srdf, generated at boot from robot.yaml's poses + the arm's
                                 built-ins), the same goal robot/bin/arm_goto sends
  START HARVEST                  stow_arm_cpp's cut_stem action (the grid cutter; it stows the arm itself when it
                                 succeeds); result.harvested_count / total_targets = its "Cut n/m patches"
  MOVE EEF, anything else        accepted and failed at once (result.success false): not supported

One goal at a time, with one exception: GO STOW preempts whatever runs (a cut, a move) -- the running goal's
sub-goal is cancelled, that goal ends aborted ("preempted"), then the stow runs. So a restarted
husky_operations_manager (its first act is a stow) can always take the arm back, also from a cut its previous
life started. A cancelled or failed cut leaves the arm wherever the cutter's safe_state put it (lifted, gripper
open); a failed cut is followed by a stow before the goal is reported.

Feedback status: PLANNING when the sub-goal is sent, MOVING once accepted (the cutter's state text goes to the
log), MOVING_COMPLETE before success, FAILED before an abort. Timeouts (move_timeout, harvest_timeout) run on the
node clock: sim time with use_sim_time, so a paused simulation doesn't time a goal out.
"""
import re
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from plant_cutter_msgs.action import CutStem
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from status_interfaces.action import ManipulatorTask

from moveit_sim_bridge.arm_goto import joint_goal, load_states

FB = ManipulatorTask.Feedback
GO_STOW, GO_READY, GO_DROP, START_HARVEST = "GO STOW", "GO READY", "GO DROP", "START HARVEST"
MOVEIT_CODES = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}


class ArmTaskServer(Node):
    def __init__(self):
        super().__init__("arm_task_server")
        p = self.declare_parameters("", [
            ("arm_group", "arm_0"), ("srdf_path", "/etc/clearpath/robot.srdf"),
            ("stow_state", "stow"), ("ready_state", "ready"), ("drop_state", "drop"),
            ("velocity_scale", 0.3), ("move_group_action", "move_action"), ("cut_stem_action", "cut_stem"),
            ("server_wait_timeout", 10.0), ("move_timeout", 90.0), ("harvest_timeout", 900.0),
            ("feedback_period", 1.0),
        ])
        self.p = {d.name: d.value for d in p}
        self.cb = ReentrantCallbackGroup()
        self.move_client = ActionClient(self, MoveGroup, self.p["move_group_action"], callback_group=self.cb)
        self.cut_client = ActionClient(self, CutStem, self.p["cut_stem_action"], callback_group=self.cb)
        self._lock = threading.Lock()  # guards _active / _sub / _preempt
        self._exec_lock = threading.Lock()  # one execute at a time (a preempting stow waits for the old one to end)
        self._active = None  # the ManipulatorTask goal handle running now
        self._sub = None  # its sub-goal handle (move_group or cut_stem), to cancel
        self._preempt = False  # a GO STOW is waiting: the active goal must stop
        self._sub_result = None  # (label, status, result) of the last sub-goal
        self.server = ActionServer(self, ManipulatorTask, "manipulator_action_server",
                                   execute_callback=self.execute, goal_callback=self.on_goal,
                                   cancel_callback=self.on_cancel, callback_group=self.cb)
        self.get_logger().info(
            f"manipulator_action_server ready: stow={self.p['stow_state']!r} ready={self.p['ready_state']!r} "
            f"drop={self.p['drop_state']!r} (group {self.p['arm_group']}, {self.p['srdf_path']}); "
            f"harvest = {self.p['cut_stem_action']} action")

    # ---------------------------------------------------------------- goal policy

    def on_goal(self, request):
        task = request.arm_task.strip().upper()
        with self._lock:
            if self._active is not None:
                if task == GO_STOW:
                    self.get_logger().warning(f"GO STOW preempts the running {self._active.request.arm_task!r}")
                    self._preempt = True
                    if self._sub is not None:
                        self._sub.cancel_goal_async()
                    return GoalResponse.ACCEPT
                self.get_logger().warning(f"rejecting {request.arm_task!r}: busy with "
                                          f"{self._active.request.arm_task!r}")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT  # unsupported commands are accepted and aborted with a clear result

    def on_cancel(self, goal_handle):
        with self._lock:
            if self._sub is not None and self._active is goal_handle:
                self._sub.cancel_goal_async()
        return CancelResponse.ACCEPT

    # ---------------------------------------------------------------- execution

    def execute(self, goal_handle):
        task = goal_handle.request.arm_task.strip().upper()
        with self._exec_lock:
            with self._lock:
                self._active, self._sub, self._preempt = goal_handle, None, False
            try:
                if task == GO_STOW:
                    return self._named_move(goal_handle, self.p["stow_state"], required=True)
                if task == GO_READY:
                    return self._named_move(goal_handle, self.p["ready_state"])
                if task == GO_DROP:
                    return self._named_move(goal_handle, self.p["drop_state"])
                if task == START_HARVEST:
                    return self._harvest(goal_handle)
                return self._finish(goal_handle, False, f"unsupported arm_task {goal_handle.request.arm_task!r}")
            except Exception as e:  # never take the server down with a goal
                self.get_logger().error(f"{task}: {e!r}")
                return self._finish(goal_handle, False, repr(e))
            finally:
                with self._lock:
                    self._active, self._sub = None, None

    def _finish(self, goal_handle, ok, message, harvested=0, total=0, canceled=False):
        result = ManipulatorTask.Result(success=ok, harvested_count=harvested, total_targets=total)
        if canceled and goal_handle.is_cancel_requested:
            self.get_logger().info(f"{goal_handle.request.arm_task}: canceled ({message})")
            goal_handle.canceled()
        elif ok:
            self.get_logger().info(f"{goal_handle.request.arm_task}: done ({message})")
            goal_handle.publish_feedback(FB(status=FB.MOVING_COMPLETE))
            goal_handle.succeed()
        else:
            self.get_logger().error(f"{goal_handle.request.arm_task}: failed ({message})")
            goal_handle.publish_feedback(FB(status=FB.FAILED))
            goal_handle.abort()
        return result

    def _stopped(self, goal_handle):
        return goal_handle.is_cancel_requested or self._preempt

    def _run(self, goal_handle, client, goal, timeout, label, feedback=None):
        """Send `goal` to `client` and wait for it: ('done', status, result) | ('rejected' | 'unavailable' |
        'timeout' | 'canceled' | 'preempted', None, None). Cancels it on a cancel request, a preempting GO STOW,
        or when `timeout` (node clock) runs out."""
        if not client.wait_for_server(timeout_sec=self.p["server_wait_timeout"]):
            return "unavailable", None, None
        goal_handle.publish_feedback(FB(status=FB.PLANNING))
        send = client.send_goal_async(goal, feedback_callback=feedback)
        t0 = time.monotonic()
        while not send.done():
            if time.monotonic() - t0 > self.p["server_wait_timeout"]:
                return "unavailable", None, None
            time.sleep(0.05)
        handle = send.result()
        if handle is None or not handle.accepted:
            return "rejected", None, None
        with self._lock:
            self._sub = handle
        goal_handle.publish_feedback(FB(status=FB.MOVING))
        res = handle.get_result_async()
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        last_fb, outcome = time.monotonic(), None
        while not res.done():
            if outcome is None:
                if self._stopped(goal_handle):
                    outcome = "preempted" if self._preempt and not goal_handle.is_cancel_requested else "canceled"
                elif self.get_clock().now().nanoseconds > deadline:
                    outcome = "timeout"
                if outcome:
                    self.get_logger().warning(f"{label}: {outcome}, cancelling it")
                    handle.cancel_goal_async()
                    t_cancel = self.get_clock().now().nanoseconds
            # the sub-server never answered the cancel: stop waiting. Node clock, and longer than the cutter's own
            # safe-state (gripper + servo timeouts): letting go earlier would stow while it still moves the arm
            elif self.get_clock().now().nanoseconds - t_cancel > 60e9:
                break
            if time.monotonic() - last_fb > self.p["feedback_period"]:
                goal_handle.publish_feedback(FB(status=FB.MOVING))
                last_fb = time.monotonic()
            time.sleep(0.1)
        with self._lock:
            self._sub = None
        if outcome:
            return outcome, None, None
        r = res.result()
        return "done", r.status, r.result

    def _named_move(self, goal_handle, state, required=False, label=None):
        """move_group to the SRDF state `state` ('' = nothing to do); returns the goal's result."""
        label = label or goal_handle.request.arm_task
        if not state:
            return self._finish(goal_handle, True, f"{label}: no state configured, nothing to do")
        ok, message = self._move_to(goal_handle, state, label)
        if not ok and not required and message.startswith("unknown state"):
            self.get_logger().warning(f"{label}: {message}; treating as done")
            return self._finish(goal_handle, True, message)
        return self._finish(goal_handle, ok, message, canceled=not ok)

    def _move_to(self, goal_handle, state, label):
        """(ok, message) of one named move; the arm stays where it is on failure."""
        try:
            target = load_states(self.p["srdf_path"]).get(self.p["arm_group"], {}).get(state)
        except Exception as e:
            return False, f"can't read {self.p['srdf_path']}: {e}"
        if target is None:
            return False, f"unknown state {state!r} for group {self.p['arm_group']!r}"
        outcome, status, result = self._run(goal_handle, self.move_client,
                                            joint_goal(self.p["arm_group"], target, self.p["velocity_scale"]),
                                            self.p["move_timeout"], f"{label} -> {state}")
        if outcome != "done":
            return False, f"move to {state!r}: {outcome}"
        if status != GoalStatus.STATUS_SUCCEEDED or result.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, f"move to {state!r}: move_group {MOVEIT_CODES.get(result.error_code.val, result.error_code.val)}"
        return True, f"at {state!r}"

    def _harvest(self, goal_handle):
        states = {}

        def on_feedback(msg):
            text = msg.feedback.current_state
            if text and text != states.get("last"):
                states["last"] = text
                self.get_logger().info(f"cut_stem: {text}")

        outcome, status, result = self._run(goal_handle, self.cut_client, CutStem.Goal(start_cutting=True),
                                            self.p["harvest_timeout"], "cut_stem", on_feedback)
        if outcome == "rejected":
            return self._finish(goal_handle, False, "cut_stem busy (another cut is running)")
        if outcome == "unavailable":
            return self._finish(goal_handle, False, "cut_stem action server not available")
        if outcome in ("canceled", "preempted"):
            return self._finish(goal_handle, False, f"cut_stem {outcome}", canceled=True)
        if outcome == "timeout" or status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            message = "cut_stem timed out" if outcome == "timeout" else f"cut_stem failed: {result.message}"
            # the cutter's safe_state left the arm lifted with the gripper open: put it away before reporting
            if not self._stopped(goal_handle):
                ok, stow_msg = self._move_to(goal_handle, self.p["stow_state"], "stow after a failed cut")
                message += f"; {stow_msg}" if ok else f"; stow failed too: {stow_msg}"
            return self._finish(goal_handle, False, message)
        m = re.search(r"Cut (\d+)/(\d+)", result.message or "")
        n, total = (int(m.group(1)), int(m.group(2))) if m else (1, 1)
        return self._finish(goal_handle, True, result.message or "cut done", harvested=n, total=total)


def main():
    rclpy.init()
    node = ArmTaskServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    # Not executor.spin(): under use_sim_time it never returns after SIGTERM/SIGINT (the signal shuts the context down,
    # the timers run on the ROS clock and /clock stops with the context, so nothing wakes the executor's wait).
    # spin_once with a wall-time timeout re-checks rclpy.ok() at least every 0.5 s.
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
