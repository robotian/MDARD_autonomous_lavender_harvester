"""stuck_again: a second navigation failure from the same spot means a stuck robot."""
from status_server.job_publisher import stuck_again


def test_stuck_again():
    assert stuck_again((7.69, -8.97), (7.70, -8.95))
    assert not stuck_again((8.57, -8.33), (7.69, -8.97))
    assert not stuck_again(None, (7.69, -8.97))
    assert not stuck_again((7.69, -8.97), None)


def test_second_nav_failure_from_the_same_spot_pauses_the_robot():
    from unittest.mock import MagicMock
    from status_interfaces.msg import Task
    from status_server.enum import TaskEnum
    from status_server.job_publisher import JobPublisher

    jp = object.__new__(JobPublisher)  # no node: only the failure bookkeeping is under test
    jp.get_logger = MagicMock()
    jp.db_ops, jp.task_generator = MagicMock(), MagicMock()
    jp.task_generator._robot_pose.return_value = (7.69, -8.97, 0.0)
    jp.failed_task_handled, jp.nav_failure_pose, jp.robot_may_take_work = {}, {}, {'j100_0921': True}
    jp.current_task, jp.last_published_task_id = {}, {}
    jp.assign_next_task = MagicMock()
    robot, op = MagicMock(id=7), MagicMock(max_attempts=3)
    for task_id in (642, 643):
        jp.current_task['j100_0921'] = Task(task_id=task_id, task_type=TaskEnum.HARVESTING_TASK.value)
        jp.handle_task_failure('j100_0921', robot, MagicMock(task=f'FAILED:nav:{task_id}:3 retries'), op)
    jp.db_ops.fail_plant_attempt.assert_called_once()           # 642: the plant's attempt
    jp.db_ops.release_plant.assert_called_once_with(643)          # 643: the robot's fault, no attempt
    assert jp.robot_may_take_work['j100_0921'] is False
    assert "state = 'paused'" in jp.db_ops.db.execute_write.call_args[0][0]
