from TeamControl.robot.voronoi_navigator import (
    _robot_is_in_front_of_possessor,
    _steal_ignore_keys,
)


class _Robots:
    def __init__(self, poses):
        self._poses = poses

    def iter_team(self, is_yellow):
        for (team_yellow, robot_id), pose in self._poses.items():
            if team_yellow == is_yellow:
                yield robot_id, pose

    def relative_to_ball(self, is_yellow, robot_id, ball):
        pose = self._poses[(is_yellow, robot_id)]
        from TeamControl.world.transform_cords import world2robot

        rel = world2robot(pose, ball)
        import math

        return rel, math.hypot(rel[0], rel[1]), math.atan2(rel[1], rel[0])


class _Cache:
    def __init__(self, poses):
        self.robots = _Robots(poses)


def test_robot_in_front_of_possessor_rule():
    possessor = (0.0, 0.0, 0.0)

    assert _robot_is_in_front_of_possessor((400.0, 0.0, 0.0), possessor)
    assert not _robot_is_in_front_of_possessor((-100.0, 0.0, 0.0), possessor)
    assert not _robot_is_in_front_of_possessor((400.0, 400.0, 0.0), possessor)


def test_steal_ignore_key_requires_possession_and_front_position():
    poses = {
        (True, 0): (350.0, 0.0, 3.14),
        (False, 1): (0.0, 0.0, 0.0),
    }
    cache = _Cache(poses)

    keys = _steal_ignore_keys(
        cache,
        is_yellow=True,
        robot_id=0,
        robot_pose=poses[(True, 0)],
        ball_pos=(80.0, 0.0),
    )

    assert keys == ((False, 1),)


def test_steal_ignore_key_rejects_loose_or_off_angle_ball():
    poses = {
        (True, 0): (350.0, 0.0, 3.14),
        (False, 1): (0.0, 0.0, 0.0),
    }
    cache = _Cache(poses)

    loose = _steal_ignore_keys(
        cache,
        is_yellow=True,
        robot_id=0,
        robot_pose=poses[(True, 0)],
        ball_pos=(95.0, 0.0),
    )
    off_angle = _steal_ignore_keys(
        cache,
        is_yellow=True,
        robot_id=0,
        robot_pose=poses[(True, 0)],
        ball_pos=(80.0, 2.0),
    )

    assert loose == ()
    assert off_angle == ()
