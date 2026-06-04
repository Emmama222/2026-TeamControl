from TeamControl.planner import PlannerAPI, PlannerInput, plan


def test_planner_api_returns_planner_output():
    api = PlannerAPI()

    output = api.plan(
        PlannerInput(
            robot_id=0,
            is_yellow=True,
            current_pose=(0.0, 0.0, 0.0),
            target_pose=(1000.0, 0.0, 0.0),
        )
    )

    assert output.is_path_free is True
    assert output.active_target_pose == (1000.0, 0.0, 0.0)


def test_plan_helper_accepts_one_shot_input():
    output = plan(
        PlannerInput(
            robot_id=0,
            is_yellow=True,
            current_pose=(0.0, 0.0),
            target_pose=(1000.0, 0.0),
        )
    )

    assert output.active_target_pose == (1000.0, 0.0, 0.0)
