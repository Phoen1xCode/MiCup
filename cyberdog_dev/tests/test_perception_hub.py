from perception.hub import CorridorState


def test_corridor_state_fields():
    c = CorridorState(left=1.0, front=2.0, right=1.5)
    assert c.left == 1.0
    assert c.front == 2.0
    assert c.right == 1.5


def test_corridor_state_default_is_no_return():
    # 默认值应是"很远"，让 Stage 在没数据时不会误判为撞墙
    c = CorridorState()
    assert c.left > 10.0
    assert c.front > 10.0
    assert c.right > 10.0
