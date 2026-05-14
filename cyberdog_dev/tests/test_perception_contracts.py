from perception.hub import (
    BallDet,
    CorridorState,
    DashedLineDet,
    LaneEdges,
    ObjDet,
    PerceptionHub,
    PoleDet,
)


def test_detection_dataclasses_keep_geometry_fields():
    ball = BallDet(
        bbox=(10, 20, 30, 40),
        center_px=(25.0, 40.0),
        area_px=800.0,
        bearing_rad=0.1,
        distance_m=0.7,
        confidence=0.9,
    )
    assert ball.bearing_rad == 0.1
    assert ball.distance_m == 0.7

    obj = ObjDet(label="coke", bbox=(1, 2, 3, 4), center_px=(2.5, 4.0), area_px=12.0)
    assert obj.label == "coke"

    pole = PoleDet(bbox=(0, 0, 10, 100), center_px=(5.0, 50.0), area_px=1000.0,
                   bearing_rad=0.0, confidence=0.8)
    assert pole.aspect_ratio == 10.0


def test_perception_hub_defaults_are_empty_or_safe():
    hub = PerceptionHub()
    assert hub.latest_orange_balls() == []
    assert hub.latest_red_poles() == []
    assert hub.latest_footballs() == []
    assert hub.latest_coke_bottles() == []
    assert hub.latest_block_obstacles() == []
    assert hub.latest_dashed_line() is None
    assert isinstance(hub.latest_lane_edges(), LaneEdges)
    assert isinstance(hub.latest_lidar_corridor(), CorridorState)


def test_lane_and_dashed_defaults():
    lane = LaneEdges()
    assert lane.left_offset_px == 0.0
    assert lane.right_offset_px == 0.0
    assert lane.confidence == 0.0

    dashed = DashedLineDet(center_px=(50.0, 80.0), confidence=0.7)
    assert dashed.center_px == (50.0, 80.0)
