"""Lane edge detector node."""

from perception._color_detector_node import run_scalar_detector


def main(args=None):
    run_scalar_detector("perception_lane_edge", "/perception/lane_edge", args=args)


if __name__ == "__main__":
    main()
