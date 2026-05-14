"""Dashed line detector node."""

from perception._color_detector_node import run_scalar_detector


def main(args=None):
    run_scalar_detector("perception_dashed_line", "/perception/dashed_line", args=args)


if __name__ == "__main__":
    main()
