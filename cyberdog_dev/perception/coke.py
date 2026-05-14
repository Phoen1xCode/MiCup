"""Coke bottle detector node."""

from perception._color_detector_node import run_object_detector


def main(args=None):
    run_object_detector("perception_coke", "/perception/coke",
                        "coke", "coke", real_width_m=0.12, args=args)


if __name__ == "__main__":
    main()
