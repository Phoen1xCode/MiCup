"""Red limit-pole detector node."""

from perception._color_detector_node import run_object_detector


def main(args=None):
    run_object_detector("perception_red_pole", "/perception/red_pole",
                        "red_pole", "red_pole", real_width_m=0.1, args=args)


if __name__ == "__main__":
    main()
