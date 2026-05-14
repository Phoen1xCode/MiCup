"""Orange ball detector node."""

from perception._color_detector_node import run_object_detector


def main(args=None):
    run_object_detector("perception_orange_ball", "/perception/orange_ball",
                        "orange_ball", "orange_ball", real_width_m=0.2, args=args)


if __name__ == "__main__":
    main()
