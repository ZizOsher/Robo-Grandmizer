# robot.inc
define pioneer2dx model
(
    # Robot dimensions
    size [0.3 0.3 0.5]

    # Robot base color
    color "blue"

    # Enable the position interface
    position
    (
        localization "odom"
        localization_origin [-8 -14 0 0]
    )

    # Add a laser range finder
    ranger
    (
        samples 180
        range_max 10.0
        fov 180.0
    )
)

