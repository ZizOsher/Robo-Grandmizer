# robot.inc
define pioneer2dx model
(
    # Robot dimensions
    size [0.5 0.5 0.5]

    # Robot base color
    color "blue"

    # Enable the position interface
    position
    (
        localization "odom"
        localization_origin [4.37 -4.56 0 0]
    )

    # Add a laser range finder
    ranger
    (
        samples 180
        range_max 10.0
        fov 180.0
    )
)

