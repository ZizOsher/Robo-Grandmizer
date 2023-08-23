# robot.inc
define pioneer2dx position
(
    # Robot dimensions
    size [0.2 0.2 0.2]

    # Robot base color
    color "blue"
    gui_nose 1

    # Enable the position interface
    position
    (
        localization "odom"
        localization_origin [-8 -14 0 0]
    )

    alwayson 1
    sicklaser()
)

