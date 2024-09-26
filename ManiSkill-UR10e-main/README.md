# Universal Robots UR10e Description (MJCF)

This is adapted from the original MJCF in the [Mujoco Menagerie Project](https://github.com/google-deepmind/mujoco_menagerie/blob/main/universal_robots_ur10e/README.md)


## Changes Made

- Made the capsule collision of `shoulder_link` smaller so that it doesn't hit the floor and cause the controller to explode due to collisions.