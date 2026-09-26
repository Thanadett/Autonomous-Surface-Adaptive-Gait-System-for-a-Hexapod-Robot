# Worlds

`flat_world.sdf`, `hexapod_training_course.sdf` (slope + body-clearance
boxes + zigzag pylons), and `obstacle_course.sdf` are carried over from
49x_Hexapod_test unchanged -- they're a reasonable manual test course, not
what the dataset pipeline trains on.

**P3 (Dataset) still needs:** a procedural terrain generator that produces
many world variants across the curriculum in the project plan (slope
0-25 deg, heightfield roughness 0-40 mm, step/gap 0-50 mm, domain
randomization on friction/servo delay/sensor noise/camera tilt) plus a
headless sim-runner script that drives it. Neither exists yet -- these
three hand-built worlds are only enough to smoke-test locomotion and the
sensor bridge (P1/P2), not to train the gait-selection model.
