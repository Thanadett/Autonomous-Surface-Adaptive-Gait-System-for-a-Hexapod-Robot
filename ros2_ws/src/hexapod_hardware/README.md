# hexapod_hardware -- P6, not yet implemented

No `hardware_interface::SystemInterface` plugin exists yet -- this is the
one component in the workspace that can command real motors, so it is
deliberately not stubbed out with fake no-op read()/write() methods (see
`include/hexapod_hardware/hexapod_system.hpp`). What's here is the
planned header, the empty `config/calibration.yaml` schema, and this
README.

Do not implement this until real servo, limit-switch, and IMU hardware
are chosen (see the project plan's open-questions table, items #2-5) --
the interface design depends on which servo bus (PWM vs. a serial bus
with position feedback) is used.
