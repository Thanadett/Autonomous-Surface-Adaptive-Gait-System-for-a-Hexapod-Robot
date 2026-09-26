#ifndef HEXAPOD_HARDWARE__HEXAPOD_SYSTEM_HPP_
#define HEXAPOD_HARDWARE__HEXAPOD_SYSTEM_HPP_

// P6 stub -- not yet implemented. See the project plan section 10 for the
// intended design: a hardware_interface::SystemInterface exposing the
// same 18 position command/state interfaces as Gazebo's GazeboSimSystem
// (so hexapod_locomotion needs zero changes to run on hardware), plus six
// extra state interfaces ("<leg>/contact") reading real limit switches
// through a GPIO component, servo mapping via
// hexapod_description/config/joint_limits.yaml's servo_mapping block, an
// e-stop hardware input, and a >100ms-no-command watchdog that holds
// position rather than going slack.
//
// Deliberately not stubbed out as a fake SystemInterface here: a
// SystemInterface that "builds" but silently no-ops read()/write() would
// be far more dangerous to leave lying around than an interface that
// does not exist yet, since it is the one component in this workspace
// that can command real motors. Implement this only once real servo/
// limit-switch hardware is chosen (see the project plan's open-questions
// table, #4 and #5).

#endif  // HEXAPOD_HARDWARE__HEXAPOD_SYSTEM_HPP_
