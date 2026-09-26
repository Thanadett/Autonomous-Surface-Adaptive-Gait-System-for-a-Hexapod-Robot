std_msgs/Header header
string active_gait
string transition_state     # standing | walking | pending_switch | blending | safe_stop
float64 phase               # 0..1 within the current gait cycle
bool[6] leg_in_swing         # canonical leg order (see FootContacts); current phase, not the look-ahead
float64 ik_residual_max_m    # worst per-leg IK residual this tick
float64 stability_margin_m   # planned SSM: signed distance, CoM projection to support polygon edge (m)
float64 stance_height_m      # current body height command after the ramp (m)
float64 step_height_m        # current swing apex height after the ramp and the reach clamp (m)
float64 cadence_scale        # cycle-time stretch for this step/body height (1.0 = gait.yaml cycle)
bool posture_control         # IMU posture loop enabled
float64 posture_roll_rad     # body-levelling correction applied (after the reach limit)
float64 posture_pitch_rad
bool estop
