# Fused terrain + body-state feature vector, published at gait_selector's
# input rate (10 Hz). hexapod_gait_selector must use exactly this message
# for both simulated and real inference so there is no train/serve skew
# between the feature code path used offline and the one used online.
std_msgs/Header header

# From hexapod_perception (depth camera -> plane fit over the ROI ahead of
# the robot).
float64 slope_pitch_rad
float64 slope_roll_rad
float64 roughness_m        # residual std-dev of points off the fitted plane
float64 max_step_height_m  # largest discontinuity in the elevation grid

# From hexapod_state_estimation (IMU + joint states + foot contacts).
float64 body_roll_rad
float64 body_pitch_rad
float64 body_roll_rate_rad_s
float64 body_pitch_rate_rad_s
float64 tilt_variance       # rolling variance of roll/pitch over ~1 s
float64 contact_ratio       # fraction of legs in stance, averaged over ~1 s
float64 slip_index          # 0 = no slip, higher = more foot slip detected

bool valid                  # false while any input sensor is still warming up
