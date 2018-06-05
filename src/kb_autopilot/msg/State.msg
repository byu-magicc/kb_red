# Vehicle state 'x_hat' output from the estimator
# The global origin is as the Brigham Young Statue just south of the ASB

Header header

# States
float32 p_north # north position (m)
float32 p_east  # east position (m)
float32 psi     # unwrapped yaw angle (rad)
float32 u       # body fixed forward velocity (m/s)
float32 b_u     # velocity bias
float32 L       # length between axles on the vehicle
float32 a       # servo to steering angle scale
float32 b_s     # servo steering angle bias

# Additional States for convenience
float32 psi_deg		# Wrapped yaw angle (deg)
