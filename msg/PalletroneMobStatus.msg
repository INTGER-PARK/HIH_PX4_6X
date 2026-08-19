# Second-order momentum-observer estimate at the modeled Palletrone CoM/body origin.
# All vectors use the PX4 FRD body frame (x forward, y right, z down).
# Positive external wrench means a wrench acting on the vehicle.

uint64 timestamp                         # [us] publication time since PX4 boot
uint64 timestamp_sample                  # [us] vehicle_odometry sample time

uint8 FRAME_BODY_FRD = 1
uint8 frame                              # FRAME_BODY_FRD

bool force_valid
bool torque_valid
bool initialized
bool warmup_complete

uint32 FLAG_ENABLED = 1
uint32 FLAG_ODOM_VALID = 2
uint32 FLAG_ATTITUDE_VALID = 4
uint32 FLAG_VELOCITY_VALID = 8
uint32 FLAG_ACTUATOR_FRESH = 16
uint32 FLAG_SERVO_FRESH = 32
uint32 FLAG_ARMED = 64
uint32 FLAG_LANDED = 128
uint32 FLAG_WARMUP = 256
uint32 FLAG_ESTIMATOR_RESET = 512
uint32 FLAG_DT_INVALID = 1024
uint32 FLAG_NONFINITE_INPUT = 2048
uint32 FLAG_PARAMETER_INVALID = 4096
uint32 status_flags
uint32 reset_count

float32[3] external_force                # [N] raw external/residual force, BODY_FRD, modeled CoM/body origin
float32[3] external_torque               # [N m] raw external/residual torque, BODY_FRD, modeled CoM/body origin
float32[3] external_force_corrected      # [N] external_force - PMOB_BIAS_F*, BODY_FRD, modeled CoM/body origin
float32[3] external_torque_corrected     # [N m] external_torque - PMOB_BIAS_T*, BODY_FRD, modeled CoM/body origin
float32[3] known_actuator_force          # [N] command-based rotor force, FRD
float32[3] known_actuator_torque         # [N m] command-based rotor torque about modeled CoM, FRD
float32[3] linear_momentum_error          # [kg m/s] measured minus estimated momentum, FRD
float32[3] angular_momentum_error         # [kg m^2/s] measured minus estimated angular momentum, FRD

float32 dt                               # [s] accepted observer sample interval
float32 odometry_age_s                   # [s] age based on local uORB reception time, -1 before first sample
float32 actuator_age_s                   # [s] age of physical thrust_command, -1 before first sample
float32 servo_age_s                      # [s] age of actual servo_angle, -1 before first sample
