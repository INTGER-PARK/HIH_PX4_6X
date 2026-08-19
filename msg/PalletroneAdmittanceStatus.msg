# CoM/body-origin 4-DoF admittance status. Force/torque and q/dq/ddq use body FRD; position setpoints use local NED.
uint64 timestamp
uint64 timestamp_sample
uint8 STATE_DISABLED_HOLD = 0
uint8 STATE_WAIT_VALID = 1
uint8 STATE_WAIT_CONTACT = 2
uint8 STATE_BLEND_IN = 3
uint8 STATE_ACTIVE = 4
uint8 STATE_HANDOVER_DECEL = 5
uint8 STATE_FAULT_HOLD = 6
uint8 state
uint32 FLAG_MASTER_ENABLED = 1
uint32 FLAG_ENABLE_REQUESTED = 2
uint32 FLAG_COMMAND_FRESH = 4
uint32 FLAG_COMMAND_TIMEOUT = 8
uint32 FLAG_MOB_FRESH = 16
uint32 FLAG_FORCE_VALID = 32
uint32 FLAG_TORQUE_VALID = 64
uint32 FLAG_CONTACT = 128
uint32 FLAG_ACTIVE = 256
uint32 FLAG_HANDOVER = 512
uint32 FLAG_HOLD_LATCHED = 1024
uint32 FLAG_CONTACT_LOST = 2048
uint32 FLAG_PARAMETER_INVALID = 4096
uint32 FLAG_DT_INVALID = 8192
uint32 FLAG_NONFINITE_INPUT = 16384
uint32 FLAG_INPUT_LIMIT = 32768
uint32 FLAG_ACCEL_LIMIT = 65536
uint32 FLAG_VELOCITY_LIMIT = 131072
uint32 FLAG_POSITION_LIMIT = 262144
uint32 FLAG_FAULT_HOLD = 524288
bool master_enabled
bool enable_requested
bool command_fresh
bool active
bool contact
bool translation_valid
bool yaw_valid
bool handover_active
bool hold_latched
uint32 status_flags
uint32 reset_count
uint32 command_sequence
uint32 command_reset_counter
float32 normal_force              # [N]
float32 normal_force_desired      # [N]
float32 normal_error              # [N]
float32[4] external_wrench        # [Fx,Fy,Fz,Tz] [N,N,N,N m], body FRD at modeled CoM
float32[4] admittance_input       # [N,N,N,N m]
float32[4] displacement           # [m,m,m,rad], body-aligned generalized state
float32[4] velocity               # [m/s,m/s,m/s,rad/s], body FRD
float32[4] acceleration           # [m/s^2,m/s^2,m/s^2,rad/s^2], body FRD
float32[3] local_position_offset  # [m], local NED
float32 local_yaw_offset          # [rad], continuous
float32[3] hold_position          # [m], local NED
float32 hold_yaw                  # [rad]
float32[3] effective_position_setpoint # [m], local NED
float32[3] effective_velocity_setpoint # [m/s], local NED
float32 effective_yaw_setpoint    # [rad]
float32 effective_yaw_rate_setpoint # [rad/s]
float32 command_age_s
float32 mob_age_s
float32 dt
