#pragma once

/**
 * Convert an actual non-reversible ESC throttle fraction into
 * PX4 actuator_motors.control [0, 1].
 *
 * PX4 maps actuator_motors.control=0 to DSHOT_MIN and control=1 to full DShot.
 */
float dshot_throttle_to_actuator_control(float esc_throttle, float dshot_min_norm);

/**
 * Convert requested motor thrust [N] to PX4 actuator_motors.control [0, 1]
 * using the F60 PRO V-LV 2020KV lookup table and the 80% limit.
 */
float force_to_dshot_control(float force_n, float dshot_min_norm);
