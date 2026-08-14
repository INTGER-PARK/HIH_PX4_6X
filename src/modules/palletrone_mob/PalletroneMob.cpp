#include "PalletroneMob.hpp"

#include <lib/geo/geo.h>
#include <lib/palletrone/PalletroneConfig.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

using namespace time_literals;
using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector3f;

ModuleBase::Descriptor PalletroneMob::desc{task_spawn, custom_command, print_usage};

// Bind the module to PX4's navigation/controller work queue. The constructor
// only establishes execution context; it does not enable or start estimation.
PalletroneMob::PalletroneMob() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

// Release diagnostic resources allocated by this module.
PalletroneMob::~PalletroneMob()
{
	perf_free(_cycle_perf);
}

// Create the single work-queue instance requested by `palletrone_mob start`.
int PalletroneMob::task_spawn(int argc, char *argv[])
{
	PalletroneMob *instance = new PalletroneMob();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;
	instance->start();
	return PX4_OK;
}

// Run the wrapper periodically. Actual observer dt still comes from the
// monotonically increasing vehicle_odometry timestamp_sample, not this period.
void PalletroneMob::start()
{
	ScheduleOnInterval(10_ms);
}

// Reject parameter sets that would make momentum or observer dynamics invalid.
// Sylvester's criterion checks that the symmetric 3x3 inertia is positive definite.
bool PalletroneMob::parametersValid() const
{
	const float ixx = _param_ixx.get();
	const float iyy = _param_iyy.get();
	const float izz = _param_izz.get();
	const float ixy = _param_ixy.get();
	const float ixz = _param_ixz.get();
	const float iyz = _param_iyz.get();
	const float minor2 = ixx * iyy - ixy * ixy;
	const float determinant = ixx * iyy * izz + 2.f * ixy * ixz * iyz
				  - ixx * iyz * iyz - iyy * ixz * ixz - izz * ixy * ixy;

	return PX4_ISFINITE(_param_mass.get()) && _param_mass.get() > 0.f
	       && PX4_ISFINITE(ixx) && PX4_ISFINITE(iyy) && PX4_ISFINITE(izz)
	       && PX4_ISFINITE(ixy) && PX4_ISFINITE(ixz) && PX4_ISFINITE(iyz)
	       && ixx > 0.f && minor2 > 0.f && determinant > 0.f
	       && PX4_ISFINITE(_param_force_wn.get()) && _param_force_wn.get() > 0.f
	       && PX4_ISFINITE(_param_force_zeta.get()) && _param_force_zeta.get() > 0.f
	       && PX4_ISFINITE(_param_torque_wn.get()) && _param_torque_wn.get() > 0.f
	       && PX4_ISFINITE(_param_torque_zeta.get()) && _param_torque_zeta.get() > 0.f;
}

// Convert final motor thrust [N] and actual DYNAMIXEL angle [rad] to the known
// body-FRD actuator wrench. Torque terms reproduce the active allocator rows.
void PalletroneMob::computeActuatorWrench(Vector3f &force, Vector3f &torque) const
{
	force.setZero();
	torque.setZero();

	for (int i = 0; i < 4; ++i) {
		const float sine = sinf(_servo_angle_rad[i]);
		const float cosine = cosf(_servo_angle_rad[i]);
		const Vector3f direction{
			palletrone::kTiltDirectionXY[i][0] * palletrone::kInvSqrt2 * sine,
			palletrone::kTiltDirectionXY[i][1] * palletrone::kInvSqrt2 * sine,
			-cosine
		};
		const Vector3f rotor_force = direction * _motor_thrust_n[i];
		force += rotor_force;

		// Match the active custom allocator effectiveness matrix exactly. Its
		// yaw row intentionally contains reaction torque only at the modeled
		// zero CoM offset (it omits the tilted-force r x f yaw component).
		const float z = palletrone::kRotorVerticalOffsetM;
		const float kappa = palletrone::kRotorReactionTorqueRatioM;
		const float arm = palletrone::kArmLengthM * palletrone::kInvSqrt2;
		const float sine_scaled = sine * palletrone::kInvSqrt2;
		const float roll_sign = i < 2 ? 1.f : -1.f;
		const float pitch_sign = (i == 0 || i == 1) ? 1.f : -1.f;
		const float tilt_coefficient = (i == 0 || i == 3) ? (-z + kappa) : (z - kappa);
		const float pitch_tilt_coefficient[4] = {z + kappa, z - kappa, -z - kappa, -z + kappa};
		torque(0) += (roll_sign * arm * cosine + tilt_coefficient * sine_scaled) * _motor_thrust_n[i];
		torque(1) += (pitch_sign * arm * cosine + pitch_tilt_coefficient[i] * sine_scaled) * _motor_thrust_n[i];
		torque(2) += (-palletrone::kRotorReactionSign[i] * kappa * cosine) * _motor_thrust_n[i];
	}
}

// Initialize p_hat from current measurements, clear w_hat, and begin the
// post-reset warm-up interval during which estimates are published invalid.
void PalletroneMob::resetObservers(const Vector3f &linear_momentum,
				   const Vector3f &angular_momentum, hrt_abstime now)
{
	_force_observer.reset(linear_momentum);
	_torque_observer.reset(angular_momentum);
	_warmup_started = now;
	_reset_count++;
	_reset_flag_pending = true;
}

// Serialize observer state, reconstructed actuator wrench, validity flags, and
// local uORB reception ages into the DDS/logging status topic.
void PalletroneMob::publishStatus(hrt_abstime now, uint32_t flags, float dt,
				 const Vector3f &actuator_force, const Vector3f &actuator_torque,
				 bool force_valid, bool torque_valid)
{
	palletrone_mob_status_s status{};
	status.timestamp = now;
	status.timestamp_sample = _odometry.timestamp_sample;
	status.frame = palletrone_mob_status_s::FRAME_BODY_FRD;
	status.force_valid = force_valid;
	status.torque_valid = torque_valid;
	status.initialized = _force_observer.initialized() || _torque_observer.initialized();
	status.warmup_complete = _warmup_started != 0
		&& (now - _warmup_started) >= static_cast<hrt_abstime>(_param_warmup.get() * 1e6f);
	status.status_flags = flags;
	status.reset_count = _reset_count;
	_force_observer.estimate().copyTo(status.external_force);
	_torque_observer.estimate().copyTo(status.external_torque);
	actuator_force.copyTo(status.known_actuator_force);
	actuator_torque.copyTo(status.known_actuator_torque);
	_force_observer.error().copyTo(status.linear_momentum_error);
	_torque_observer.error().copyTo(status.angular_momentum_error);
	status.dt = dt;
	status.odometry_age_s = _odometry_received ? (now - _odometry_received) * 1e-6f : -1.f;
	status.actuator_age_s = _actuator_received ? (now - _actuator_received) * 1e-6f : -1.f;
	status.servo_age_s = _servo_received ? (now - _servo_received) * 1e-6f : -1.f;
	_status_pub.publish(status);
}

// Receive asynchronous inputs and execute one guarded observer cycle. Input
// freshness uses local uORB reception time because CM4 servo timestamps can be
// in a different clock domain. Only PalletroneMobStatus is published here.
void PalletroneMob::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);
	const hrt_abstime now = hrt_absolute_time();
	uint32_t flags = 0;

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();
		_active_last_cycle = false;
	}

	vehicle_status_s vehicle_status{};

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}

	vehicle_land_detected_s land_detected{};

	if (_land_detected_sub.update(&land_detected)) {
		_landed = land_detected.landed;
	}

	thrust_command_s thrust{};

	if (_thrust_sub.update(&thrust)) {
		bool finite = true;

		for (int i = 0; i < 4; ++i) {
			finite = finite && PX4_ISFINITE(thrust.thrust_command[i]) && thrust.thrust_command[i] >= 0.f;
			_motor_thrust_n[i] = thrust.thrust_command[i];
		}

		if (finite) {
			_actuator_received = now;
		}
	}

	servo_angle_s servo{};

	if (_servo_sub.update(&servo)) {
		bool finite = true;

		for (int i = 0; i < 4; ++i) {
			finite = finite && PX4_ISFINITE(servo.servo_angle[i]);
			_servo_angle_rad[i] = servo.servo_angle[i];
		}

		if (finite) {
			_servo_received = now;
		}
	}

	const bool new_odometry = _odometry_sub.update(&_odometry);

	if (new_odometry) {
		_odometry_received = now;
	}

	Vector3f actuator_force{};
	Vector3f actuator_torque{};
	computeActuatorWrench(actuator_force, actuator_torque);

	const bool enabled = _param_enable.get();
	const bool parameter_valid = parametersValid();
	const bool actuator_fresh = _actuator_received != 0
		&& (now - _actuator_received) <= static_cast<hrt_abstime>(_param_actuator_timeout.get() * 1e6f);
	const bool servo_fresh = _servo_received != 0
		&& (now - _servo_received) <= static_cast<hrt_abstime>(_param_servo_timeout.get() * 1e6f);
	const bool odometry_fresh = _odometry_received != 0
		&& (now - _odometry_received) <= static_cast<hrt_abstime>(_param_odom_timeout.get() * 1e6f);

	if (enabled) { flags |= palletrone_mob_status_s::FLAG_ENABLED; }
	if (_armed) { flags |= palletrone_mob_status_s::FLAG_ARMED; }
	if (_landed) { flags |= palletrone_mob_status_s::FLAG_LANDED; }
	if (actuator_fresh) { flags |= palletrone_mob_status_s::FLAG_ACTUATOR_FRESH; }
	if (servo_fresh) { flags |= palletrone_mob_status_s::FLAG_SERVO_FRESH; }
	if (!parameter_valid) { flags |= palletrone_mob_status_s::FLAG_PARAMETER_INVALID; }

	if (!enabled && (_force_observer.initialized() || _torque_observer.initialized())) {
		_force_observer.invalidate();
		_torque_observer.invalidate();
		_active_last_cycle = false;
	}

	const Quatf quaternion{_odometry.q};
	const bool attitude_valid = (_odometry.pose_frame == vehicle_odometry_s::POSE_FRAME_NED
		|| _odometry.pose_frame == vehicle_odometry_s::POSE_FRAME_FRD)
		&& quaternion.isAllFinite() && quaternion.norm() > 0.5f;
	const bool angular_velocity_valid = Vector3f{_odometry.angular_velocity}.isAllFinite();
	Vector3f velocity_body{};
	bool velocity_valid = false;

	const bool velocity_variance_finite = Vector3f{_odometry.velocity_variance}.isAllFinite();

	if (attitude_valid && velocity_variance_finite && Vector3f{_odometry.velocity}.isAllFinite()) {
		if (_odometry.velocity_frame == vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD) {
			velocity_body = Vector3f{_odometry.velocity};
			velocity_valid = true;

		} else if (_odometry.velocity_frame == vehicle_odometry_s::VELOCITY_FRAME_NED
			   || _odometry.velocity_frame == vehicle_odometry_s::VELOCITY_FRAME_FRD) {
			velocity_body = Dcmf{quaternion}.transpose() * Vector3f{_odometry.velocity};
			velocity_valid = velocity_body.isAllFinite();
		}
	}

	if (odometry_fresh) { flags |= palletrone_mob_status_s::FLAG_ODOM_VALID; }
	if (attitude_valid) { flags |= palletrone_mob_status_s::FLAG_ATTITUDE_VALID; }
	if (velocity_valid) { flags |= palletrone_mob_status_s::FLAG_VELOCITY_VALID; }

	if (!quaternion.isAllFinite() || !Vector3f{_odometry.velocity}.isAllFinite()
	    || !Vector3f{_odometry.angular_velocity}.isAllFinite() || !velocity_variance_finite
	    || !actuator_force.isAllFinite() || !actuator_torque.isAllFinite()) {
		flags |= palletrone_mob_status_s::FLAG_NONFINITE_INPUT;
	}

	const bool arm_gate = !_param_arm_only.get() || _armed;
	const bool common_valid = enabled && parameter_valid && odometry_fresh && attitude_valid
		&& actuator_fresh && servo_fresh && arm_gate;
	const bool force_input_valid = common_valid && velocity_valid;
	const bool torque_input_valid = common_valid && angular_velocity_valid;
	float dt = 0.f;
	bool dt_valid = false;

	if (new_odometry && _last_timestamp_sample != 0 && _odometry.timestamp_sample > _last_timestamp_sample) {
		dt = (_odometry.timestamp_sample - _last_timestamp_sample) * 1e-6f;
		dt_valid = PX4_ISFINITE(dt) && dt >= 0.0005f && dt <= 0.05f;
	}

	const bool estimator_reset = new_odometry && _last_timestamp_sample != 0
		&& _odometry.reset_counter != _last_reset_counter;
	const bool timestamp_reversed = new_odometry && _last_timestamp_sample != 0
		&& _odometry.timestamp_sample <= _last_timestamp_sample;

	if (new_odometry) {
		_last_reset_counter = _odometry.reset_counter;
	}

	const float mass = _param_mass.get();
	const Vector3f omega{_odometry.angular_velocity};
	const Vector3f linear_momentum = velocity_body * mass;
	const Vector3f angular_momentum{
		_param_ixx.get() * omega(0) + _param_ixy.get() * omega(1) + _param_ixz.get() * omega(2),
		_param_ixy.get() * omega(0) + _param_iyy.get() * omega(1) + _param_iyz.get() * omega(2),
		_param_ixz.get() * omega(0) + _param_iyz.get() * omega(1) + _param_izz.get() * omega(2)
	};

	if ((!force_input_valid && !torque_input_valid) || estimator_reset || timestamp_reversed || !dt_valid) {
		if (new_odometry && (force_input_valid || torque_input_valid)
		    && linear_momentum.isAllFinite() && angular_momentum.isAllFinite()) {
			resetObservers(linear_momentum, angular_momentum, now);
		}

		if (estimator_reset || timestamp_reversed) { flags |= palletrone_mob_status_s::FLAG_ESTIMATOR_RESET; }
		if (new_odometry && !dt_valid) { flags |= palletrone_mob_status_s::FLAG_DT_INVALID; }
		_active_last_cycle = false;
	}

	bool force_updated = false;
	bool torque_updated = false;

	if (new_odometry && dt_valid && (force_input_valid || torque_input_valid)) {
		if (!_active_last_cycle) {
			resetObservers(linear_momentum, angular_momentum, now);
		}

		const Dcmf rotation_local_from_body{quaternion};
		const Vector3f gravity_body = rotation_local_from_body.transpose() * Vector3f{0.f, 0.f, CONSTANTS_ONE_G};
		const Vector3f b_linear = actuator_force + gravity_body * mass - omega.cross(velocity_body) * mass;
		const Vector3f b_angular = actuator_torque - omega.cross(angular_momentum);

		if (force_input_valid) {
			force_updated = _force_observer.update(linear_momentum, b_linear, dt,
							_param_force_wn.get(), _param_force_zeta.get());
		}

		if (torque_input_valid) {
			torque_updated = _torque_observer.update(angular_momentum, b_angular, dt,
							  _param_torque_wn.get(), _param_torque_zeta.get());
		}

		_active_last_cycle = force_updated || torque_updated;
	}

	if (new_odometry) {
		_last_timestamp_sample = _odometry.timestamp_sample;
	}

	const bool warmup_complete = _warmup_started != 0
		&& (now - _warmup_started) >= static_cast<hrt_abstime>(_param_warmup.get() * 1e6f);

	if (!warmup_complete) { flags |= palletrone_mob_status_s::FLAG_WARMUP; }
	if (_reset_flag_pending) {
		flags |= palletrone_mob_status_s::FLAG_ESTIMATOR_RESET;
		_reset_flag_pending = false;
	}

	const bool force_valid = force_updated && warmup_complete;
	const bool torque_valid = torque_updated && warmup_complete;
	publishStatus(now, flags, dt, actuator_force, actuator_torque, force_valid, torque_valid);
	perf_end(_cycle_perf);
}

// Report runtime state without modifying observer or flight-controller state.
int PalletroneMob::print_status()
{
	PX4_INFO("enabled: %s, force initialized: %s, torque initialized: %s, resets: %u",
		 _param_enable.get() ? "yes" : "no", _force_observer.initialized() ? "yes" : "no",
		 _torque_observer.initialized() ? "yes" : "no", (unsigned)_reset_count);
	perf_print_counter(_cycle_perf);
	return 0;
}

// Describe the NSH interface and emphasize the estimator-only purpose.
int PalletroneMob::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }
	PRINT_MODULE_DESCRIPTION("Publish-only six-axis second-order momentum observer for Palletrone.");
	PRINT_MODULE_USAGE_NAME("palletrone_mob", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

// C entry point exported to PX4's builtin command table.
extern "C" __EXPORT int palletrone_mob_main(int argc, char *argv[])
{
	return ModuleBase::main(PalletroneMob::desc, argc, argv);
}
