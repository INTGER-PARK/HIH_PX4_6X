#pragma once

#include "MomentumObserver.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/palletrone_mob_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/servo_angle.h>
#include <uORB/topics/thrust_command.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>

class PalletroneMob : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	/** Construct the wrapper on the navigation/controller work queue. */
	PalletroneMob();

	/** Release the module performance counter; no control state is owned here. */
	~PalletroneMob() override;

	/** Allocate the work-queue module instance for the NSH `start` command. */
	static int task_spawn(int argc, char *argv[]);

	/** Reject unsupported module-specific NSH commands and print usage. */
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

	/** Print NSH command usage and the publish-only module description. */
	static int print_usage(const char *reason = nullptr);

	/** Print enable/initialization/reset state and execution-time statistics. */
	int print_status() override;

	/** Schedule the non-blocking observer wrapper at a nominal 100 Hz. */
	void start();

private:
	/**
	 * Main 100 Hz cycle: receive uORB inputs, apply freshness/frame/finite
	 * validity gates, reset or advance both observers, and publish diagnostics.
	 * This method never publishes a controller, allocator, or actuator command.
	 */
	void Run() override;

	/** Reset force and torque observers to measured momentum and start warm-up. */
	void resetObservers(const matrix::Vector3f &linear_momentum,
			    const matrix::Vector3f &angular_momentum, hrt_abstime now);

	/** Validate mass, gains, and positive definiteness of the inertia matrix. */
	bool parametersValid() const;

	/**
	 * Reconstruct known body-FRD actuator force [N] and torque [N m] from the
	 * final physical thrust_command and actual servo_angle samples. The torque
	 * coefficients intentionally match the active custom allocator matrix.
	 */
	void computeActuatorWrench(matrix::Vector3f &force, matrix::Vector3f &torque) const;

	/** Populate and publish the named monitoring/logging status uORB message. */
	void publishStatus(hrt_abstime now, uint32_t flags, float dt,
			   const matrix::Vector3f &actuator_force, const matrix::Vector3f &actuator_torque,
			   bool force_valid, bool torque_valid);

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _thrust_sub{ORB_ID(thrust_command)};
	uORB::Subscription _servo_sub{ORB_ID(servo_angle)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<palletrone_mob_status_s> _status_pub{ORB_ID(palletrone_mob_status)};

	palletrone_mob::MomentumObserver _force_observer;
	palletrone_mob::MomentumObserver _torque_observer;
	vehicle_odometry_s _odometry{};
	float _motor_thrust_n[4]{};
	float _servo_angle_rad[4]{};
	hrt_abstime _odometry_received{0};
	hrt_abstime _actuator_received{0};
	hrt_abstime _servo_received{0};
	hrt_abstime _warmup_started{0};
	uint64_t _last_timestamp_sample{0};
	uint8_t _last_reset_counter{0};
	uint32_t _reset_count{0};
	bool _armed{false};
	bool _landed{true};
	bool _active_last_cycle{false};
	bool _reset_flag_pending{false};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::PMOB_EN>) _param_enable,
		(ParamFloat<px4::params::PMOB_MASS>) _param_mass,
		(ParamFloat<px4::params::PMOB_IXX>) _param_ixx,
		(ParamFloat<px4::params::PMOB_IYY>) _param_iyy,
		(ParamFloat<px4::params::PMOB_IZZ>) _param_izz,
		(ParamFloat<px4::params::PMOB_IXY>) _param_ixy,
		(ParamFloat<px4::params::PMOB_IXZ>) _param_ixz,
		(ParamFloat<px4::params::PMOB_IYZ>) _param_iyz,
		(ParamFloat<px4::params::PMOB_F_WN>) _param_force_wn,
		(ParamFloat<px4::params::PMOB_F_ZETA>) _param_force_zeta,
		(ParamFloat<px4::params::PMOB_T_WN>) _param_torque_wn,
		(ParamFloat<px4::params::PMOB_T_ZETA>) _param_torque_zeta,
		(ParamFloat<px4::params::PMOB_ODOM_TO>) _param_odom_timeout,
		(ParamFloat<px4::params::PMOB_ACT_TO>) _param_actuator_timeout,
		(ParamFloat<px4::params::PMOB_SERVO_TO>) _param_servo_timeout,
		(ParamFloat<px4::params::PMOB_WARMUP>) _param_warmup,
		(ParamBool<px4::params::PMOB_ARM_ONLY>) _param_arm_only
	)
};
