#pragma once

#include "Admittance4D.hpp"
#include <lib/matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <parameters/param.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <cstdio>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/palletrone_admittance_command.h>
#include <uORB/topics/palletrone_admittance_status.h>
#include <uORB/topics/palletrone_mob_status.h>

class AdmittanceManager
{
public:
	enum State : uint8_t { DISABLED_HOLD, WAIT_VALID, WAIT_CONTACT, BLEND_IN, ACTIVE, HANDOVER_DECEL, FAULT_HOLD };

	void update(uint64_t now, float loop_dt, const matrix::Dcmf &rotation_local_body, bool attitude_valid,
		    bool armed, bool position_mode, bool trajectory_flag, matrix::Vector4f &position_yaw_sp,
		    matrix::Vector4f &velocity_yawrate_sp)
	{
		loadParams();
		palletrone_admittance_command_s command{};
		if (_command_sub.update(&command)) {
			_last_command = command; _last_command_rx = now;
			if (!_reset_seen) { _last_reset_counter = command.reset_counter; _reset_seen = true; }
			else if (command.reset_counter != _last_reset_counter) {
				_last_reset_counter = command.reset_counter;
				if (_state != ACTIVE && _state != HANDOVER_DECEL) { resetAt(position_yaw_sp); ++_reset_count; }
			}
		}

		palletrone_mob_status_s mob{};
		const bool new_mob = _mob_sub.update(&mob);
		if (new_mob) { _mob = mob; _last_mob_rx = now; }
		const bool cmd_fresh = _last_command_rx && elapsed(now, _last_command_rx) <= _p.cmd_to;
		const bool mob_fresh = _last_mob_rx && elapsed(now, _last_mob_rx) <= _p.mob_to;
		const bool request = cmd_fresh && _last_command.enable;
		const bool dynamics_valid = validateParams();
		if (!dynamics_valid) { _fault_flags |= PARAM_BAD; } else { _fault_flags &= ~PARAM_BAD; }
		const bool force_valid = mob_fresh && _mob.frame == palletrone_mob_status_s::FRAME_BODY_FRD &&
			_mob.warmup_complete && _mob.force_valid && finiteWrench(false);
		const bool yaw_valid = !_p.axis_en[3] || (mob_fresh && _mob.torque_valid && finiteWrench(true));
		const bool basic_valid = _p.master && request && position_mode && attitude_valid &&
			(!_p.arm_only || armed) && dynamics_valid && force_valid && yaw_valid;

		const float fx = _mob.external_force_corrected[0];
		_normal_force = PX4_ISFINITE(fx) ? math::max(0.f, -fx) : 0.f;
		updateContact(now, force_valid);

		const bool trajectory_rising = trajectory_flag && !_last_trajectory_flag;
		_last_trajectory_flag = trajectory_flag;
		if (_hold_latched && trajectory_rising && _state == DISABLED_HOLD && !request) {
			releaseOwnership(); // preserve the legacy explicit new-command-session release
		}

		if (!request && (_state == WAIT_VALID || _state == WAIT_CONTACT)) {
			releaseOwnership();
			_state = DISABLED_HOLD;
		} else if ((_state == ACTIVE || _state == BLEND_IN) && (!basic_valid || !_contact)) {
			startHandover(now);
		}

		if (_state == DISABLED_HOLD) {
			if (request) {
				if (!basic_valid) {
					_state = WAIT_VALID;
				} else if (latchBase(position_yaw_sp)) {
					_state = _contact ? BLEND_IN : WAIT_CONTACT;
				}
			}
		} else if (_state == WAIT_VALID) {
			if (basic_valid && latchBase(position_yaw_sp)) {
				_state = _contact ? BLEND_IN : WAIT_CONTACT;
				_state_since = now;
			}
		} else if (_state == WAIT_CONTACT) {
			if (!basic_valid) {
				releaseOwnership();
				_state = WAIT_VALID;
			} else if (_contact) {
				_state = BLEND_IN;
				_state_since = now;
			}
		}

		if ((_state == BLEND_IN || _state == ACTIVE) && new_mob && _mob.timestamp_sample != _last_mob_sample) {
			const float sample_dt = _last_mob_sample ? (_mob.timestamp_sample - _last_mob_sample) * 1e-6f : loop_dt;
			_last_mob_sample = _mob.timestamp_sample;
			float blend = 1.f;
			if (_state == BLEND_IN) {
				blend = _p.in_t > 0.f ? math::constrain(elapsed(now, _state_since) / _p.in_t, 0.f, 1.f) : 1.f;
				if (blend >= 1.f) { _state = ACTIVE; }
			}
			float input[4]{}; makeInput(blend, input);
			if (!stepCore(sample_dt, input, false)) { severeFault(position_yaw_sp); }
			else { integrate(sample_dt, rotation_local_body); }
		}

		if (_state == HANDOVER_DECEL) {
			float zero[4]{};
			if (!stepCore(loop_dt, zero, true)) { severeFault(position_yaw_sp); }
			else {
				integrate(loop_dt, rotation_local_body);
				const bool stopped = fabsf(_core.state(0).dq) < _p.ho_v && fabsf(_core.state(1).dq) < _p.ho_v &&
					fabsf(_core.state(2).dq) < _p.ho_v && fabsf(_core.state(3).dq) < _p.ho_r;
				if (stopped || elapsed(now, _state_since) > _p.ho_t) { atomicRebase(); }
			}
		}

		if (_hold_latched) { applyEffective(position_yaw_sp, velocity_yawrate_sp); }
		publish(now, cmd_fresh, mob_fresh, force_valid, yaw_valid, loop_dt);
	}

private:
	struct Params {
		bool master{}, arm_only{true}, axis_en[4]{true,true,true,true};
		float cmd_to{.5f}, mob_to{.1f}, in_t{.25f}, n_des{2.f}, n_on{.8f}, n_off{.4f}, n_on_t{.2f}, n_off_t{.3f}, n_edb{.1f};
		palletrone::Admittance4D::AxisParams axis[4]{{4,40,0},{6,60,0},{6,60,0},{.2f,1,0}};
		float db[4]{.1f,.1f,.1f,.02f}, input_max[4]{5,5,5,1}, accel_max[4]{1,1,1,1};
		float velocity_max[4]{.05f,.15f,.15f,.3f}, position_max[4]{.1f,.3f,.3f,.35f};
		float ho_t{1.5f}, ho_v{.01f}, ho_r{.02f}, ho_ds{1.5f};
	} _p{};

	static constexpr uint32_t MASTER=1u<<0, REQUEST=1u<<1, CMD_FRESH=1u<<2, CMD_TIMEOUT=1u<<3,
		MOB_FRESH=1u<<4, FORCE_VALID=1u<<5, TORQUE_VALID=1u<<6, CONTACT=1u<<7, ACTIVE_F=1u<<8,
		HANDOVER=1u<<9, HOLD=1u<<10, CONTACT_LOST=1u<<11, PARAM_BAD=1u<<12, DT_INVALID=1u<<13,
		NONFINITE=1u<<14, INPUT_LIMIT=1u<<15, ACCEL_LIMIT=1u<<16, VELOCITY_LIMIT=1u<<17,
		POSITION_LIMIT=1u<<18, FAULT=1u<<19;

	static float elapsed(uint64_t now, uint64_t then) { return now >= then ? (now - then) * 1e-6f : INFINITY; }
	static float getf(const char *name, float fallback) { float v=fallback; param_t h=param_find(name); if (h!=PARAM_INVALID) { param_get(h,&v); } return v; }
	static bool geti(const char *name, bool fallback) { int32_t v=fallback; param_t h=param_find(name); if (h!=PARAM_INVALID) { param_get(h,&v); } return v!=0; }
	void loadParams()
	{
		_p.master=geti("PADM_EN",false); _p.arm_only=geti("PADM_ARM_ONLY",true); _p.cmd_to=getf("PADM_CMD_TO",.5f); _p.mob_to=getf("PADM_MOB_TO",.1f); _p.in_t=getf("PADM_IN_T",.25f);
		_p.n_des=getf("PADM_N_DES",2); _p.n_on=getf("PADM_N_ON",.8f); _p.n_off=getf("PADM_N_OFF",.4f); _p.n_on_t=getf("PADM_N_ON_T",.2f); _p.n_off_t=getf("PADM_N_OFF_T",.3f); _p.n_edb=getf("PADM_N_EDB",.1f);
		const char *prefix[4]={"PADM_X_","PADM_Y_","PADM_Z_","PADM_YAW_"}; const char *mass[4]={"PADM_X_M","PADM_Y_M","PADM_Z_M","PADM_YAW_J"};
		const char *damp[4]={"PADM_X_D","PADM_Y_D","PADM_Z_D","PADM_YAW_D"}; const char *stiff[4]={"PADM_X_K","PADM_Y_K","PADM_Z_K","PADM_YAW_K"};
		const char *db[4]={"PADM_N_EDB","PADM_Y_FDB","PADM_Z_FDB","PADM_YAW_TDB"};
		const char *fmax[4]={"PADM_X_FMAX","PADM_Y_FMAX","PADM_Z_FMAX","PADM_YAW_TMAX"}; const char *amax[4]={"PADM_X_AMAX","PADM_Y_AMAX","PADM_Z_AMAX","PADM_YAW_AMAX"};
		const char *vmax[4]={"PADM_X_VMAX","PADM_Y_VMAX","PADM_Z_VMAX","PADM_YAW_RMAX"}; const char *pmax[4]={"PADM_X_PMAX","PADM_Y_PMAX","PADM_Z_PMAX","PADM_YAW_QMAX"};
		for(int i=0;i<4;i++){ char en[17]{}; snprintf(en,sizeof(en),"%sEN",prefix[i]); _p.axis_en[i]=geti(en,true); _p.axis[i]={getf(mass[i],_p.axis[i].mass),getf(damp[i],_p.axis[i].damping),getf(stiff[i],_p.axis[i].stiffness)}; _p.db[i]=getf(db[i],_p.db[i]); _p.input_max[i]=getf(fmax[i],_p.input_max[i]); _p.accel_max[i]=getf(amax[i],_p.accel_max[i]); _p.velocity_max[i]=getf(vmax[i],_p.velocity_max[i]); _p.position_max[i]=getf(pmax[i],_p.position_max[i]); }
		_p.ho_t=getf("PADM_HO_T",1.5f); _p.ho_v=getf("PADM_HO_V",.01f); _p.ho_r=getf("PADM_HO_R",.02f); _p.ho_ds=getf("PADM_HO_DS",1.5f);
	}
	bool validateParams() const
	{
		if (!PX4_ISFINITE(_p.cmd_to)||_p.cmd_to<=0||!PX4_ISFINITE(_p.mob_to)||_p.mob_to<=0||_p.n_des<=0||_p.n_off<0||_p.n_on<=_p.n_off) return false;
		for(int i=0;i<4;i++) if(_p.axis_en[i] && (!PX4_ISFINITE(_p.axis[i].mass)||_p.axis[i].mass<=0||!PX4_ISFINITE(_p.axis[i].damping)||_p.axis[i].damping<=0||!PX4_ISFINITE(_p.axis[i].stiffness)||_p.axis[i].stiffness<0||_p.input_max[i]<=0||_p.accel_max[i]<=0||_p.velocity_max[i]<=0||_p.position_max[i]<=0)) return false;
		return true;
	}
	bool finiteWrench(bool torque) const { if(torque) return PX4_ISFINITE(_mob.external_torque_corrected[2]); return PX4_ISFINITE(_mob.external_force_corrected[0])&&PX4_ISFINITE(_mob.external_force_corrected[1])&&PX4_ISFINITE(_mob.external_force_corrected[2]); }
	void updateContact(uint64_t now,bool valid){ if(!valid){_contact=false;_contact_candidate_since=0;return;} bool candidate=_contact?_normal_force>_p.n_off:_normal_force>=_p.n_on; if(candidate){_contact_off_since=0;if(!_contact){if(!_contact_candidate_since)_contact_candidate_since=now;if(elapsed(now,_contact_candidate_since)>=_p.n_on_t)_contact=true;}}else{_contact_candidate_since=0;if(_contact){if(!_contact_off_since)_contact_off_since=now;if(elapsed(now,_contact_off_since)>=_p.n_off_t){_contact=false;_contact_lost=true;}}} }
	bool latchBase(const matrix::Vector4f &sp){ if(!sp.isAllFinite()){severeFault(sp);return false;} _base=sp;_offset.zero();_core.reset();_effective_velocity.zero();_hold_latched=true;_state_since=hrt_absolute_time();return true; }
	void resetAt(const matrix::Vector4f &sp){ if(sp.isAllFinite())_base=sp;_offset.zero();_core.reset();_effective_velocity.zero();_hold_latched=false;_state=DISABLED_HOLD; }
	void startHandover(uint64_t now){_state=HANDOVER_DECEL;_state_since=now;}
	void severeFault(const matrix::Vector4f &sp){if(sp.isAllFinite())_base=sp;_offset.zero();_core.reset();_effective_velocity.zero();_hold_latched=true;_state=FAULT_HOLD;_fault_flags|=FAULT;}
	void makeInput(float blend,float out[4]){float raw[4]={_p.n_des-_normal_force,_mob.external_force_corrected[1],_mob.external_force_corrected[2],_mob.external_torque_corrected[2]};for(int i=0;i<4;i++){out[i]=_p.axis_en[i]?palletrone::Admittance4D::deadzone(raw[i],i==0?_p.n_edb:_p.db[i])*blend:0.f;float c=math::constrain(out[i],-_p.input_max[i],_p.input_max[i]);if(fabsf(c-out[i])>FLT_EPSILON)_sat_flags|=INPUT_LIMIT;out[i]=c;_last_input[i]=out[i];}}
	bool stepCore(float dt,const float input[4],bool handover){palletrone::Admittance4D::AxisParams p[4]={_p.axis[0],_p.axis[1],_p.axis[2],_p.axis[3]};if(handover)for(int i=0;i<4;i++){p[i].stiffness=0;p[i].damping*=_p.ho_ds;}if(!_core.update(dt,input,p)){_fault_flags|=DT_INVALID;return false;}for(int i=0;i<4;i++){auto s=_core.state(i);float a=math::constrain(s.ddq,-_p.accel_max[i],_p.accel_max[i]);if(fabsf(a-s.ddq)>FLT_EPSILON){_sat_flags|=ACCEL_LIMIT;s.ddq=a;s.dq=math::constrain(s.dq,-_p.velocity_max[i],_p.velocity_max[i]);}_core.setState(i,s);}return true;}
	void integrate(float dt,const matrix::Dcmf &R){matrix::Vector3f vb{_core.state(0).dq,_core.state(1).dq,_core.state(2).dq};for(int i=0;i<3;i++){float c=math::constrain(vb(i),-_p.velocity_max[i],_p.velocity_max[i]);if(fabsf(c-vb(i))>FLT_EPSILON)_sat_flags|=VELOCITY_LIMIT;vb(i)=c;auto s=_core.state(i);s.dq=c;_core.setState(i,s);}matrix::Vector3f vl=R*vb;for(int i=0;i<3;i++){float next=_offset(i)+vl(i)*dt;float c=math::constrain(next,-_p.position_max[i],_p.position_max[i]);if(fabsf(c-next)>FLT_EPSILON){_sat_flags|=POSITION_LIMIT;if((c>=_p.position_max[i]&&vl(i)>0)||(c<=-_p.position_max[i]&&vl(i)<0)){auto s=_core.state(i);s.dq=0;s.ddq=0;_core.setState(i,s);vl(i)=0;}}_offset(i)=c;_effective_velocity(i)=vl(i);}auto sy=_core.state(3);sy.dq=math::constrain(sy.dq,-_p.velocity_max[3],_p.velocity_max[3]);float yn=_offset(3)+sy.dq*dt;float yc=math::constrain(yn,-_p.position_max[3],_p.position_max[3]);if(fabsf(yc-yn)>FLT_EPSILON){_sat_flags|=POSITION_LIMIT;if((yc>=_p.position_max[3]&&sy.dq>0)||(yc<=-_p.position_max[3]&&sy.dq<0)){sy.dq=0;sy.ddq=0;}}_offset(3)=yc;_effective_velocity(3)=sy.dq;_core.setState(3,sy);}
	void releaseOwnership(){_offset.zero();_core.reset();_effective_velocity.zero();_hold_latched=false;}
	void atomicRebase(){_base+=_offset;releaseOwnership();_state=DISABLED_HOLD;}
	void applyEffective(matrix::Vector4f &sp,matrix::Vector4f &vel){sp=_base+_offset;vel=_effective_velocity;if(_state==WAIT_CONTACT||_state==DISABLED_HOLD||_state==FAULT_HOLD)vel.zero();}
	void publish(uint64_t now,bool cf,bool mf,bool fv,bool yv,float dt){palletrone_admittance_status_s s{};s.timestamp=now;s.timestamp_sample=_mob.timestamp_sample;s.state=_state;s.master_enabled=_p.master;s.enable_requested=_last_command.enable;s.command_fresh=cf;s.active=_state==ACTIVE;s.contact=_contact;s.translation_valid=fv;s.yaw_valid=yv;s.handover_active=_state==HANDOVER_DECEL;s.hold_latched=_hold_latched;s.reset_count=_reset_count;s.command_sequence=_last_command.sequence;s.command_reset_counter=_last_command.reset_counter;s.normal_force=_normal_force;s.normal_force_desired=_p.n_des;s.normal_error=_p.n_des-_normal_force;s.command_age_s=_last_command_rx?elapsed(now,_last_command_rx):-1;s.mob_age_s=_last_mob_rx?elapsed(now,_last_mob_rx):-1;s.dt=dt;s.status_flags=(_p.master?MASTER:0)|(_last_command.enable?REQUEST:0)|(cf?CMD_FRESH:CMD_TIMEOUT)|(mf?MOB_FRESH:0)|(fv?FORCE_VALID:0)|(yv?TORQUE_VALID:0)|(_contact?CONTACT:0)|(_state==ACTIVE?ACTIVE_F:0)|(_state==HANDOVER_DECEL?HANDOVER:0)|(_hold_latched?HOLD:0)|(_contact_lost?CONTACT_LOST:0)|_sat_flags|_fault_flags;for(int i=0;i<4;i++){s.admittance_input[i]=_last_input[i];s.displacement[i]=_core.state(i).q;s.velocity[i]=_core.state(i).dq;s.acceleration[i]=_core.state(i).ddq;}s.external_wrench[0]=_mob.external_force_corrected[0];s.external_wrench[1]=_mob.external_force_corrected[1];s.external_wrench[2]=_mob.external_force_corrected[2];s.external_wrench[3]=_mob.external_torque_corrected[2];for(int i=0;i<3;i++){s.local_position_offset[i]=_offset(i);s.hold_position[i]=_base(i);s.effective_position_setpoint[i]=_base(i)+_offset(i);s.effective_velocity_setpoint[i]=_effective_velocity(i);}s.local_yaw_offset=_offset(3);s.hold_yaw=_base(3);s.effective_yaw_setpoint=matrix::wrap_pi(_base(3)+_offset(3));s.effective_yaw_rate_setpoint=_effective_velocity(3);_status_pub.publish(s);_sat_flags=0;}

	uORB::Subscription _command_sub{ORB_ID(palletrone_admittance_command)}; uORB::Subscription _mob_sub{ORB_ID(palletrone_mob_status)}; uORB::Publication<palletrone_admittance_status_s> _status_pub{ORB_ID(palletrone_admittance_status)};
	palletrone_admittance_command_s _last_command{}; palletrone_mob_status_s _mob{}; palletrone::Admittance4D _core{}; State _state{DISABLED_HOLD};
	matrix::Vector4f _base{},_offset{},_effective_velocity{}; float _last_input[4]{},_normal_force{}; uint64_t _last_command_rx{},_last_mob_rx{},_last_mob_sample{},_state_since{},_contact_candidate_since{},_contact_off_since{}; uint32_t _last_reset_counter{},_reset_count{},_sat_flags{},_fault_flags{}; bool _reset_seen{},_contact{},_contact_lost{},_hold_latched{},_last_trajectory_flag{};
};
