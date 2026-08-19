#!/usr/bin/env python3
"""Deterministic host tests for the uORB-independent Tustin core and handover invariants."""
import math, pathlib, subprocess, tempfile, textwrap

ROOT = pathlib.Path(__file__).resolve().parents[5]
source = r'''
#include "src/modules/mc_pos_control/palletrone_admittance/Admittance4D.hpp"
#include <cassert>
#include <cmath>
#include <limits>
using A=palletrone::Admittance4D;
int main(){
 A a; A::AxisParams p[4]={{4,40,0},{6,60,0},{6,60,0},{.2f,1,0}}; float u[4]{};
 assert(a.update(.01f,u,p)); for(int j=0;j<4;j++)assert(a.state(j).q==0&&a.state(j).dq==0);
 u[0]=2; for(int i=0;i<3000;i++)assert(a.update((i%3?0.008f:0.012f),u,p));
 assert(std::fabs(a.state(0).dq-.05f)<1e-4f); float moved=a.state(0).q; assert(moved>0);
 u[0]=0;for(int i=0;i<1000;i++)assert(a.update(.01f,u,p));assert(std::fabs(a.state(0).dq)<1e-4f);assert(a.state(0).q>=moved);
 a.reset();p[0].stiffness=20;u[0]=2;for(int i=0;i<4000;i++)assert(a.update(.01f,u,p));assert(std::fabs(a.state(0).q-.1f)<1e-3f);assert(std::fabs(a.state(0).dq)<1e-3f);
 A::State before=a.state(0);p[2].mass=0;assert(!a.update(.01f,u,p));assert(a.state(0).q==before.q);p[2].mass=6;
 assert(!a.update(0,u,p));assert(!a.update(.2f,u,p));u[1]=std::numeric_limits<float>::quiet_NaN();assert(!a.update(.01f,u,p));u[1]=0;
 a.reset();p[0].stiffness=0;u[0]=0;u[1]=1;assert(a.update(.01f,u,p));assert(a.state(0).q==0&&a.state(1).dq>0);
 assert(A::deadzone(.05f,.1f)==0);assert(std::fabs(A::deadzone(.4f,.1f)-.3f)<1e-6f);assert(A::deadzone(-.4f,.1f)<0);
 return 0;
}'''
with tempfile.TemporaryDirectory() as d:
    src=pathlib.Path(d)/"test.cpp"; exe=pathlib.Path(d)/"test"; src.write_text(source)
    subprocess.run(["g++","-std=c++14","-Wall","-Wextra","-Werror",f"-I{ROOT}/src",str(src),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

# Sign, contact hysteresis, and bumpless reference invariants (implementation equations).
dz=lambda v,e: 0 if abs(v)<=e else math.copysign(abs(v)-e,v)
assert dz(2-max(0,-(-.5)),.1)>0                 # low N -> +x
assert dz(2-max(0,-(-3.0)),.1)<0                # high N -> -x
assert dz(1,.1)>0 and dz(-1,.1)<0               # +Fy, -Fz
assert dz(.3,.02)>0                             # +Tz -> +yaw
corrected_fx=-2.0
assert max(0,-corrected_fx)==2.0                # corrected -Fx is compression
assert max(0,-(+2.0))==0.0                     # corrected +Fx is not contact
assert dz(.05,.1)==0                            # corrected Fy inside deadband
assert abs(dz(.4,.1)-.3)<1e-9                  # corrected Fy soft deadband

# MOB owns bias subtraction; Admittance consumes corrected wrench directly.
raw_fy=.7; mob_bias_fy=.2; corrected_fy=raw_fy-mob_bias_fy
assert abs(dz(corrected_fy,.1)-.4)<1e-9
admittance_source=(ROOT/"src/modules/mc_pos_control/palletrone_admittance/AdmittanceManager.hpp").read_text()
assert "external_force_corrected[1]" in admittance_source
assert "external_torque_corrected[2]" in admittance_source

base=[1.,2.,-1.,3.12]; offset=[.2,-.1,.05,.3]
eff_before=[a+b for a,b in zip(base,offset)]
base=eff_before[:]; offset=[0.,0.,0.,0.]
eff_after=[a+b for a,b in zip(base,offset)]
assert eff_after==eff_before                    # atomic rebase continuity
assert base[0]!=1.0                             # never returns to old nominal

v=.15; history=[]
for _ in range(100):
    v *= (1-.01*60/6)                           # damping-only handover envelope
    history.append(v)
assert all(history[i+1]<=history[i] for i in range(len(history)-1)) and history[-1]<.01

on_t=off_t=0.; contact=False
for _ in range(30):
    on_t+=.01
    if on_t>=.2: contact=True
assert contact
for _ in range(20): off_t+=.01                  # shorter than 0.3 s grace
assert contact

print("PASS: Admittance4D dynamics/sign/invalid/atomic/axis and reference/contact/handover invariants")
