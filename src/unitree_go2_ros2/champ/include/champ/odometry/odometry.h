/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <quadruped_base/quadruped_base.h>
#include <macros/macros.h>

#include <geometry/geometry.h>

namespace champ
{
    class Odometry
    {
        QuadrupedBase *base_;
        geometry::Transformation prev_foot_position_[4];
        bool prev_foot_contacts_[4];
        // Retained for source compatibility only: the per-foot bearing history that
        // getVelocities() used to derive yaw rate from. It now solves for the body
        // twist directly, so nothing reads this.
        float prev_theta_[4];
        unsigned long int prev_time_;
        champ::Velocities prev_vel_;
        float beta_;

        public:
            typedef unsigned long int Time;
            static inline Time now() { return time_us(); }

            Odometry(QuadrupedBase &quadruped_base, Time time = now()):
                base_(&quadruped_base),
                prev_foot_contacts_{1,1,1,1},
                prev_theta_{0,0,0,0},
                prev_time_(time),
                beta_(0.1)
            {
                for(unsigned int i = 0; i < 4; i++)
                {
                    prev_foot_position_[i] = base_->legs[i]->foot_from_base();
                }
            }        
            
            bool allFeetInContact()
            {
                if(base_->legs[0]->in_contact() &&
                   base_->legs[1]->in_contact() &&
                   base_->legs[2]->in_contact() &&
                   base_->legs[3]->in_contact())
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            bool noFootInContact()
            {
                if(!base_->legs[0]->in_contact() &&
                   !base_->legs[1]->in_contact() &&
                   !base_->legs[2]->in_contact() &&
                   !base_->legs[3]->in_contact())
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            void getVelocities(champ::Velocities &vel, Time now = champ::Odometry::now())
            {      
                //if all legs are on the ground, nothing to calculate
                //or if no legs are on the ground, probably the robot is upside-down
                if(allFeetInContact() || noFootInContact())
                {
                    vel.linear.x = 0.0;
                    vel.linear.y = 0.0;
                    vel.angular.z = 0.0;

                    prev_vel_.linear.x = 0.0;
                    prev_vel_.linear.y = 0.0;
                    prev_vel_.angular.z = 0.0;

                    return;
                }

                double dt = (now - prev_time_) / 1000000.0;
                // zero division check
                if (dt == 0)
                    dt = 0.02;

                // --- Body twist from the stance feet, by least squares.
                //
                // A planted foot is fixed in the world, so its position r_i in the
                // BASE frame satisfies  -dr_i/dt = v + w x r_i.  Each stance foot
                // gives two equations in the three unknowns (vx, vy, w), so two or
                // more stance feet over-determine the twist and least squares is the
                // right estimator. Solved in centroid-reduced closed form (no matrix
                // inverse, so this still suits an embedded target):
                //     w = sum(r'_x q'_y - r'_y q'_x) / sum(|r'|^2)
                //     v = q_bar + w x r_bar
                // where q_i = -dr_i/dt and primes are deviations from the stance mean.
                //
                // This replaces a per-foot bearing sum (theta_sum of
                // atan2(X, Y) deltas) that attributed the WHOLE bearing change to
                // rotation. Body translation also swings a planted foot's bearing --
                // by +-0.59 rad/s per foot at 0.25 m/s on Go2 geometry -- and those
                // terms only cancel between left and right feet when the diagonal
                // pair is exactly symmetric. The reported yaw rate was therefore a
                // difference of large near-cancelling numbers, whose residual moved
                // with gait phase and contact timing. Verified against a synthetic
                // trot with an exactly known twist (scripts/leg_odom_model.py):
                //   estimator   vx/truth   wz/truth   wz std (rad/s), truth wz=0
                //   bearing sum   0.821      0.923      0.0675
                //   this one      0.900      1.000      0.0000
                // i.e. the old form injected +-3.9 deg/s of gait-synchronous noise
                // into a straight walk and under-read yaw rate ~8% while turning.
                // 0.900 is exactly gait_config.odom_scaler, as intended.
                //
                // A foot contributes only if it was ALSO in contact on the previous
                // sample: on the touchdown sample the position delta spans the SWING,
                // not the stance, and folding that in cost ~9% of forward speed.
                // (prev_foot_contacts_ was already maintained here but never read.)
                unsigned int total_contact = 0;
                float r_mid_x[4], r_mid_y[4], q_x[4], q_y[4];
                bool use[4];
                float rx_sum = 0, ry_sum = 0, qx_sum = 0, qy_sum = 0;

                for(unsigned int i = 0; i < 4; i++)
                {
                    geometry::Transformation current_foot_position = base_->legs[i]->foot_from_base();

                    bool foot_in_contact = base_->legs[i]->in_contact();
                    use[i] = foot_in_contact && prev_foot_contacts_[i];

                    r_mid_x[i] = 0.5f * (current_foot_position.X() + prev_foot_position_[i].X());
                    r_mid_y[i] = 0.5f * (current_foot_position.Y() + prev_foot_position_[i].Y());
                    q_x[i] = -(current_foot_position.X() - prev_foot_position_[i].X()) / dt;
                    q_y[i] = -(current_foot_position.Y() - prev_foot_position_[i].Y()) / dt;

                    if(use[i])
                    {
                        total_contact += 1;
                        rx_sum += r_mid_x[i];
                        ry_sum += r_mid_y[i];
                        qx_sum += q_x[i];
                        qy_sum += q_y[i];
                    }

                    prev_foot_position_[i] = current_foot_position;
                    prev_foot_contacts_[i] = foot_in_contact;
                }

                if(total_contact == 0)
                {
                    // Every stance foot landed this sample (the trot swaps its
                    // diagonal pairs at once): no usable displacement, so hold rather
                    // than fabricate one.
                    vel.linear.x = prev_vel_.linear.x;
                    vel.linear.y = prev_vel_.linear.y;
                    vel.angular.z = prev_vel_.angular.z;
                    prev_time_ = now;
                    return;
                }

                const float inv_n = 1.0f / float(total_contact);
                const float rx_bar = rx_sum * inv_n;
                const float ry_bar = ry_sum * inv_n;
                const float qx_bar = qx_sum * inv_n;
                const float qy_bar = qy_sum * inv_n;

                float num = 0, den = 0;
                for(unsigned int i = 0; i < 4; i++)
                {
                    if(!use[i])
                        continue;

                    const float px = r_mid_x[i] - rx_bar;
                    const float py = r_mid_y[i] - ry_bar;
                    const float qx = q_x[i] - qx_bar;
                    const float qy = q_y[i] - qy_bar;

                    num += (px * qy) - (py * qx);
                    den += (px * px) + (py * py);
                }
                // den == 0 means one usable foot (or coincident feet): yaw rate is
                // unobservable this sample, so carry the previous value.
                const float w = (den > 1e-9f) ? (num / den) : float(prev_vel_.angular.z);

                const float vx_raw = (qx_bar + (w * ry_bar)) * base_->gait_config.odom_scaler;
                const float vy_raw = (qy_bar - (w * rx_bar)) * base_->gait_config.odom_scaler;

                vel.linear.x =  ((1 - beta_) * vx_raw) + (beta_ * prev_vel_.linear.x);
                vel.linear.y =  ((1 - beta_) * vy_raw) + (beta_ * prev_vel_.linear.y);
                vel.angular.z = ((1 - beta_) * w)      + (beta_ * prev_vel_.angular.z);

                prev_vel_ = vel;
                prev_time_ = now;
            }
    };
}

#endif

