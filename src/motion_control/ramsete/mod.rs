use alloc::boxed::Box;

use command_rs::command::Command;
use motion_profiling::motion_profile::MotionProfile;
use nalgebra::Matrix3;
use uom::{
    num_traits::{real::Real, Pow},
    si::{
        angular_velocity::radian_per_second, f64::AngularVelocity, length::meter,
        velocity::meter_per_second,
    },
};
use vexide::core::time::Instant;

use crate::{
    config::{track_width, wheel_diameter},
    utils::angle_difference,
};

pub struct Ramsete {
    zeta: f64,
    beta: f64,
    motion_profile: Box<dyn MotionProfile>,
    start_time: Instant,
}

#[derive(Debug)]
pub enum RamseteError {
    InvalidBeta,
}

impl Ramsete {
    pub fn try_new(
        zeta: f64,
        beta: f64,
        motion_profile: Box<dyn MotionProfile>,
    ) -> Result<Self, RamseteError> {
        if 0.0 < beta && 1.0 > beta {
            Err(RamseteError::InvalidBeta)
        } else {
            Ok(Self {
                zeta,
                beta,
                motion_profile,
                start_time: Instant::now(),
            })
        }
    }
}

impl<'a> Command for Ramsete {
    fn initialize(&mut self) {
        self.start_time = Instant::now();
    }

    fn execute(&mut self) -> Option<(AngularVelocity, AngularVelocity)> {
        let i = todo!();
        let command = self.motion_profile.get(Instant::now() - self.start_time)?;

        let error = Matrix3::new(
            i.z.cos(),
            i.z.sin(),
            0.0,
            -i.z.sin(),
            i.z.cos(),
            0.0,
            0.0,
            0.0,
            1.0,
        ) * (command.desired_pose - i);

        let k = 2.0
            * self.zeta
            * (command.desired_angular.get::<radian_per_second>().pow(2) as f64
                + self.beta * command.desired_velocity.get::<meter_per_second>().pow(2) as f64)
                .sqrt();

        let velocity_commanded =
            command.desired_velocity.get::<meter_per_second>() * error.z.cos() + k * error.x;
        let angular_wheel_velocity_commanded = (command.desired_angular.get::<radian_per_second>()
            + k * angle_difference(error.z, 0.0)
            + self.beta
                * command.desired_velocity.get::<meter_per_second>()
                * error.z.simd_sinc()
                * error.y)
            * track_width().get::<meter>()
            / 2.0;

        Some((
            AngularVelocity::new::<radian_per_second>(
                (velocity_commanded - angular_wheel_velocity_commanded)
                    / (wheel_diameter().get::<meter>() / 2.0),
            ),
            AngularVelocity::new::<radian_per_second>(
                (velocity_commanded + angular_wheel_velocity_commanded)
                    / (wheel_diameter().get::<meter>() / 2.0),
            ),
        ))
    }
}
