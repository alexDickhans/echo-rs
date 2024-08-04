use alloc::sync::Arc;
use core::time::Duration;

use vexide::{core::sync::Mutex, prelude::*};

use crate::{localization::localization::ParticleFilter, state_machine::*};

/// Example implementation of a drivetrain subsystem.
pub struct Drivetrain {
    left_motor: Motor,
    right_motor: Motor,
}

impl Drivetrain {
    pub fn new(left_motor: Motor, right_motor: Motor) -> Self {
        Self {
            left_motor,
            right_motor,
        }
    }
}

impl Subsystem<(f32, f32)> for Drivetrain {
    async fn run(&mut self, mut state: impl State<(f32, f32)>) {
        state.init().await;
        while let Some(output) = state.update().await {
            let _ = self.left_motor.set_voltage(output.0 as f64);
            let _ = self.right_motor.set_voltage(output.1 as f64);

            sleep(Duration::from_millis(10)).await;
        }
    }
}

pub struct TankDrive<'a> {
    controller: &'a Controller,
}

impl<'a> TankDrive<'a> {
    pub fn new(controller: &'a Controller) -> Self {
        TankDrive { controller }
    }
}

impl<'a> State<(f32, f32)> for TankDrive<'a> {
    async fn update(&mut self) -> Option<(f32, f32)> {
        Some((
            self.controller.left_stick.y().ok()? * 12.0,
            self.controller.right_stick.y().ok()? * 12.0,
        ))
    }
}

pub struct VoltageDrive {
    left_voltage: f64,
    right_voltage: f64,
    localization: Arc<Mutex<ParticleFilter<100>>>,
}

impl VoltageDrive {
    pub fn new(
        left_voltage: f64,
        right_voltage: f64,
        localization: Arc<Mutex<ParticleFilter<100>>>,
    ) -> Self {
        Self {
            left_voltage,
            right_voltage,
            localization,
        }
    }
}

impl State<(f32, f32)> for VoltageDrive {
    async fn update(&mut self) -> Option<(f32, f32)> {
        Some((self.left_voltage as f32, self.right_voltage as f32))
    }
}
