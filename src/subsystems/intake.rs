use core::time::Duration;

use uom::si::{angular_velocity::revolution_per_minute, f64::AngularVelocity};
use vexide::prelude::{sleep, Controller, Motor, Position};

use crate::config::{INTAKE_RATIO, LIFT_RATIO};

pub struct Intake {
    bottom: Motor,
    top: Motor,
    lift: Motor,
}

pub struct IntakeState {
    bottom_ring: bool,
    top_1_ring: bool,
    top_2_ring: bool,
}

pub enum TopCommand {
    Velocity(i32),
    Position(f64),
}

pub struct IntakeCommand {
    bottom_speed: AngularVelocity,
    top_command: TopCommand,
    lift_position: (f64, i32),
}

impl Intake {
    pub fn new(bottom: Motor, top: Motor, lift: Motor) -> Self {
        Self { bottom, top, lift }
    }

    pub async fn run(&mut self, mut state: impl State<f64, IntakeCommand>) {
        state.init();

        loop {
            if let Some(command) = state.update(
                &(self
                    .top
                    .position()
                    .unwrap_or(Position::from_revolutions(0.0))
                    .as_revolutions()
                    / INTAKE_RATIO),
            ) {
                let _ = self.lift.set_position_target(
                    Position::from_revolutions(command.lift_position.0 * LIFT_RATIO),
                    command.lift_position.1,
                );

                let _ = match command.top_command {
                    TopCommand::Position(pos) => self
                        .top
                        .set_position_target(Position::from_revolutions(pos * INTAKE_RATIO), 600),
                    TopCommand::Velocity(vel) => self.top.set_velocity(vel),
                };

                let _ = self
                    .bottom
                    .set_velocity(command.bottom_speed.get::<revolution_per_minute>() as i32);
            } else {
                return;
            }

            sleep(Duration::from_millis(10)).await;
        }
    }
}

pub struct LoadGoal;

impl LoadGoal {
    pub fn new() -> Self {
        Self
    }
}

impl State<f64, IntakeCommand> for LoadGoal {
    fn update(&mut self, _: &f64) -> Option<IntakeCommand> {
        Some(IntakeCommand {
            bottom_speed: AngularVelocity::new::<revolution_per_minute>(600.0),
            top_command: TopCommand::Velocity(600),
            lift_position: (0.0, 600),
        })
    }
}

pub struct IntakeManual<'a> {
    pub controller: &'a mut Controller,
    pub lift_pos: f64,
    pub top_pos: f64,
}

impl<'a> State<f64, IntakeCommand> for IntakeManual<'a> {
    fn update(&mut self, _: &f64) -> Option<IntakeCommand> {
        if self
            .controller
            .right_trigger_1
            .was_pressed()
            .unwrap_or(false)
        {
            self.top_pos += 1.0;
        } else if self
            .controller
            .right_trigger_2
            .was_pressed()
            .unwrap_or(false)
        {
            self.top_pos -= 1.0;
        }

        if self
            .controller
            .left_trigger_1
            .was_pressed()
            .unwrap_or(false)
        {
            self.lift_pos += 1.0;
        } else if self
            .controller
            .left_trigger_2
            .was_pressed()
            .unwrap_or(false)
        {
            self.lift_pos -= 1.0;
        }

        Some(IntakeCommand {
            bottom_speed: AngularVelocity::new::<revolution_per_minute>(
                (self.controller.right_stick.y().unwrap_or(0.0) * 200.0).into(),
            ),
            top_command: TopCommand::Position(self.top_pos),
            lift_position: (self.lift_pos, 600),
        })
    }
}
