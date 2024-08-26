#![no_main]
#![no_std]
#![feature(future_join)]
#![feature(async_closure)]
#![feature(duration_millis_float)]
#![feature(let_chains)]
extern crate alloc;
extern crate uom;

use alloc::{boxed::Box, ffi::CString, format, sync::Arc, vec};
use core::{future::join, panic::PanicInfo, time::Duration};

use futures::{FutureExt, select_biased};
use motion_profiling::combined_mp::CombinedMP;
use nalgebra::Matrix3;
use uom::si::{angle::revolution, f64::Angle, length::meter};
use vexide::{
    core::sync::Mutex,
    devices::{
        controller::ControllerId,
        screen::{Text, TextSize},
        smart::GpsSensor,
    },
    prelude::*,
};

use subsystems::drivetrain::VoltageDrive;

use crate::{
    actuator::{motor_group::MotorGroup, telemetry::Telemetry},
    config::{
        DRIVE_RATIO, get_distance_1_offset, get_distance_2_offset, get_distance_3_offset,
        get_gps_offset, get_line_1_offset, track_width, wheel_diameter,
    },
    localization::localization::StateRepresentation,
    motion_control::ramsete::Ramsete,
    subsystems::{
        drivetrain::{Drivetrain, TankDrive},
        goal_clamp::{GoalClamp, GoalController},
        hook::{Hook, HookPosition},
        intake::{Intake, IntakeManual, LoadGoal},
    },
};

mod actuator;
mod config;
mod localization;
mod motion_control;
mod sensor;
mod subsystems;
mod utils;

struct Robot {
    drivetrain: Drivetrain,
    intake: Intake,
    hook: Hook,
    controller_primary: Controller,
    controller_partner: Controller,
    goal_clamp: GoalClamp,
    _telemetry: Telemetry,
    _telemetry_task: Task<()>,
}

impl Robot {
    async fn new(mut peripherals: Peripherals) -> Self {
        let _telemetry = Telemetry::new(SerialPort::open(
            peripherals.port_20,
            SerialPort::MAX_BAUD_RATE,
        ));

        // TODO Fix drivetrain encoders with 5.5W behavior

        let drivetrain = Drivetrain::new(
            Arc::new(Mutex::new(MotorGroup::new(vec![
                Motor::new(peripherals.port_4, Gearset::Green, Direction::Forward),
                Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
            ]))),
            Arc::new(Mutex::new(MotorGroup::new(vec![
                Motor::new(peripherals.port_9, Gearset::Green, Direction::Reverse),
                Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
            ]))),
            InertialSensor::new(peripherals.port_19),
            wheel_diameter(),
            DRIVE_RATIO,
            _telemetry.clone(),
            vec![
                (
                    DistanceSensor::new(peripherals.port_12),
                    get_distance_1_offset(),
                ),
                (
                    DistanceSensor::new(peripherals.port_13),
                    get_distance_2_offset(),
                ),
                (
                    DistanceSensor::new(peripherals.port_14),
                    get_distance_3_offset(),
                ),
            ],
            vec![(AdiLineTracker::new(peripherals.adi_b), get_line_1_offset())],
            GpsSensor::new(peripherals.port_11, get_gps_offset(), ((0.0, 0.0), 0.0)),
        )
            .await;

        Self {
            drivetrain,
            intake: Intake::new(
                Motor::new(peripherals.port_10, Gearset::Green, Direction::Reverse),
                Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_17, Gearset::Red, Direction::Forward),
            ),
            hook: Hook::new(Motor::new(
                peripherals.port_8,
                Gearset::Green,
                Direction::Reverse,
            )),
            controller_primary: peripherals.primary_controller,
            controller_partner: peripherals.partner_controller,
            goal_clamp: GoalClamp::new(AdiDigitalOut::new(peripherals.adi_a)),
            _telemetry: _telemetry.clone(),
            _telemetry_task: spawn(async move {
                let mut text = Text::new_aligned(
                    "test-text",
                    TextSize::Small,
                    (0, 0),
                    Default::default(),
                    Default::default(),
                );
                loop {
                    text.text = CString::new(format!("Current time: {}", 0.0)).unwrap();
                    peripherals.screen.fill(&text, Rgb::WHITE);

                    sleep(Duration::from_millis(50)).await;
                }
            }),
        }
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("auto");

        self.drivetrain
            .init_norm(
                &StateRepresentation::new(0.0, 0.0, 0.0),
                &Matrix3::new(0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0),
            )
            .await;

        self.hook
            .run(HookPosition(Angle::new::<revolution>(0.0)))
            .await;

        let ramsete = Ramsete::try_new(
            0.1,
            0.5,
            Box::new(
                CombinedMP::try_new_2d(
                    serde_json::from_str(include_str!("../bins/paths/test.json")).unwrap(),
                    track_width().get::<meter>(),
                )
                    .unwrap(),
            ),
        )
            .unwrap();

        self.drivetrain.run_velocity(ramsete).await;

        {
            let drive_state = self.drivetrain.run(VoltageDrive::new(-12.0, -12.0));

            let _ = select_biased! {
                () = drive_state.fuse() => 1,
                () = sleep(Duration::from_secs(2)).fuse() => 2,
            };
        }

        self.drivetrain.run(VoltageDrive::new(0.0, 0.0)).await;
    }

    async fn driver(&mut self) {
        println!("Drive");
        // let drive_state = self.drivetrain.run(TankDrive::new(&self.controller));

        join!(self.goal_clamp.run(GoalController {
            controller: &mut self.controller_primary
        }))
            .await;
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals).await;
    robot.compete().await;
}
