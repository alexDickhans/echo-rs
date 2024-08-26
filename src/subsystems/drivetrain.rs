use alloc::{sync::Arc, vec::Vec};
use core::{f64::consts::TAU, ops::Add, time::Duration};
use core::cell::RefCell;

use command_rs::command::Command;
use command_rs::subsystem::AnySubsystem;
use nalgebra::{Matrix3, Vector2};
use uom::si::f64::Length;
use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::{PortError, smart::GpsSensor},
    prelude::*,
};

use crate::{
    actuator::{motor_group::MotorGroup, telemetry::Telemetry},
    config::{
        ANGLE_NOISE, distance_threshold, DRIVE_NOISE, FIELD_MAX, LINE_SENSOR_THRESHOLD,
        localization_min_update_distance, LOCALIZATION_MIN_UPDATE_INTERVAL, NUM_PARTICLES, TELEMETRY_ENABLED,
    },
    localization::{
        localization::{Localization, particle_filter::ParticleFilter, StateRepresentation},
        predict::tank_pose_tracking::TankPoseTracking,
        sensor::{distance::WallDistanceSensor, line_tracker::LineTrackerSensor},
    },
    sensor::rotary::TrackingWheel,
};

/// Example implementation of a drivetrain subsystem.
#[allow(dead_code)]
pub struct Drivetrain {
    left_motor: Arc<RefCell<MotorGroup>>,
    right_motor: Arc<RefCell<MotorGroup>>,
    localization: Arc<RefCell<ParticleFilter<NUM_PARTICLES>>>,
    _localization_task: Task<()>,
    telemetry: Telemetry,
}

impl Drivetrain {
    pub async fn new(
        left_motor: Arc<RefCell<MotorGroup>>,
        right_motor: Arc<RefCell<MotorGroup>>,
        imu: InertialSensor,
        tracking_wheel_diameter: Length,
        drive_ratio: f64,
        telemetry: Telemetry,
        distance_sensors: Vec<(DistanceSensor, StateRepresentation)>,
        line_sensors: Vec<(AdiLineTracker, Vector2<f64>)>,
        gps: Result<GpsSensor, PortError>,
    ) -> Self {
        let localization = Arc::new(RefCell::new(ParticleFilter::new(
            TankPoseTracking::new(
                TrackingWheel::new(
                    left_motor.clone(),
                    tracking_wheel_diameter,
                    Option::from(drive_ratio),
                ),
                TrackingWheel::new(
                    right_motor.clone(),
                    tracking_wheel_diameter,
                    Option::from(drive_ratio),
                ),
                imu,
                DRIVE_NOISE,
                ANGLE_NOISE,
            )
                .await,
            LOCALIZATION_MIN_UPDATE_INTERVAL,
            localization_min_update_distance(),
        )));

        {
            let mut loc_lock = localization.borrow_mut();

            for (sensor, pose) in distance_sensors {
                loc_lock.add_sensor(WallDistanceSensor::new(sensor, pose));
            }

            for (sensor, pose) in line_sensors {
                loc_lock.add_sensor(LineTrackerSensor::new(
                    sensor,
                    pose,
                    LINE_SENSOR_THRESHOLD,
                    distance_threshold(),
                ));
            }

            if let Ok(gps_ok) = gps {
                loc_lock.add_sensor(gps_ok);
            }

            // loc_lock.add_sensor(DummySensor {
            //     covariance: 0.5,
            //     mean: Default::default(),
            // });

            loc_lock.init_uniform(
                &StateRepresentation::new(-FIELD_MAX, -FIELD_MAX, 0.0),
                &StateRepresentation::new(FIELD_MAX, FIELD_MAX, TAU),
            );
        }

        Self {
            localization: localization.clone(),
            telemetry: telemetry.clone(),
            _localization_task: spawn(async move {
                loop {
                    let now = Instant::now();

                    let mut loc = localization.borrow_mut();

                    loc.update().await;

                    drop(loc);

                    if TELEMETRY_ENABLED {
                        telemetry.send_json(loc.get_estimates().to_vec()).await;
                        telemetry.send("\n".as_bytes()).await;
                    }

                    sleep_until(now.add(Duration::from_millis(10))).await;
                }
            }),
            left_motor,
            right_motor,
        }
    }

    pub fn init_norm(&mut self, mean: &StateRepresentation, covariance: &Matrix3<f64>) {
        self.localization.borrow_mut().init_norm(mean, covariance);
    }

    pub fn get_pose(&self) -> StateRepresentation {
        self.localization.borrow().pose_estimate()
    }

    pub fn move_
}

pub struct TankDrive<'a> {
    drivetrain: Arc<RefCell<Drivetrain>>,
    controller: &'a Controller,
}

impl<'a> TankDrive<'a> {
    pub fn new(drivetrain: Arc<RefCell<Drivetrain>>, controller: &'a Controller) -> Self {
        TankDrive { drivetrain, controller }
    }
}

impl<'a> Command for TankDrive<'a> {
    fn execute(&mut self) {
        todo!()
    }

    fn requirements(&self) -> &[AnySubsystem] {
        &[self.drivetrain.clone().into()]
    }
}

pub struct VoltageDrive {
    drivetrain: Arc<RefCell<Drivetrain>>,
    left_voltage: f64,
    right_voltage: f64,
}

impl VoltageDrive {
    pub fn new(left_voltage: f64, right_voltage: f64) -> Self {
        Self {
            left_voltage,
            right_voltage,
        }
    }
}

impl Command for VoltageDrive {
    fn initialize(&mut self) {
        self.drivetrain.borrow_mut().
    }

    fn requirements(&self) -> &[AnySubsystem] {
        &[self.drivetrain.clone().into()]
    }
}
