use nalgebra::{Rotation2, Vector2};
use rand::{rngs::SmallRng, Rng, SeedableRng};
use rand_distr::Normal;
use uom::si::{
    angle::{degree, radian},
    f64::Angle,
};
use vexide::prelude::InertialSensor;

use crate::{
    localization::localization::StateRepresentation,
    sensor::rotary::{RotarySensor, TrackingWheel},
};

pub struct TankPoseTracking<T: RotarySensor> {
    left_side: TrackingWheel<T>,
    right_side: TrackingWheel<T>,
    orientation: InertialSensor,
    left_delta: f64,
    right_delta: f64,
    heading: Rotation2<f64>,
    rng: SmallRng,
    pub drive_noise: f64,
    pub angle_noise: f64,
}

impl<T: RotarySensor> TankPoseTracking<T> {
    pub async fn new(
        left_side: TrackingWheel<T>,
        right_side: TrackingWheel<T>,
        mut orientation: InertialSensor,
        drive_noise: f64,
        angle_noise: f64,
    ) -> Self {
        orientation.calibrate().await.expect("Failed to calibrate");
        Self {
            left_side,
            right_side,
            orientation,
            left_delta: 0.0,
            right_delta: 0.0,
            drive_noise,
            angle_noise,
            heading: Rotation2::new(0.0),
            rng: SmallRng::seed_from_u64(0),
        }
    }

    pub async fn update(&mut self) {
        // Update the deltas for each side of the drive this frame
        self.left_delta = self.left_side.update().await;
        self.right_delta = self.right_side.update().await;

        // Update the heading once per frame
        self.heading = self.orientation();
    }

    pub fn predict(&mut self) -> StateRepresentation {
        // Calculate noise for each side of the drive
        let left_noisy = self
            .rng
            .sample(Normal::new(self.left_delta, self.drive_noise * self.left_delta).unwrap());
        let right_noisy = self
            .rng
            .sample(Normal::new(self.right_delta, self.drive_noise * self.right_delta).unwrap());

        // Because we are calculating from the center of the drive the mean is the displacement of
        // the center of rotation
        let mean = left_noisy + right_noisy / 2.0;

        // Calculate a local displacement vector from the current position of the robot
        let local = Rotation2::new(
            self.heading.angle() + self.rng.sample(Normal::new(0.0, self.angle_noise).unwrap()),
        ) * Vector2::new(mean, 0.0);

        // Create a new state representation without a angle(z) change
        // We do this because the IMU is more than accurate over the entire match
        StateRepresentation::new(local.x, local.y, 0.0)
    }

    pub fn orientation(&self) -> Rotation2<f64> {
        // Convert to a Rotation2 object to use for linear algebra
        Rotation2::new(
            Angle::new::<degree>(-self.orientation.heading().expect("Failed to read from IMU"))
                .get::<radian>(),
        )
    }
}
