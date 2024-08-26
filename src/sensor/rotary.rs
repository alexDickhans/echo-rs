use alloc::sync::Arc;
use core::cell::RefCell;

use uom::si::{f64::Length, length::meter};

use crate::actuator::motor_group::MotorGroup;

pub trait RotarySensor {
    async fn pos(&self) -> f64;
}

impl RotarySensor for Arc<RefCell<MotorGroup>> {
    async fn pos(&self) -> f64 {
        self.borrow().position()
    }
}

pub struct TrackingWheel<T: RotarySensor> {
    sensor: T,
    diameter: Length,
    gearing: Option<f64>,
    last_length: f64,
}

impl<T: RotarySensor> TrackingWheel<T> {
    pub fn new(sensor: T, diameter: Length, gearing: Option<f64>) -> Self {
        Self {
            sensor,
            diameter,
            gearing,
            last_length: 0.0,
        }
    }

    pub async fn travel(&self) -> f64 {
        self.sensor.pos().await * self.diameter.get::<meter>() * self.gearing.unwrap_or(1.0)
    }

    pub async fn update(&mut self) -> f64 {
        let current_length = self.travel().await;

        let delta = current_length - self.last_length;

        self.last_length = current_length;

        delta
    }
}
