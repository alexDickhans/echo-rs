use alloc::sync::Arc;
use vexide::core::sync::Mutex;
use crate::actuator::motor_group::MotorGroup;

pub trait RotarySensor {
    async fn pos(&self) -> f64;
}

impl RotarySensor for Arc<Mutex<MotorGroup>> {
    async fn pos(&self) -> f64 {
        self.lock().await.position()
    }
}

pub struct TrackingWheel<T: RotarySensor> {
    sensor: T,
    // offset: f64,
    diameter: f64,
    gearing: Option<f64>,
    last_length: f64,
}

impl<T: RotarySensor> TrackingWheel<T> {
    pub fn new(sensor: T, diameter: f64, gearing: Option<f64>) -> Self {
        Self {
            sensor,
            diameter,
            // offset,
            gearing,
            last_length: 0.0,
        }
    }

    pub async fn travel(&self) -> f64 {
        self.sensor.pos().await * self.diameter * self.gearing.unwrap_or(1.0)
    }

    pub async fn update(&mut self) -> f64 {
        let current_length = self.travel().await;

        let delta = current_length - self.last_length;

        self.last_length = current_length;

        delta
    }
}