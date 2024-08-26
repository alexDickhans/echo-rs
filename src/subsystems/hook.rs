use uom::si::{angle::revolution, f64::Angle};
use vexide::prelude::{Motor, Position};

pub struct Hook {
    motor: Motor,
}

impl Hook {
    pub fn new(motor: Motor) -> Self {
        Self { motor }
    }

    pub fn set_position(&mut self, angle: Angle) {
        self.motor
            .set_position_target(Position::from_revolutions(angle.get::<revolution>()), 600)
            .unwrap()
    }
}

pub struct HookPosition(pub Angle);

impl State<(), f64> for HookPosition {
    fn update(&mut self, _: &()) -> Option<f64> {
        Some(self.0.get::<revolution>())
    }
}
