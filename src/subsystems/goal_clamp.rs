use alloc::sync::Arc;
use core::cell::RefCell;

use command_rs::command::Command;
use vexide::{devices::adi::digital::LogicLevel, prelude::AdiDigitalOut};

pub struct GoalClamp {
    adi_solenoid: AdiDigitalOut,
}

impl GoalClamp {
    pub fn new(adi_solenoid: AdiDigitalOut) -> Self {
        Self { adi_solenoid }
    }

    pub fn set_value(&mut self, level: LogicLevel) {
        self.adi_solenoid.set_level(level).unwrap()
    }
}

pub struct GoalController<'a> {
    pub logic_level: LogicLevel,
    pub goal_clamp: Arc<RefCell<GoalClamp>>,
}

impl<'a> Command for GoalController<'a> {
    fn initialize(&mut self) {
        self.goal_clamp.borrow_mut().set_value(self.logic_level);
    }
}
