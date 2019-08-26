extern crate ncollide2d as ncollide;
extern crate nphysics2d as nphysics;

use nalgebra::{RealField, Vector2};
use nphysics2d::world::{GeometricalWorld, MechanicalWorld};
use specs::world::Index;

#[allow(unused_variables)]
pub mod storage;
use storage::BodyStorage;
mod systems;

pub struct PhysicsWorld<N: RealField> {
    pub geometric: GeometricalWorld<N, Index, Index>,
    pub mechanical: MechanicalWorld<N, BodyStorage<N>, Index>,
}

impl<N: RealField> Default for PhysicsWorld<N> {
    fn default() -> Self {
        Self {
            geometric: GeometricalWorld::new(),
            mechanical: MechanicalWorld::new(Vector2::new(N::zero(), N::zero())),
        }
    }
}

impl<N: RealField> PhysicsWorld<N> {
    pub fn split(
        &mut self,
    ) -> (
        &mut GeometricalWorld<N, Index, Index>,
        &mut MechanicalWorld<N, BodyStorage<N>, Index>,
    ) {
        (&mut self.geometric, &mut self.mechanical)
    }
}
