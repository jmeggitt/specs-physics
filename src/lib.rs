#![feature(unboxed_closures)]
#![allow(unused_imports)]

extern crate ncollide2d as ncollide;
extern crate nphysics2d as nphysics;

use nalgebra::RealField;
use nphysics2d::object::{Body, BodyHandle, BodySet, ColliderHandle, ColliderSet};
use nphysics2d::world::{GeometricalWorld, MechanicalWorld};
use specs::storage::{DenseVecStorage, FlaggedStorage, GenericWriteStorage, WriteStorage};
use specs::world::Index;
use specs::{Component, Entity, SystemData, World, WriteExpect, Join};
use std::ops::{Deref, DerefMut};


#[allow(unused_variables)]
pub mod storage;
use storage::{BodyStorage};
mod systems;

pub struct PhysicsWorld<N: RealField> {
    pub geometric: GeometricalWorld<N, Index, Index>,
    pub mechanical: MechanicalWorld<N, BodyStorage<N>, Index>,
}

impl<N: RealField> PhysicsWorld<N> {
    pub fn split(&mut self) -> (&mut GeometricalWorld<N, Index, Index>, &mut MechanicalWorld<N, BodyStorage<N>, Index>) {
        (&mut self.geometric, &mut self.mechanical)
    }
}
