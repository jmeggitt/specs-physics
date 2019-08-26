#![feature(unboxed_closures)]
#![allow(unused_imports)]

extern crate ncollide2d as ncollide;
extern crate nphysics2d as nphysics;

use amethyst_assets::Handle;
use nalgebra::RealField;
use nphysics2d::object::{Body, BodyHandle, BodySet, ColliderHandle, ColliderSet};
use nphysics2d::world::{GeometricalWorld, MechanicalWorld};
use specs::storage::{DenseVecStorage, FlaggedStorage, GenericWriteStorage, WriteStorage};
use specs::world::Index;
use specs::{Component, Entity, SystemData, World, WriteExpect};
use std::ops::{Deref, DerefMut};

// TODO: Remove later (here for convenience)
pub trait Real: RealField + Default {}
impl<T: RealField + Default> Real for T {}

#[allow(dead_code)]
#[allow(unused_variables)]
pub mod storageb;
use storageb::{BodyStorage};
mod systems;

pub struct PhysicsWorld<N: Real> {
    pub geometric: GeometricalWorld<N, Index, Index>,
    pub mechanical: MechanicalWorld<N, BodyStorage<N>, Index>,
}
