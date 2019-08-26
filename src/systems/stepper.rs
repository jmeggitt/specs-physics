use crate::PhysicsWorld;
use crate::storage::{PhysicsStorage, BodyStorage, SpecsWrapper};
use nphysics::object::{Body, BodyPartHandle, BodySet, Collider, ColliderRemovalData, ColliderSet};
use nphysics::joint::{JointConstraint, JointConstraintSet};
use nphysics::force_generator::{ForceGenerator, ForceGeneratorSet};
use specs::System;
use specs::{WriteStorage, WriteExpect, Write};
use specs::world::Index;
use std::marker::PhantomData;
use nalgebra::RealField;

#[derive(Default)]
pub struct PhysicsStepper<N>(PhantomData<N>);

impl<'s, N: RealField> System<'s> for PhysicsStepper<N> {
    type SystemData = (WriteExpect<'s, PhysicsWorld<N>>,
        Write<'s, BodyStorage<N>>,
        Write<'s, PhysicsStorage<SpecsWrapper<Collider<N, Index>>>>,
        Write<'s, PhysicsStorage<SpecsWrapper<Box<dyn JointConstraint<N, BodyStorage<N>>>>>>,
        Write<'s, PhysicsStorage<SpecsWrapper<Box<dyn ForceGenerator<N, BodyStorage<N>>>>>>,
    );

    fn run(&mut self, (mut world, mut bodies, mut colliders, mut joints, mut forces): Self::SystemData) {
        let (geometric, mechanical) = world.split();
        mechanical.step(geometric, &mut bodies, &mut *colliders, &mut *joints, &mut *forces);
    }
}