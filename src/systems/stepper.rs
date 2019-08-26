use crate::storage::{BodyStorage, PhysicsStorage, SpecsWrapper};
use crate::PhysicsWorld;
use nalgebra::RealField;
use nphysics::force_generator::ForceGenerator;
use nphysics::joint::JointConstraint;
use nphysics::object::Collider;
use specs::world::Index;
use specs::System;
use specs::{Write, WriteExpect};
use std::marker::PhantomData;

#[derive(Default)]
pub struct PhysicsStepper<N>(PhantomData<N>);

impl<'s, N: RealField> System<'s> for PhysicsStepper<N> {
    type SystemData = (
        WriteExpect<'s, PhysicsWorld<N>>,
        Write<'s, BodyStorage<N>>,
        Write<'s, PhysicsStorage<SpecsWrapper<Collider<N, Index>>>>,
        Write<'s, PhysicsStorage<SpecsWrapper<Box<dyn JointConstraint<N, BodyStorage<N>>>>>>,
        Write<'s, PhysicsStorage<SpecsWrapper<Box<dyn ForceGenerator<N, BodyStorage<N>>>>>>,
    );

    fn run(
        &mut self,
        (mut world, mut bodies, mut colliders, mut joints, mut forces): Self::SystemData,
    ) {
        let (geometric, mechanical) = world.split();
        mechanical.step(
            geometric,
            &mut bodies,
            &mut *colliders,
            &mut *joints,
            &mut *forces,
        );
    }
}
