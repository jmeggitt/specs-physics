use std::marker::PhantomData;

use specs::{Entities, Read, System, Write, WriteExpect};

use crate::events::{ContactEvent, ContactEvents, ProximityEvent, ProximityEvents};
use crate::{parameters::TimeStep, Physics, PhysicsWorld};
use nalgebra::RealField;
use nphysics::force_generator::DefaultForceGeneratorSet;
use nphysics::joint::DefaultJointConstraintSet;
use nphysics::object::{DefaultBodySet, DefaultColliderSet};

pub type StorageSets<'a, N> = (
    WriteExpect<'a, DefaultBodySet<N>>,
    WriteExpect<'a, DefaultColliderSet<N>>,
    WriteExpect<'a, DefaultJointConstraintSet<N>>,
    WriteExpect<'a, DefaultForceGeneratorSet<N>>,
);

/// The `PhysicsStepperSystem` progresses the nphysics `World`.
pub struct PhysicsStepperSystem<N>(PhantomData<N>);

impl<'s, N: RealField> System<'s> for PhysicsStepperSystem<N> {
    type SystemData = (
        Entities<'s>,
        Option<Read<'s, TimeStep<N>>>,
        Write<'s, ContactEvents>,
        Write<'s, ProximityEvents>,
        PhysicsWorld<'s, N>,
        StorageSets<'s, N>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, time_step, mut contact_events, mut proximity_events, mut physics, storage) =
            data;

        let (mut bodies, mut colliders, mut joints, mut forces) = storage;

        // if a TimeStep resource exits, set the timestep for the nphysics integration
        // accordingly; this should not be required if the Systems are executed in a
        // fixed interval
        if let Some(time_step) = time_step {
            // only update timestep if it actually differs from the current nphysics World
            // one; keep in mind that changing the Resource will destabilize the simulation
            if physics.mechanical_world.timestep() != time_step.0 {
                warn!(
                    "TimeStep and mechanical_world.timestep() differ, changing worlds timestep from {} to: {:?}",
                    physics.mechanical_world.timestep(),
                    time_step.0
                );
                physics
                    .mechanical_world
                    .integration_parameters
                    .set_dt(time_step.0);
            }
        }

        let Physics {
            ref mut mechanical_world,
            ref mut geometric_world,
            ..
        } = *physics;

        mechanical_world.step(
            geometric_world,
            &mut *bodies,
            &mut *colliders,
            &mut *joints,
            &mut *forces,
        );

        // Map occurred ncollide ContactEvents to a custom ContactEvent type
        let contact_iter = geometric_world.contact_events().iter();
        contact_events.iter_write(
            contact_iter.map(|x| ContactEvent::from_ncollide(*x, &entities, &colliders).unwrap()),
        );

        // Map occurred ncollide ProximityEvents to a custom ProximityEvent type
        let proximity_iter = geometric_world.proximity_events().iter();
        proximity_events.iter_write(
            proximity_iter
                .map(|x| ProximityEvent::from_ncollide(*x, &entities, &colliders).unwrap()),
        );
    }
}

impl<N> Default for PhysicsStepperSystem<N> {
    fn default() -> Self {
        Self(PhantomData)
    }
}
