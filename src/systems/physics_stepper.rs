use std::marker::PhantomData;

use specs::{world::Index, Entities, Entity, Read, Resources, System, SystemData, Write,
            WriteExpect};

use crate::events::{ContactEvent, ContactEvents, ContactType, ProximityEvent, ProximityEvents};
use crate::{parameters::TimeStep, Physics};
use nalgebra::RealField;
use ncollide::pipeline::ContactEvent as NContactEvent;
use nphysics::force_generator::DefaultForceGeneratorSet;
use nphysics::joint::DefaultJointConstraintSet;
use nphysics::object::{DefaultBodySet, DefaultColliderHandle, DefaultColliderSet};

pub type StorageSets<'a, N> = (
    WriteExpect<'a, DefaultBodySet<N>>,
    WriteExpect<'a, DefaultColliderSet<N>>,
    WriteExpect<'a, DefaultJointConstraintSet<N>>,
    WriteExpect<'a, DefaultForceGeneratorSet<N>>,
);

/// The `PhysicsStepperSystem` progresses the nphysics `World`.
pub struct PhysicsStepperSystem<N> {
    _phantom: PhantomData<N>,
}

impl<'s, N: RealField> System<'s> for PhysicsStepperSystem<N> {
    type SystemData = (
        Entities<'s>,
        Option<Read<'s, TimeStep<N>>>,
        Write<'s, ContactEvents>,
        Write<'s, ProximityEvents>,
        Write<'s, Physics<N>>,
        StorageSets<'s, N>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            time_step,
            mut contact_events,
            mut proximity_events,
            mut physics,
            storage,
        ) = data;

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

        // map occurred ncollide ContactEvents to a custom ContactEvent type; this
        // custom type contains data that is more relevant for Specs users than
        // CollisionObjectHandles, such as the Entities that took part in the collision
        contact_events.iter_write(
            geometric_world
                .contact_events()
                .iter()
                .map(|contact_event| {
                    debug!("Got ContactEvent: {:?}", contact_event);
                    // retrieve CollisionObjectHandles from ContactEvent and map the ContactEvent
                    // type to our own custom ContactType
                    let (handle1, handle2, contact_type) = match contact_event {
                        NContactEvent::Started(handle1, handle2) => {
                            (*handle1, *handle2, ContactType::Started)
                        }
                        NContactEvent::Stopped(handle1, handle2) => {
                            (*handle1, *handle2, ContactType::Stopped)
                        }
                    };

                    // create our own ContactEvent from the extracted data; mapping the
                    // CollisionObjectHandles to Entities is error prone but should work as intended
                    // as long as we're the only ones working directly with the nphysics World
                    ContactEvent {
                        collider1: entity_from_collision_object_handle(
                            &entities, handle1, &colliders,
                        ),
                        collider2: entity_from_collision_object_handle(
                            &entities, handle2, &colliders,
                        ),
                        contact_type,
                    }
                }),
        );

        // map occurred ncollide ProximityEvents to a custom ProximityEvent type; see
        // ContactEvents for reasoning
        proximity_events.iter_write(geometric_world.proximity_events().iter().map(
            |proximity_event| {
                debug!("Got ProximityEvent: {:?}", proximity_event);
                // retrieve CollisionObjectHandles and Proximity statuses from the ncollide
                // ProximityEvent
                let (handle1, handle2, prev_status, new_status) = (
                    proximity_event.collider1,
                    proximity_event.collider2,
                    proximity_event.prev_status,
                    proximity_event.new_status,
                );

                // create our own ProximityEvent from the extracted data; mapping
                // CollisionObjectHandles to Entities is once again error prone, but yeah...
                // ncollides Proximity types are mapped to our own types
                ProximityEvent {
                    collider1: entity_from_collision_object_handle(&entities, handle1, &colliders),
                    collider2: entity_from_collision_object_handle(&entities, handle2, &colliders),
                    prev_status,
                    new_status,
                }
            },
        ));
    }

    // TODO: This can probably be deleted without causing any issues
    fn setup(&mut self, res: &mut Resources) {
        info!("PhysicsStepperSystem.setup");
        Self::SystemData::setup(res);

        // initialise required resources
        res.entry::<Physics<N>>().or_insert_with(Physics::default);
    }
}

//TODO: Derive
impl<N> Default for PhysicsStepperSystem<N>
where
    N: RealField,
{
    fn default() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }
}

// TODO: Make this readable
fn entity_from_collision_object_handle<N: RealField>(
    entities: &Entities,
    handle: DefaultColliderHandle,
    colliders: &DefaultColliderSet<N>,
) -> Entity {
    entities.entity(
        *colliders
            .get(handle)
            .unwrap()
            .user_data()
            .unwrap()
            .downcast_ref::<Index>()
            .unwrap(),
    )
}
