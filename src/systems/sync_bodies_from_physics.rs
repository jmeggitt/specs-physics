use std::marker::PhantomData;

use specs::{Join, ReadExpect, Resources, System, SystemData, WriteStorage};

use crate::{bodies::PhysicsBody, positon::Pose, Physics};
use nalgebra::RealField;

/// The `SyncBodiesFromPhysicsSystem` synchronised the updated position of
/// the `RigidBody`s in the nphysics `World` with their Specs counterparts. This
/// affects the `Pose` `Component` related to the `Entity`.
pub struct SyncBodiesFromPhysicsSystem<N, P> {
    n_marker: PhantomData<N>,
    p_marker: PhantomData<P>,
}

impl<'s, N, P> System<'s> for SyncBodiesFromPhysicsSystem<N, P>
where
    N: RealField,
    P: Pose<N>,
{
    type SystemData = (
        ReadExpect<'s, Physics<N>>,
        WriteStorage<'s, PhysicsBody<N>>,
        WriteStorage<'s, P>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (physics, mut physics_bodies, mut positions) = data;

        // iterate over all PhysicBody components joined with their Poses
        for (physics_body, position) in (&mut physics_bodies, &mut positions).join() {
            // if a RigidBody exists in the nphysics World we fetch it and update the
            // Pose component accordingly
            if let Some(rigid_body) = physics.world.rigid_body(physics_body.handle.unwrap()) {
                *position.isometry_mut() = *rigid_body.position();
                physics_body.update_from_physics_world(rigid_body);
            }
        }
    }

    fn setup(&mut self, res: &mut Resources) {
        info!("SyncBodiesFromPhysicsSystem.setup");
        Self::SystemData::setup(res);

        // initialise required resources
        res.entry::<Physics<N>>().or_insert_with(Physics::default);
    }
}

impl<N, P> Default for SyncBodiesFromPhysicsSystem<N, P>
where
    N: RealField,
    P: Pose<N>,
{
    fn default() -> Self {
        Self {
            n_marker: PhantomData,
            p_marker: PhantomData,
        }
    }
}
