use std::marker::PhantomData;

use specs::{Join, System, WriteExpect, WriteStorage};

use crate::{bodies::PhysicsBody, pose::Pose};
use nalgebra::RealField;
use nphysics::object::DefaultBodySet;

/// The `SyncBodiesFromPhysicsSystem` synchronised the updated position of
/// the `RigidBody`s in the nphysics `World` with their Specs counterparts. This
/// affects the `Pose` `Component` related to the `Entity`.
pub struct SyncBodiesFromPhysicsSystem<N, P>(PhantomData<(N, P)>);

impl<'s, N, P> System<'s> for SyncBodiesFromPhysicsSystem<N, P>
where
    N: RealField,
    P: Pose<N>,
{
    type SystemData = (
        WriteStorage<'s, PhysicsBody<N>>,
        WriteExpect<'s, DefaultBodySet<N>>,
        WriteStorage<'s, P>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (mut physics_bodies, bodies, mut positions) = data;

        // iterate over all PhysicBody components joined with their Poses
        for (physics_body, position) in (&mut physics_bodies, &mut positions).join() {
            // if a RigidBody exists in the nphysics World we fetch it and update the
            // Pose component accordingly
            if let Some(rigid_body) = bodies.rigid_body(physics_body.handle.unwrap()) {
                position.set_isometry(*rigid_body.position());
                physics_body.update_from_physics_world(rigid_body);
            }
        }
    }
}

impl<N, P> Default for SyncBodiesFromPhysicsSystem<N, P> {
    fn default() -> Self {
        Self(PhantomData)
    }
}
