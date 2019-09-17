use std::marker::PhantomData;

use specs::{Join, Resources, System, SystemData, WriteExpect, WriteStorage};

use crate::{bodies::PhysicsBody, pose::Pose, Physics};
use nalgebra::RealField;
use nphysics::object::DefaultBodySet;

/// The `SyncBodiesFromPhysicsSystem` synchronised the updated position of
/// the `RigidBody`s in the nphysics `World` with their Specs counterparts. This
/// affects the `Pose` `Component` related to the `Entity`.
pub struct SyncBodiesFromPhysicsSystem<N, P> {
    _phantom: PhantomData<(N, P)>,
}

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

    // TODO: Remove
    fn setup(&mut self, res: &mut Resources) {
        info!("SyncBodiesFromPhysicsSystem.setup");
        Self::SystemData::setup(res);

        // initialise required resources
        res.entry::<Physics<N>>().or_insert_with(Physics::default);
    }
}

// TODO: Attempt to derive
impl<N, P> Default for SyncBodiesFromPhysicsSystem<N, P>
where
    N: RealField,
    P: Pose<N>,
{
    fn default() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }
}
