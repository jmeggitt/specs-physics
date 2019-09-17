use std::marker::PhantomData;

use specs::{storage::ComponentEvent, world::Index, BitSet, Join, ReadStorage, ReaderId, Resources,
            System, SystemData, WriteExpect, WriteStorage};

use crate::{bodies::PhysicsBody, pose::Pose, Physics, PhysicsWorld};
use nalgebra::RealField;
use nphysics::object::DefaultBodySet;

use super::iterate_component_events;

/// The `SyncBodiesToPhysicsSystem` handles the synchronisation of `PhysicsBody`
/// `Component`s into the physics `World`.
pub struct SyncBodiesToPhysicsSystem<N, P> {
    positions_reader_id: Option<ReaderId<ComponentEvent>>,
    physics_bodies_reader_id: Option<ReaderId<ComponentEvent>>,
    _phantom: PhantomData<(N, P)>,
}

impl<'s, N, P> System<'s> for SyncBodiesToPhysicsSystem<N, P>
where
    N: RealField,
    P: Pose<N>,
{
    type SystemData = (
        ReadStorage<'s, P>,
        PhysicsWorld<'s, N>,
        WriteExpect<'s, DefaultBodySet<N>>,
        WriteStorage<'s, PhysicsBody<N>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (positions, mut physics, mut bodies, mut physics_bodies) = data;

        // collect all ComponentEvents for the Pose storage
        let (inserted_positions, modified_positions, removed_positions) =
            iterate_component_events(&positions, self.positions_reader_id.as_mut().unwrap());

        // collect all ComponentEvents for the PhysicsBody storage
        let (inserted_physics_bodies, modified_physics_bodies, removed_physics_bodies) =
            iterate_component_events(
                &physics_bodies,
                self.physics_bodies_reader_id.as_mut().unwrap(),
            );

        // iterate over PhysicsBody and Pose components with an id/Index that
        // exists in either of the collected ComponentEvent BitSets
        for (position, mut physics_body, id) in (
            &positions,
            &mut physics_bodies,
            &inserted_positions
                | &modified_positions
                | &removed_positions
                | &inserted_physics_bodies
                | &modified_physics_bodies
                | &removed_physics_bodies,
        )
            .join()
        {
            // handle inserted events
            if inserted_positions.contains(id) || inserted_physics_bodies.contains(id) {
                debug!("Inserted PhysicsBody with id: {}", id);
                add_rigid_body::<N, P>(
                    id,
                    &position,
                    &mut physics,
                    &mut physics_body,
                    &mut *bodies,
                );
            }

            // handle modified events
            if modified_positions.contains(id) || modified_physics_bodies.contains(id) {
                debug!("Modified PhysicsBody with id: {}", id);
                update_rigid_body::<N, P>(
                    id,
                    &position,
                    &mut physics_body,
                    &mut *bodies,
                    &modified_positions,
                    &modified_physics_bodies,
                );
            }

            // handle removed events
            if removed_positions.contains(id) || removed_physics_bodies.contains(id) {
                debug!("Removed PhysicsBody with id: {}", id);
                remove_rigid_body::<N, P>(id, &mut physics, &mut *bodies);
            }
        }
    }

    fn setup(&mut self, res: &mut Resources) {
        Self::SystemData::setup(res);

        // register reader id for the Pose storage
        let mut position_storage: WriteStorage<P> = SystemData::fetch(&res);
        self.positions_reader_id = Some(position_storage.register_reader());

        // register reader id for the PhysicsBody storage
        let mut physics_body_storage: WriteStorage<PhysicsBody<N>> = SystemData::fetch(&res);
        self.physics_bodies_reader_id = Some(physics_body_storage.register_reader());
    }
}

impl<N, P> Default for SyncBodiesToPhysicsSystem<N, P> {
    fn default() -> Self {
        Self {
            positions_reader_id: None,
            physics_bodies_reader_id: None,
            _phantom: PhantomData,
        }
    }
}

fn add_rigid_body<N, P>(
    id: Index,
    position: &P,
    physics: &mut Physics<N>,
    physics_body: &mut PhysicsBody<N>,
    bodies: &mut DefaultBodySet<N>,
) where
    N: RealField,
    P: Pose<N>,
{
    // remove already existing bodies for this inserted component;
    // this technically should never happen but we need to keep the list of body
    // handles clean
    if let Some(body_handle) = physics.body_handles.remove(&id) {
        warn!("Removing orphaned body handle: {:?}", body_handle);
        bodies.remove(body_handle);
    }

    // create a new RigidBody in the PhysicsWorld and store its
    // handle for later usage
    let body = physics_body
        .to_rigid_body_desc()
        .position(position.isometry())
        .user_data(id)
        .build();
    let handle = bodies.insert(body);

    physics_body.handle = Some(handle);
    physics.body_handles.insert(id, handle);

    info!(
        "Inserted rigid body to world with values: {:?}",
        physics_body
    );
}

fn update_rigid_body<N, P>(
    id: Index,
    position: &P,
    physics_body: &mut PhysicsBody<N>,
    bodies: &mut DefaultBodySet<N>,
    modified_positions: &BitSet,
    modified_physics_bodies: &BitSet,
) where
    N: RealField,
    P: Pose<N>,
{
    if let Some(rigid_body) = bodies.rigid_body_mut(physics_body.handle.unwrap()) {
        // the PhysicsBody was modified, update everything but the position
        if modified_physics_bodies.contains(id) {
            physics_body.apply_to_physics_world(rigid_body);
        }

        // the Pose was modified, update the position directly
        if modified_positions.contains(id) {
            rigid_body.set_position(position.isometry());
        }

        trace!(
            "Updated rigid body in world with values: {:?}",
            physics_body
        );
    }
}

fn remove_rigid_body<N, P>(id: Index, physics: &mut Physics<N>, bodies: &mut DefaultBodySet<N>)
where
    N: RealField,
    P: Pose<N>,
{
    if let Some(handle) = physics.body_handles.remove(&id) {
        // remove body if it still exists in the PhysicsWorld
        bodies.remove(handle);
        info!("Removed rigid body from world with id: {}", id);
    }
}

#[cfg(all(test, feature = "physics3d"))]
mod tests {
    use crate::{systems::SyncBodiesToPhysicsSystem, Physics, PhysicsBodyBuilder, SimplePosition};
    use nalgebra::Isometry3;
    use nphysics::object::BodyStatus;

    use specs::{world::Builder, DispatcherBuilder, World};

    #[test]
    fn add_rigid_body() {
        let mut world = World::new();
        let mut dispatcher = DispatcherBuilder::new()
            .with(
                SyncBodiesToPhysicsSystem::<f32, SimplePosition<f32>>::default(),
                "sync_bodies_to_physics_system",
                &[],
            )
            .build();
        dispatcher.setup(&mut world.res);

        // create an Entity with the PhysicsBody component and execute the dispatcher
        world
            .create_entity()
            .with(SimplePosition::<f32>(Isometry3::<f32>::translation(
                1.0, 1.0, 1.0,
            )))
            .with(PhysicsBodyBuilder::<f32>::from(BodyStatus::Dynamic).build())
            .build();
        dispatcher.dispatch(&mut world.res);

        // fetch the Physics instance and check for new bodies
        let physics = world.read_resource::<Physics<f32>>();
        assert_eq!(physics.body_handles.len(), 1);
        assert_eq!(physics.world.bodies().count(), 1);
    }
}
