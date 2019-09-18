use std::marker::PhantomData;

use specs::{storage::ComponentEvent, world::Index, Join, ReadStorage, ReaderId, Resources, System,
            SystemData, WriteExpect, WriteStorage};

use crate::{colliders::PhysicsCollider, pose::Pose, Physics, PhysicsParent, PhysicsWorld};
use nalgebra::RealField;
use nphysics::object::{BodyPartHandle, ColliderDesc, DefaultColliderSet};

use super::iterate_component_events;

/// The `SyncCollidersToPhysicsSystem` handles the synchronisation of
/// `PhysicsCollider` `Component`s into the physics `World`.
pub struct SyncCollidersToPhysicsSystem<N, P> {
    positions_reader_id: Option<ReaderId<ComponentEvent>>,
    physics_colliders_reader_id: Option<ReaderId<ComponentEvent>>,
    _phantom: PhantomData<(N, P)>,
}

impl<'s, N, P> System<'s> for SyncCollidersToPhysicsSystem<N, P>
where
    N: RealField,
    P: Pose<N>,
{
    type SystemData = (
        ReadStorage<'s, P>,
        ReadStorage<'s, PhysicsParent>,
        PhysicsWorld<'s, N>,
        WriteExpect<'s, DefaultColliderSet<N>>,
        WriteStorage<'s, PhysicsCollider<N>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (positions, parent_entities, mut physics, mut colliders, mut physics_colliders) = data;

        // collect all ComponentEvents for the Pose storage
        let (inserted_positions, ..) =
            iterate_component_events(&positions, self.positions_reader_id.as_mut().unwrap());

        // collect all ComponentEvents for the PhysicsCollider storage
        let (inserted_physics_colliders, modified_physics_colliders, removed_physics_colliders) =
            iterate_component_events(
                &physics_colliders,
                self.physics_colliders_reader_id.as_mut().unwrap(),
            );

        // iterate over PhysicsCollider and Pose components with an id/Index that
        // exists in either of the collected ComponentEvent BitSets
        for (position, parent_entity, mut physics_collider, id) in (
            &positions,
            parent_entities.maybe(),
            &mut physics_colliders.restrict_mut(),
            &inserted_positions
                | &inserted_physics_colliders
                | &modified_physics_colliders
                | &removed_physics_colliders,
        )
            .join()
        {
            // handle inserted events
            if inserted_positions.contains(id) || inserted_physics_colliders.contains(id) {
                debug!("Inserted PhysicsCollider with id: {}", id);
                add_collider::<N, P>(
                    id,
                    parent_entity,
                    &position,
                    &mut physics,
                    physics_collider.get_mut_unchecked(),
                    &mut *colliders,
                );
            }

            // handle modified events
            if modified_physics_colliders.contains(id) {
                let physics_collider = physics_collider.get_unchecked();
                let collider_handle = physics_collider.handle.unwrap();

                colliders
                    .get_mut(collider_handle)
                    .unwrap()
                    .set_collision_groups(physics_collider.collision_groups);
                debug!(
                    "Updated collider with id {:?} with values: {:?}",
                    id, physics_collider
                );
            }

            // handle removed events
            if removed_physics_colliders.contains(id) {
                debug!("Removed PhysicsCollider with id: {}", id);
                if let Some(handle) = physics.collider_handles.remove(&id) {
                    // we have to check if the collider still exists in the nphysics World before
                    // attempting to delete it as removing a collider that does not exist anymore
                    // causes the nphysics World to panic; colliders are implicitly removed when a
                    // parent body is removed so this is actually a valid scenario
                    if colliders.get(handle).is_some() {
                        colliders.remove(handle);
                    }
                }
            }
        }

        // Drain update triggers caused by inserts
        let event_iter = physics_colliders
            .channel()
            .read(self.physics_colliders_reader_id.as_mut().unwrap());
        for _ in event_iter {}
    }

    fn setup(&mut self, res: &mut Resources) {
        Self::SystemData::setup(res);

        // register reader id for the Pose storage
        let mut position_storage: WriteStorage<P> = SystemData::fetch(&res);
        self.positions_reader_id = Some(position_storage.register_reader());

        // register reader id for the PhysicsBody storage
        let mut physics_collider_storage: WriteStorage<PhysicsCollider<N>> =
            SystemData::fetch(&res);
        self.physics_colliders_reader_id = Some(physics_collider_storage.register_reader());
    }
}

impl<N, P> Default for SyncCollidersToPhysicsSystem<N, P> {
    fn default() -> Self {
        Self {
            positions_reader_id: None,
            physics_colliders_reader_id: None,
            _phantom: PhantomData,
        }
    }
}

fn add_collider<N, P>(
    id: Index,
    parent_entity: Option<&PhysicsParent>,
    position: &P,
    physics: &mut Physics<N>,
    physics_collider: &mut PhysicsCollider<N>,
    colliders: &mut DefaultColliderSet<N>,
) where
    N: RealField,
    P: Pose<N>,
{
    // remove already existing colliders for this inserted event
    if let Some(handle) = physics.collider_handles.remove(&id) {
        warn!("Removing orphaned collider handle: {:?}", handle);
        colliders.remove(handle);
    }

    // Don't allow mis-matched colliders and bodies
    let parent_part_handle = match (physics.body_handles.get(&id), parent_entity) {
        (Some(x), _) => x,
        (_, Some(x)) if physics.body_handles.contains_key(&x.entity.id()) => {
            physics.body_handles.get(&x.entity.id()).unwrap()
        }
        _ => {
            error!("Attempted to add a collider to nonexistent body! Skipping...");
            return;
        }
    };

    // Create the Collider and fetch its handle. We know the body part handle will
    // always have index 0 due to ecs requirement.
    let mut collider = ColliderDesc::new(physics_collider.shape_handle())
        .position(position.isometry() * physics_collider.offset_from_parent)
        .density(physics_collider.density)
        .margin(physics_collider.margin)
        .collision_groups(physics_collider.collision_groups)
        .linear_prediction(physics_collider.linear_prediction)
        .angular_prediction(physics_collider.angular_prediction)
        .sensor(physics_collider.sensor)
        .user_data(id);

    if let Some(material) = &physics_collider.material {
        collider = collider.material(material.clone());
    }

    let collider = collider.build(BodyPartHandle(*parent_part_handle, 0));

    let handle = colliders.insert(collider);

    physics_collider.handle = Some(handle);
    physics.collider_handles.insert(id, handle);

    info!(
        "Inserted collider to world with values: {:?}",
        physics_collider
    );
}

#[cfg(all(test, feature = "physics3d"))]
mod tests {
    use specs::{world::Builder, DispatcherBuilder, World};

    use crate::{colliders::Shape, systems::SyncCollidersToPhysicsSystem, Physics,
                PhysicsColliderBuilder, SimplePosition};
    use nalgebra::Isometry3;

    #[test]
    fn add_collider() {
        let mut world = World::new();
        let mut dispatcher = DispatcherBuilder::new()
            .with(
                SyncCollidersToPhysicsSystem::<f32, SimplePosition<f32>>::default(),
                "sync_colliders_to_physics_system",
                &[],
            )
            .build();
        dispatcher.setup(&mut world.res);

        // create an Entity with the PhysicsCollider component and execute the
        // dispatcher
        world
            .create_entity()
            .with(SimplePosition::<f32>(Isometry3::<f32>::translation(
                1.0, 1.0, 1.0,
            )))
            .with(PhysicsColliderBuilder::<f32>::from(Shape::Ball { radius: 5.0 }).build())
            .build();
        dispatcher.dispatch(&mut world.res);

        // fetch the Physics instance and check for new colliders
        let physics = world.read_resource::<Physics<f32>>();
        assert_eq!(physics.collider_handles.len(), 1);
        assert_eq!(physics.world.colliders().count(), 1);
    }
}
