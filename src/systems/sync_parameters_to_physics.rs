use std::marker::PhantomData;

use specs::{Read, System};

use crate::parameters::{Gravity, PhysicsIntegrationParameters, PhysicsProfilingEnabled};
use crate::PhysicsWorld;
use nalgebra::RealField;

/// The `SyncParametersToPhysicsSystem` synchronises the simulation parameters
/// with the nphysics `World`.
pub struct SyncParametersToPhysicsSystem<N>(PhantomData<N>);

impl<'s, N: RealField> System<'s> for SyncParametersToPhysicsSystem<N> {
    type SystemData = (
        Option<Read<'s, Gravity<N>>>,
        Option<Read<'s, PhysicsProfilingEnabled>>,
        Option<Read<'s, PhysicsIntegrationParameters<N>>>,
        PhysicsWorld<'s, N>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (gravity, profiling, integration_params, mut physics) = data;

        // if a Gravity resource exists, synchronise its values with the nphysics World
        if let Some(gravity) = gravity {
            if gravity.0 != *physics.gravity() {
                info!(
                    "Global physics gravity modified from {}, updating to {}.",
                    physics.gravity(),
                    gravity.0
                );
                physics.mechanical_world.gravity = gravity.0;
            }
        }

        if let Some(enable_profiling) = profiling {
            if enable_profiling.0 != physics.performance_counters().enabled() {
                info!(
                    "Physics performance counters.enabled={:}",
                    enable_profiling.0
                );
                physics.mechanical_world.counters.enabled = enable_profiling.0;
            }
        }

        if let Some(params) = integration_params {
            if *params != *physics.integration_parameters() {
                params.apply(&mut physics.mechanical_world.integration_parameters);
                info!("Integration parameters have been updated.");
            }
        }
    }
}

impl<N> Default for SyncParametersToPhysicsSystem<N> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

#[cfg(all(test, feature = "physics3d"))]
mod tests {
    use specs::{DispatcherBuilder, World};

    use crate::{parameters::Gravity, systems::SyncParametersToPhysicsSystem, Physics};
    use nalgebra::Vector3;

    #[test]
    fn update_gravity() {
        let mut world = World::new();
        let mut dispatcher = DispatcherBuilder::new()
            .with(
                SyncParametersToPhysicsSystem::<f32>::default(),
                "sync_parameters_to_physics_system",
                &[],
            )
            .build();
        dispatcher.setup(&mut world.res);

        world.add_resource(Gravity(Vector3::<f32>::new(1.0, 2.0, 3.0).into()));
        dispatcher.dispatch(&mut world.res);

        let physics = world.read_resource::<Physics<f32>>();
        assert_eq!(physics.world.gravity().x, 1.0);
        assert_eq!(physics.world.gravity().y, 2.0);
        assert_eq!(physics.world.gravity().z, 3.0);
    }
}
