use nalgebra::RealField;
use specs::{Component, DenseVecStorage, FlaggedStorage};

#[cfg(feature = "physics3d")]
use nalgebra::Matrix3;

use nphysics::{math::{Force, ForceType, Point, Velocity},
               object::{Body, BodyPart, BodyStatus, DefaultBodyHandle, RigidBody, RigidBodyDesc}};

// Note: `nphysics::math::AngularInertia` could have been used in this file, but
// it uses a 1x1 matrix in 2d which is inconvenient.

/// The `PhysicsBody` `Component` represents a `PhysicsWorld` `RigidBody` in
/// Specs and contains all the data required for the synchronisation between
/// both worlds.
#[derive(Clone, Copy, Debug)]
pub struct PhysicsBody<N: RealField> {
    pub(crate) handle: Option<DefaultBodyHandle>,
    pub gravity_enabled: bool,
    pub body_status: BodyStatus,
    pub velocity: Velocity<N>,
    #[cfg(feature = "physics3d")]
    pub angular_inertia: Matrix3<N>,
    #[cfg(feature = "physics2d")]
    pub angular_inertia: N,
    pub mass: N,
    pub local_center_of_mass: Point<N>,
    external_forces: Force<N>,
}

impl<N: RealField> Component for PhysicsBody<N> {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl<N: RealField> PhysicsBody<N> {
    pub fn check_external_force(&self) -> &Force<N> {
        &self.external_forces
    }

    pub fn apply_external_force(&mut self, force: &Force<N>) -> &mut Self {
        self.external_forces += *force;
        self
    }

    /// For creating new rigid body from this component's values
    pub(crate) fn to_rigid_body_desc(&self) -> RigidBodyDesc<N> {
        RigidBodyDesc::new()
            .gravity_enabled(self.gravity_enabled)
            .status(self.body_status)
            .velocity(self.velocity)
            .angular_inertia(self.angular_inertia)
            .mass(self.mass)
            .local_center_of_mass(self.local_center_of_mass)
    }

    /// Note: applies forces by draining external force property
    pub(crate) fn apply_to_physics_world(&mut self, rigid_body: &mut RigidBody<N>) -> &mut Self {
        rigid_body.enable_gravity(self.gravity_enabled);
        rigid_body.set_status(self.body_status);
        rigid_body.set_velocity(self.velocity);
        rigid_body.set_angular_inertia(self.angular_inertia);
        rigid_body.set_mass(self.mass);
        rigid_body.set_local_center_of_mass(self.local_center_of_mass);
        rigid_body.apply_force(0, &self.drain_external_force(), ForceType::Force, true);
        self
    }

    pub(crate) fn update_from_physics_world(&mut self, rigid_body: &RigidBody<N>) -> &mut Self {
        // These two probably won't be modified but hey
        self.gravity_enabled = rigid_body.gravity_enabled();
        self.body_status = rigid_body.status();

        self.velocity = *rigid_body.velocity();

        let local_inertia = rigid_body.local_inertia();
        self.angular_inertia = local_inertia.angular;
        self.mass = local_inertia.linear;
        self
    }

    fn drain_external_force(&mut self) -> Force<N> {
        let value = self.external_forces;
        self.external_forces = Force::<N>::zero();
        value
    }
}

/// The `PhysicsBodyBuilder` implements the builder pattern for `PhysicsBody`s
/// and is the recommended way of instantiating and customising new
/// `PhysicsBody` instances.
///
/// # Example
///
/// ```rust,ignore
/// use specs_physics::{
///     nalgebra::{Matrix3, Point3},
///     nphysics::{algebra::Velocity3, object::BodyStatus},
///     PhysicsBodyBuilder,
/// };
///
/// let physics_body = PhysicsBodyBuilder::from(BodyStatus::Dynamic)
///     .gravity_enabled(true)
///     .velocity(Velocity3::linear(1.0, 1.0, 1.0))
///     .angular_inertia(Matrix3::from_diagonal_element(3.0))
///     .mass(1.3)
///     .local_center_of_mass(Point3::new(0.0, 0.0, 0.0))
///     .build();
/// ```
pub struct PhysicsBodyBuilder<N: RealField> {
    gravity_enabled: bool,
    body_status: BodyStatus,
    velocity: Velocity<N>,
    #[cfg(feature = "physics3d")]
    angular_inertia: Matrix3<N>,
    #[cfg(feature = "physics2d")]
    angular_inertia: N,
    mass: N,
    local_center_of_mass: Point<N>,
}

impl<N: RealField> From<BodyStatus> for PhysicsBodyBuilder<N> {
    /// Creates a new `PhysicsBodyBuilder` from the given `BodyStatus`. This
    /// also populates the `PhysicsBody` with sane defaults.
    fn from(body_status: BodyStatus) -> Self {
        Self {
            gravity_enabled: false,
            body_status,
            velocity: Velocity::zero(),
            #[cfg(feature = "physics3d")]
            angular_inertia: Matrix3::zeros(),
            #[cfg(feature = "physics2d")]
            angular_inertia: N::zero(),
            mass: N::from_f32(1.2).unwrap(),
            local_center_of_mass: Point::origin(),
        }
    }
}

impl<N: RealField> PhysicsBodyBuilder<N> {
    /// Sets the `gravity_enabled` value of the `PhysicsBodyBuilder`.
    pub fn gravity_enabled(mut self, gravity_enabled: bool) -> Self {
        self.gravity_enabled = gravity_enabled;
        self
    }

    // Sets the `velocity` value of the `PhysicsBodyBuilder`.
    pub fn velocity(mut self, velocity: Velocity<N>) -> Self {
        self.velocity = velocity;
        self
    }

    /// Sets the `angular_inertia` value of the `PhysicsBodyBuilder`.
    #[cfg(feature = "physics3d")]
    pub fn angular_inertia(mut self, angular_inertia: Matrix3<N>) -> Self {
        self.angular_inertia = angular_inertia;
        self
    }

    /// Sets the `angular_inertia` value of the `PhysicsBodyBuilder`.
    #[cfg(feature = "physics2d")]
    pub fn angular_inertia(mut self, angular_inertia: N) -> Self {
        self.angular_inertia = angular_inertia;
        self
    }

    /// Sets the `mass` value of the `PhysicsBodyBuilder`.
    pub fn mass(mut self, mass: N) -> Self {
        self.mass = mass;
        self
    }

    /// Sets the `local_center_of_mass` value of the `PhysicsBodyBuilder`.
    pub fn local_center_of_mass(mut self, local_center_of_mass: Point<N>) -> Self {
        self.local_center_of_mass = local_center_of_mass;
        self
    }

    /// Builds the `PhysicsBody` from the values set in the `PhysicsBodyBuilder`
    /// instance.
    pub fn build(self) -> PhysicsBody<N> {
        PhysicsBody {
            handle: None,
            gravity_enabled: self.gravity_enabled,
            body_status: self.body_status,
            velocity: self.velocity,
            angular_inertia: self.angular_inertia,
            mass: self.mass,
            local_center_of_mass: self.local_center_of_mass,
            external_forces: Force::zero(),
        }
    }
}
