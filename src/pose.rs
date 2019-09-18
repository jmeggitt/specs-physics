use nalgebra::RealField;
use specs::{Component, DenseVecStorage, FlaggedStorage};
use std::ops::{Deref, DerefMut};

use nphysics::math::{Isometry, Point};

#[cfg(feature = "physics2d")]
use amethyst_core::math::{Translation2, UnitComplex};

/// A `Pose` is a position and an orientation. They are wrapped together into an
/// isometry for use in the physics engine.
///
/// An implementation of the `Pose` trait is required for the
/// synchronisation of the position of Specs and nphysics objects.
///
/// Initially, it is used to position bodies in the nphysics `World`. Then after
/// progressing the `World` it is used to synchronise the updated positions back
/// towards Specs.
pub trait Pose<N: RealField>:
    Component<Storage = FlaggedStorage<Self, DenseVecStorage<Self>>> + Send + Sync
{
    fn isometry(&self) -> Isometry<N>;
    fn set_isometry(&mut self, isometry: Isometry<N>) -> &mut Self;

    /// Helper function to extract the location of this `Pose`. Using
    /// `Pose::isometry()` is preferable, but can be harder to work with.
    /// The translation of this `Pose` can be set using `Pose::
    /// isometry_mut()`.
    fn translation(&self) -> Point<N> {
        self.isometry().translation.vector.into()
    }

    /// Helper function to extract the rotation of this `Pose`. Using
    /// `Pose::isometry()` is preferable, but can be harder to work with.
    /// The rotation of this `Pose` can be set using `Pose::isometry_mut()`.
    /// This is only available when the `physics2d` feature is enabled.
    #[cfg(feature = "physics2d")]
    fn angle(&self) -> N {
        self.isometry().rotation.angle()
    }
}

#[deprecated(
    since = "0.4.0",
    note = "Position was renamed to Pose to better reflect how it\
            contains an orientation and to make it slightly shorter."
)]
pub trait Position<N: RealField>: Pose<N> {}

#[cfg(feature = "amethyst")]
#[cfg(feature = "physics3d")]
impl Pose<f32> for amethyst_core::Transform {
    fn isometry(&self) -> Isometry<f32> {
        *self.isometry()
    }

    fn set_isometry(&mut self, isometry: Isometry<f32>) -> &mut Self {
        self.set_isometry(isometry)
    }
}

#[cfg(feature = "amethyst")]
#[cfg(feature = "physics2d")]
impl Pose<f32> for amethyst_core::Transform {
    fn isometry(&self) -> Isometry<f32> {
        let iso3 = self.isometry();
        let translation = Translation2::new(iso3.translation.x, iso3.translation.y);
        let angle = iso3.rotation.euler_angles();
        let rotation = UnitComplex::new(angle.2);
        Isometry::from_parts(translation, rotation)
    }

    fn set_isometry(&mut self, isometry: Isometry<f32>) -> &mut Self {
        let z_angle = isometry.rotation.angle();
        let euler_angles = self.rotation().euler_angles();
        self.set_rotation_euler(euler_angles.0, euler_angles.1, z_angle);
        self.set_translation_x(isometry.translation.x);
        self.set_translation_y(isometry.translation.y);
        self
    }
}

pub struct SimplePosition<N: RealField>(pub Isometry<N>);

impl<N: RealField> Pose<N> for SimplePosition<N> {
    fn isometry(&self) -> Isometry<N> {
        self.0
    }

    fn set_isometry(&mut self, isometry: Isometry<N>) -> &mut Self {
        self.0 = isometry;
        self
    }
}

impl<N: RealField> Component for SimplePosition<N> {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl<N: RealField> Deref for SimplePosition<N> {
    type Target = Isometry<N>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<N: RealField> DerefMut for SimplePosition<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
