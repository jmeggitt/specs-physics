use specs::Component;
use specs::storage::{FlaggedStorage, DenseVecStorage, UnprotectedStorage};
use std::ops::{Deref, DerefMut};
use crate::storage::PhysicsStorage;

/// Helper struct to bridge the gap between `nphysics::Body` and `specs::Component`.
pub struct FlaggedComponent<T>(T);

impl<T> FlaggedComponent<T> {
    pub fn unwrap(self) -> T {
        self.0
    }
}

impl<T: 'static> Component for FlaggedComponent<T> where PhysicsStorage<Self>: UnprotectedStorage<Self> + Send + Sync {
    type Storage = PhysicsStorage<Self>;
}

impl<T> Deref for FlaggedComponent<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> DerefMut for FlaggedComponent<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T> From<T> for FlaggedComponent<T> {
    fn from(inner: T) -> Self {
        Self(inner)
    }
}

impl<T: Default> Default for FlaggedComponent<T> {
    fn default() -> Self {
        Self(Default::default())
    }
}