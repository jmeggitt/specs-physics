use hibitset::{BitSet, BitSetLike};
use nalgebra::RealField;
use ncollide::pipeline::object::CollisionObjectSet;
use nphysics::force_generator::{ForceGenerator, ForceGeneratorSet};
use nphysics::joint::{JointConstraint, JointConstraintSet};
use nphysics::object::{Body, BodyPartHandle, BodySet, Collider, ColliderRemovalData, ColliderSet};
use specs::storage::DenseVecStorage;
use specs::storage::UnprotectedStorage;
use specs::world::Index;
use specs::{Component, Join};
use std::any::Any;
use std::ops::{Deref, DerefMut};

pub struct SpecsWrapper<T>(T);

impl<T> Component for SpecsWrapper<T>
where
    PhysicsStorage<Self>: UnprotectedStorage<Self> + Send + Sync + Default,
    Self: Any,
{
    type Storage = PhysicsStorage<Self>;
}

impl<T> Deref for SpecsWrapper<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> DerefMut for SpecsWrapper<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

pub struct PhysicsStorage<T> {
    storage: DenseVecStorage<T>,
    mask: BitSet,
}

impl<'a, T> Join for &'a PhysicsStorage<SpecsWrapper<T>> {
    type Type = &'a T;
    type Value = &'a DenseVecStorage<SpecsWrapper<T>>;
    type Mask = &'a BitSet;

    unsafe fn open(self) -> (Self::Mask, Self::Value) {
        (&self.mask, &self.storage)
    }

    unsafe fn get(value: &mut Self::Value, id: Index) -> Self::Type {
        &value.get(id)
    }
}

impl<'a, T> Join for &'a mut PhysicsStorage<SpecsWrapper<T>> {
    type Type = &'a mut T;
    type Value = &'a mut DenseVecStorage<SpecsWrapper<T>>;
    type Mask = &'a BitSet;

    unsafe fn open(self) -> (Self::Mask, Self::Value) {
        (&self.mask, &mut self.storage)
    }

    unsafe fn get(value: &mut Self::Value, id: Index) -> Self::Type {
        let value: *mut Self::Value = value as *mut Self::Value;
        (*value).get_mut(id)
    }
}

// Derive was creating something with some weird bounds so I wrote it by hand
impl<T> Default for PhysicsStorage<T> {
    fn default() -> Self {
        Self {
            storage: DenseVecStorage::default(),
            mask: BitSet::default(),
        }
    }
}

impl<T> UnprotectedStorage<T> for PhysicsStorage<T> {
    unsafe fn clean<B>(&mut self, has: B)
    where
        B: BitSetLike,
    {
        self.storage.clean(has)
    }

    unsafe fn get(&self, id: u32) -> &T {
        self.storage.get(id)
    }

    unsafe fn get_mut(&mut self, id: u32) -> &mut T {
        self.storage.get_mut(id)
    }

    unsafe fn insert(&mut self, id: u32, value: T) {
        self.storage.insert(id, value)
    }

    unsafe fn remove(&mut self, id: u32) -> T {
        self.storage.remove(id)
    }
}

pub type BodyStorage<N> = PhysicsStorage<SpecsWrapper<Box<dyn Body<N>>>>;

impl<N: RealField> BodySet<N> for BodyStorage<N> {
    type Body = dyn Body<N>;
    type Handle = Index;

    fn get(&self, handle: Self::Handle) -> Option<&Self::Body> {
        unimplemented!()
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::Body> {
        unimplemented!()
    }

    fn get_pair_mut(
        &mut self,
        handle1: Self::Handle,
        handle2: Self::Handle,
    ) -> (Option<&mut Self::Body>, Option<&mut Self::Body>) {
        unimplemented!()
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        unimplemented!()
    }

    fn foreach(&self, f: impl FnMut(Self::Handle, &Self::Body)) {
        unimplemented!()
    }

    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Self::Body)) {
        unimplemented!()
    }

    fn pop_removal_event(&mut self) -> Option<Self::Handle> {
        unimplemented!()
    }
}

impl<N: RealField> CollisionObjectSet<N> for PhysicsStorage<SpecsWrapper<Collider<N, Index>>> {
    type CollisionObject = Collider<N, Index>;
    type CollisionObjectHandle = Index;

    fn collision_object(
        &self,
        handle: Self::CollisionObjectHandle,
    ) -> Option<&Self::CollisionObject> {
        unimplemented!()
    }

    fn foreach(&self, f: impl FnMut(Self::CollisionObjectHandle, &Self::CollisionObject)) {
        unimplemented!()
    }
}

impl<N: RealField> ColliderSet<N, Index> for PhysicsStorage<SpecsWrapper<Collider<N, Index>>> {
    type Handle = Index;

    fn get(&self, handle: Self::Handle) -> Option<&Collider<N, Index>> {
        unimplemented!()
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Collider<N, Index>> {
        unimplemented!()
    }

    fn get_pair_mut(
        &mut self,
        handle1: Self::Handle,
        handle2: Self::Handle,
    ) -> (
        Option<&mut Collider<N, Index>>,
        Option<&mut Collider<N, Index>>,
    ) {
        unimplemented!()
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        unimplemented!()
    }

    fn foreach(&self, f: impl FnMut(Self::Handle, &Collider<N, Index>)) {
        unimplemented!()
    }

    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Collider<N, Index>)) {
        unimplemented!()
    }

    fn pop_insertion_event(&mut self) -> Option<Self::Handle> {
        unimplemented!()
    }

    fn pop_removal_event(&mut self) -> Option<(Self::Handle, ColliderRemovalData<N, Index>)> {
        unimplemented!()
    }

    fn remove(&mut self, to_remove: Self::Handle) -> Option<&mut ColliderRemovalData<N, Index>> {
        unimplemented!()
    }
}

impl<N: RealField> JointConstraintSet<N, BodyStorage<N>>
    for PhysicsStorage<SpecsWrapper<Box<dyn JointConstraint<N, BodyStorage<N>>>>>
{
    type JointConstraint = dyn JointConstraint<N, BodyStorage<N>>;
    type Handle = Index;

    fn get(&self, handle: Self::Handle) -> Option<&Self::JointConstraint> {
        unimplemented!()
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::JointConstraint> {
        unimplemented!()
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        unimplemented!()
    }

    fn foreach(&self, f: impl FnMut(Self::Handle, &Self::JointConstraint)) {
        unimplemented!()
    }

    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Self::JointConstraint)) {
        unimplemented!()
    }

    fn pop_insertion_event(
        &mut self,
    ) -> Option<(
        Self::Handle,
        BodyPartHandle<<BodyStorage<N> as BodySet<N>>::Handle>,
        BodyPartHandle<<BodyStorage<N> as BodySet<N>>::Handle>,
    )> {
        unimplemented!()
    }

    fn pop_removal_event(
        &mut self,
    ) -> Option<(
        Self::Handle,
        BodyPartHandle<<BodyStorage<N> as BodySet<N>>::Handle>,
        BodyPartHandle<<BodyStorage<N> as BodySet<N>>::Handle>,
    )> {
        unimplemented!()
    }

    fn remove(&mut self, to_remove: Self::Handle) {
        unimplemented!()
    }
}

impl<N: RealField> ForceGeneratorSet<N, BodyStorage<N>>
    for PhysicsStorage<SpecsWrapper<Box<dyn ForceGenerator<N, BodyStorage<N>>>>>
{
    type ForceGenerator = dyn ForceGenerator<N, BodyStorage<N>>;
    type Handle = Index;

    fn get(&self, handle: Self::Handle) -> Option<&Self::ForceGenerator> {
        unimplemented!()
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::ForceGenerator> {
        unimplemented!()
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        unimplemented!()
    }

    fn foreach(&self, f: impl FnMut(Self::Handle, &Self::ForceGenerator)) {
        unimplemented!()
    }

    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Self::ForceGenerator)) {
        unimplemented!()
    }
}
