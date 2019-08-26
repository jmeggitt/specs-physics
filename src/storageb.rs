use ncollide::pipeline::object::CollisionObjectSet;
use nphysics::object::{Body, BodyPartHandle, BodySet, Collider, ColliderRemovalData, ColliderSet};
use nphysics::joint::{JointConstraint, JointConstraintSet};
use nphysics::force_generator::{ForceGenerator, ForceGeneratorSet};
use specs::storage::DenseVecStorage;
use specs::storage::DistinctStorage;
use specs::storage::MaskedStorage;
use specs::storage::{TryDefault, UnprotectedStorage};
use specs::world::Index;
use specs::Component;
use specs::SystemData;
use specs::storage::WriteStorage;
use super::Real;
use hibitset::{BitSet, BitSetLike};
use nalgebra::RealField;
use std::marker::PhantomData;
use std::any::Any;

//macro_rules! unprotected {
//    ($generic: ident, $type: ty, $content: ty) => {
//        impl<$generic: Real> UnprotectedStorage<$content> for $type {
//            unsafe fn clean<B: BitSetLike>(&mut self, has: B) {
//                self.storage.clean(has)
//            }
//
//            unsafe fn get(&self, id: Index) -> &$content {
//                self.storage.get(id)
//            }
//
//            unsafe fn get_mut(&mut self, id: Index) -> &mut $content {
//                self.storage.get_mut(id)
//            }
//
//            unsafe fn insert(&mut self, id: Index, value: $content) {
//                self.storage.insert(id, value)
//            }
//
//            unsafe fn remove(&mut self, id: Index) -> $content {
//                self.storage.remove(id)
//            }
//        }
//    };
//}

pub struct SpecsWrapper<T>(T);

impl<T> Component for SpecsWrapper<T>
    where PhysicsStorage<Self>: UnprotectedStorage<Self> + Send + Sync + Default,
        Self: Any {
    type Storage = PhysicsStorage<Self>;
}

pub struct PhysicsStorage<T> {
    storage: DenseVecStorage<T>,
    mask: BitSet,
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
    unsafe fn clean<B>(&mut self, has: B) where
        B: BitSetLike {
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


//unprotected! {N, BodyStorage<N>, SpecsWrapper<Box<dyn Body<N>>>}

impl<N: Real> BodySet<N> for BodyStorage<N> {
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

//unprotected! {N, PhysicsStorage<SpecsWrapper<Collider<N, Index>>>, SpecsWrapper<Collider<N, Index>>}

impl<N: Real> CollisionObjectSet<N> for PhysicsStorage<SpecsWrapper<Collider<N, Index>>> {
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

impl<N: Real> ColliderSet<N, Index> for PhysicsStorage<SpecsWrapper<Collider<N, Index>>> {
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

//unprotected!{N, PhysicsStorage<SpecsWrapper<Box<dyn JointConstraint<N, BodyStorage<N>>>>>, SpecsWrapper<Box<dyn JointConstraint<N, BodyStorage<N>>>>}

impl<N: Real> JointConstraintSet<N, BodyStorage<N>> for PhysicsStorage<SpecsWrapper<Box<dyn JointConstraint<N, BodyStorage<N>>>>> {
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

    fn pop_insertion_event(&mut self) -> Option<(Self::Handle, BodyPartHandle<<BodyStorage<N> as BodySet<N>>::Handle>, BodyPartHandle<<BodyStorage<N> as BodySet<N>>::Handle>)> {
        unimplemented!()
    }

    fn pop_removal_event(&mut self) -> Option<(Self::Handle, BodyPartHandle<<BodyStorage<N> as BodySet<N>>::Handle>, BodyPartHandle<<BodyStorage<N> as BodySet<N>>::Handle>)> {
        unimplemented!()
    }

    fn remove(&mut self, to_remove: Self::Handle) {
        unimplemented!()
    }
}

//unprotected!{N, PhysicsStorage<SpecsWrapper<Box<dyn ForceGenerator<N, BodyStorage<N>>>>>, SpecsWrapper<Box<dyn ForceGenerator<N, BodyStorage<N>>>>}

impl<N: Real> ForceGeneratorSet<N, BodyStorage<N>> for PhysicsStorage<SpecsWrapper<Box<dyn ForceGenerator<N, BodyStorage<N>>>>> {
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
