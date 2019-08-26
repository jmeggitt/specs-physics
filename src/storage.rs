use nphysics::object::{BodySet, Body, ColliderSet, Collider, ColliderRemovalData};
use ncollide::pipeline::object::CollisionObjectSet;
use specs::storage::{UnprotectedStorage, TryDefault};
use specs::storage::DenseVecStorage;
use specs::storage::DistinctStorage;
use specs::storage::MaskedStorage;
use specs::world::Index;
use specs::Component;
use crate::wrappers::FlaggedComponent;
use hibitset::{BitSet, BitSetLike};
use nalgebra::RealField;

/// Basically a masked storage
pub struct PhysicsStorage<T> {
    storage: DenseVecStorage<T>,
    mask: BitSet,
}

impl<T> TryDefault for PhysicsStorage<T> {
    fn try_default() -> Result<Self, String> {
        Err(":(".into()) // TODO
    }
}

//impl<T: Component + Default> Default for PhysicsStorage<T> {
//    fn default() -> Self {
//        Self {
//            storage: DenseVecStorage::default(),
//            mask: BitSet::default(),
//        }
//    }
//}

/// Just wrap the functionality of the DenseVecStorage
impl<T: Component> UnprotectedStorage<T> for PhysicsStorage<T> {
    unsafe fn clean<B: BitSetLike>(&mut self, has: B) {
        self.storage.clean(has)
    }

    unsafe fn get(&self, id: Index) -> &T {
        self.storage.get(id)
    }

    unsafe fn get_mut(&mut self, id: Index) -> &mut T {
        self.storage.get_mut(id)
    }

    unsafe fn insert(&mut self, id: Index, value: T) {
        self.storage.insert(id, value)
    }

    unsafe fn remove(&mut self, id: Index) -> T {
        self.storage.remove(id)
    }
}

//unsafe impl <T> DistinctStorage for BodyStorage<T> {}

impl<'a, N: RealField, B: Body<N>> BodySet<N> for PhysicsStorage<FlaggedComponent<B>> {
    type Body = B;
    type Handle = Index;

    fn get(&self, handle: Self::Handle) -> Option<&Self::Body> {
        match self.mask.contains(handle) {
            true => unsafe {Some(self.storage.get(handle))}
            false => None,
        }
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::Body> {
        match self.mask.contains(handle) {
            true => unsafe {Some(self.storage.get_mut(handle))}
            false => None,
        }
    }

    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Self::Body>, Option<&mut Self::Body>) {
        unimplemented!()
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.mask.contains(handle)
    }

    fn foreach(&self, f: impl FnMut<(Self::Handle, &Self::Body)>) {
        for index in self.mask.iter() {
            unsafe { f(index, self.storage.get(index)); }
        }
    }

    fn foreach_mut(&mut self, f: impl FnMut<(Self::Handle, &'a mut Self::Body)>) {
        for index in self.mask.iter() {
            unsafe { f(index, self.storage.get_mut(index)); }
        }
    }

    fn pop_removal_event(&mut self) -> Option<Self::Handle> {
        unimplemented!()
    }
}

impl<'a, N: RealField, B: Body<N>> ColliderSet<N, Index> for PhysicsStorage<FlaggedComponent<B>> {
    type Handle = Index;

    fn get(&self, handle: Self::Handle) -> Option<&Collider<N, Index>> {
        unimplemented!()
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Collider<N, Index>> {
        unimplemented!()
    }

    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Collider<N, Index>>, Option<&mut Collider<N, Index>>) {
        unimplemented!()
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        unimplemented!()
    }

    fn foreach(&self, f: impl FnMut<(Self::Handle, &'a Collider<N, Index>)>) {
        unimplemented!()
    }

    fn foreach_mut(&mut self, f: impl FnMut<(Self::Handle, &'a mut Collider<N, Index>)>) {
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

impl<'a, N: RealField, B: Body<N>> CollisionObjectSet<N> for PhysicsStorage<FlaggedComponent<B>> {
    type CollisionObject = Collider<N, Index>;
    type CollisionObjectHandle = <Self as ColliderSet<N, Index>>::Handle;

    fn collision_object(&self, handle: Self::CollisionObjectHandle) -> Option<&Self::CollisionObject> {
        unimplemented!()
    }

    fn foreach(&self, f: impl FnMut<(Self::CollisionObjectHandle, &'a Self::CollisionObject)>) {
        unimplemented!()
    }
}


