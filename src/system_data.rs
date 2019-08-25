use specs::{WriteStorage, WriteExpect, ReadStorage, SystemData, Join, Resources, Component, BitSet};
use specs::storage::UnprotectedStorage;
use specs::Entity;
use specs::shred::ResourceId;
use specs::storage::{MaskedStorage, AnyStorage, FlaggedStorage, DenseVecStorage};
use specs::shred::MetaTable;
use specs::storage::TryDefault;

use nphysics::object::BodyHandle;
use nphysics::object::RigidBody;
use nalgebra::RealField;
use crate::Physics;

pub struct HandleWrapper (BodyHandle);

impl Component for HandleWrapper {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

pub struct PhysicsBodies<'a, N: RealField> {
    physics_world: WriteExpect<'a, Physics<N>>,
    handles: WriteStorage<'a, HandleWrapper>,
}

impl<'s, N: RealField> SystemData<'s> for PhysicsBodies<'s, N> {
    fn setup(res: &mut Resources) {
        WriteExpect::<Physics<N>>::setup(res);
        WriteStorage::<HandleWrapper>::setup(res);
    }

    fn fetch(res: &'s Resources) -> Self {
        PhysicsBodies {
            physics_world: WriteExpect::fetch(res),
            handles: WriteStorage::fetch(res),
        }
    }

    fn reads() -> Vec<ResourceId> {
        let mut reads = WriteExpect::<Physics<N>>::reads();
        reads.extend(WriteStorage::<HandleWrapper>::reads());
        reads
    }

    fn writes() -> Vec<ResourceId> {
        let mut reads = WriteExpect::<Physics<N>>::writes();
        reads.extend(WriteStorage::<HandleWrapper>::writes());
        reads
    }
}

impl<'a, 'e, N: RealField> Join for &'a PhysicsBodies<'e, N> {
    type Type = &'a RigidBody<N>;
    type Value = (&'a WriteExpect<'a, Physics<N>>, &'a <HandleWrapper as Component>::Storage);
    type Mask = <&'a WriteStorage<'e, HandleWrapper> as Join>::Mask;

    unsafe fn open(self) -> (Self::Mask, Self::Value) {
        let (handle_masks, handle_storage) = self.handles.open();
        (handle_masks, (&self.physics_world, handle_storage))
    }

    unsafe fn get(value: &mut Self::Value, id: u32) -> Self::Type {
        let handle_inner = value.1.get(id).0;
        value.0.world.rigid_body(handle_inner).unwrap()
    }
}

//#[cfg(feature = "parallel")]
//unsafe impl<'a, 'e, T, D> ParJoin for &'a Storage<'e, T, D>
//    where
//        T: Component,
//        D: Deref<Target = MaskedStorage<T>>,
//        T::Storage: Sync,
//{}

//impl<'a, 'e, N: RealField> Join for &'a mut PhysicsBodies<'e, N> {
//    type Type = &'a mut RigidBody<N>;
//    type Value = (&'a mut WriteExpect<'a, Physics<N>>, &'a mut <HandleWrapper as Component>::Storage);
//    type Mask = <&'a WriteStorage<'e, HandleWrapper> as Join>::Mask;
//
//    unsafe fn open(self) -> (Self::Mask, Self::Value) {
//        let (handle_masks, handle_storage) = (&mut self.handles).open();
//        (handle_masks, (&mut self.physics_world, handle_storage))
//    }
//
//    unsafe fn get(value: &mut Self::Value, id: u32) -> Self::Type {
//        let handle_inner = value.1.get(id).0;
//        value.0.world.rigid_body_mut(handle_inner).unwrap()
//    }
//}



