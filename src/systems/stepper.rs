use crate::{PhysicsWorld, Real};
use crate::storageb::{PhysicsStorage, BodyStorage};
use specs::System;
use specs::{WriteStorage, WriteExpect, Write};
use std::marker::PhantomData;

#[derive(Default)]
pub struct PhysicsStepper<N>(PhantomData<N>);

impl<'s, N: Real> System<'s> for PhysicsStepper<N> {
    type SystemData = (WriteExpect<'s, PhysicsWorld<N>>,
        Write<'s, BodyStorage<N>>,
        Write<'s, BodyStorage<N>>,
        Write<'s, BodyStorage<N>>,
        Write<'s, BodyStorage<N>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        unimplemented!()
    }
}