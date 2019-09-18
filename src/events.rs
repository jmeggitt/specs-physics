use specs::{Entities, Entity};

use crate::colliders::collider_handle_to_entity;
use ncollide::pipeline::{ContactEvent as OldContactEvent, ProximityEvent as OldProximityEvent};
use ncollide::query::Proximity;
use nphysics::object::{DefaultColliderHandle, DefaultColliderSet};

use nalgebra::RealField;
use shrev::EventChannel;

/// The `ContactType` is set accordingly to whether a contact began or ended.
#[derive(Debug, Copy, Clone)]
pub enum ContactType {
    /// Event occurring when two collision objects start being in contact.
    Started,
    /// Event occurring when two collision objects stop being in contact.    
    Stopped,
}

/// The `ContactEvent` type contains information about the objects that
/// collided.
#[derive(Debug, Copy, Clone)]
pub struct ContactEvent {
    pub collider1: Entity,
    pub collider2: Entity,

    pub contact_type: ContactType,
}

impl ContactEvent {
    /// Create our own ProximityEvent from the extracted data
    pub(crate) fn from_ncollide<N: RealField>(
        event: OldContactEvent<DefaultColliderHandle>,
        entities: &Entities,
        colliders: &DefaultColliderSet<N>,
    ) -> Option<Self> {
        // retrieve CollisionObjectHandles from ContactEvent and map the ContactEvent
        // type to our own custom ContactType
        let (handle1, handle2, contact_type) = match event {
            OldContactEvent::Started(c1, c2) => (c1, c2, ContactType::Started),
            OldContactEvent::Stopped(c1, c2) => (c1, c2, ContactType::Stopped),
        };

        // create our own ContactEvent from the extracted data; mapping the
        // CollisionObjectHandles to Entities is error prone but should work as intended
        // as long as we're the only ones working directly with the nphysics World
        let result = Self {
            collider1: collider_handle_to_entity(handle1, &entities, &colliders)?,
            collider2: collider_handle_to_entity(handle2, &entities, &colliders)?,
            contact_type,
        };

        Some(result)
    }
}

/// `ContactEvents` is a custom `EventChannel` type used to expose
/// `ContactEvent`s.
pub type ContactEvents = EventChannel<ContactEvent>;

/// The `ProximityEvent` type contains information about the objects that
/// triggered a proximity "collision". These kind of events contain at least one
/// *sensor* `PhysicsCollider`.
#[derive(Debug, Copy, Clone)]
pub struct ProximityEvent {
    pub collider1: Entity,
    pub collider2: Entity,

    pub prev_status: Proximity,
    pub new_status: Proximity,
}

impl ProximityEvent {
    /// Create our own ProximityEvent from the extracted data
    pub(crate) fn from_ncollide<N: RealField>(
        event: OldProximityEvent<DefaultColliderHandle>,
        entities: &Entities,
        colliders: &DefaultColliderSet<N>,
    ) -> Option<Self> {
        let result = Self {
            collider1: collider_handle_to_entity(event.collider1, entities, colliders)?,
            collider2: collider_handle_to_entity(event.collider2, entities, colliders)?,
            prev_status: event.prev_status,
            new_status: event.new_status,
        };

        Some(result)
    }
}

/// `ProximityEvent` is a custom `EventChannel` type used to expose
/// `ProximityEvent`s.
pub type ProximityEvents = EventChannel<ProximityEvent>;
