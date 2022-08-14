use std::sync::Mutex;

use hecs::{Entity, ErgoScope};
use macroquad::prelude::*;
use rapier2d::{na::Vector2, prelude::*};

pub struct EntityCollision {
    pub entity_a: Entity,
    pub entity_b: Entity,
}

#[derive(Default)]
struct EventHandlerBuffer {
    // why
    buffer: Mutex<Vec<EntityCollision>>,
}

impl EventHandler for EventHandlerBuffer {
    fn handle_collision_event(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        event: CollisionEvent,
        contact_pair: Option<&ContactPair>,
    ) {
        if event.started() {
            let entity_a = Entity::from_bits(
                bodies
                    .get(colliders.get(event.collider1()).unwrap().parent().unwrap())
                    .unwrap()
                    .user_data as u64,
            )
            .unwrap();
            let entity_b = Entity::from_bits(
                bodies
                    .get(colliders.get(event.collider2()).unwrap().parent().unwrap())
                    .unwrap()
                    .user_data as u64,
            )
            .unwrap();
            self.buffer
                .lock()
                .unwrap()
                .push(EntityCollision { entity_a, entity_b });
        }
    }

    fn handle_contact_force_event(
        &self,
        dt: Real,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        panic!()
    }
}

#[derive(Default)]
pub struct PhysicsState {
    /* Create other structures necessary for the simulation. */
    pub gravity: Vec2,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: BroadPhase,
    pub narrow_phase: NarrowPhase,
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
}

impl PhysicsState {
    pub fn step(&mut self) -> Vec<EntityCollision> {
        let event_handler = EventHandlerBuffer::default();
        self.physics_pipeline.step(
            &rapier2d::na::Vector2::new(self.gravity.x, self.gravity.y),
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &(),
            &event_handler,
        );
        self.query_pipeline.update(
            &self.island_manager,
            &self.rigid_body_set,
            &self.collider_set,
        );
        event_handler.buffer.into_inner().unwrap()
    }
    pub fn add_ball(&mut self, entity: Entity, pos: Vec2, size: f32) -> RigidBodyHandle {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![pos.x, pos.y])
            .user_data(entity.to_bits().get() as u128)
            .build();
        let collider = ColliderBuilder::ball(size)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .user_data(entity.to_bits().get() as u128)
            .build();
        let ball_body_handle = self.rigid_body_set.insert(rigid_body);
        self.collider_set
            .insert_with_parent(collider, ball_body_handle, &mut self.rigid_body_set);

        ball_body_handle
    }

    pub fn set_ball_velocity(&mut self, handle: RigidBodyHandle, vel: Vec2) {
        let rigidbody = self.rigid_body_set.get_mut(handle).unwrap();
        rigidbody.set_linvel(vector![vel.x, vel.y], true);
    }

    pub fn ball_size(&self, handle: RigidBodyHandle) -> f32 {
        let rigidbody = self.rigid_body_set.get(handle).unwrap();
        self.collider_set
            .get(rigidbody.colliders()[0])
            .unwrap()
            .shape()
            .as_ball()
            .unwrap()
            .radius
    }

    pub fn get_pos(&self, handle: RigidBodyHandle) -> Vec2 {
        let rigidbody = self.rigid_body_set.get(handle).unwrap();
        Vec2::new(
            rigidbody.position().translation.x,
            rigidbody.position().translation.y,
        )
    }

    pub fn get_entities_within(
        &self,
        pos: Vec2,
        radius: f32,
        cb: impl Fn(ColliderHandle, Entity) -> bool,
    ) {
        self.query_pipeline.intersections_with_shape(
            &self.rigid_body_set,
            &self.collider_set,
            &Isometry::new(vector![pos.x, pos.y], 0.0),
            &Ball::new(radius),
            QueryFilter::new(),
            |collider| {
                cb(
                    collider,
                    Entity::from_bits(self.collider_set.get(collider).unwrap().user_data as u64)
                        .unwrap(),
                )
            },
        );
    }

    pub fn remove(&mut self, rigidbody: RigidBodyHandle) {
        self.rigid_body_set.remove(
            rigidbody,
            &mut self.island_manager,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            true,
        );
    }
}
