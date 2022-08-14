mod physics;
use physics::*;
use rapier2d::prelude::RigidBodyHandle;

use std::time;

use hecs::{ergo::*, Entity};
use macroquad::{prelude::*, rand::gen_range};

struct Transform {
    pos: Vec2,
}

struct Shape {
    rigidbody: RigidBodyHandle,
}

struct Movement {
    velocity: Vec2,
}

#[derive(Copy, Clone)]
struct Explosion {
    pos: Vec2,
    start_time: f64,
}

const WORLD_SIZE: Vec2 = const_vec2!([1000.0, 1000.0]);
const NUM_DUDES: i32 = 1000;
const EXPLODE_TIME: f64 = 1.5;
const MAX_SPEED: f32 = 50.0;

fn handle_collisions(world: &ErgoScope<'_>, collisions: Vec<EntityCollision>) {
    let mut physics = get_physics(world);
    for event in collisions {
        let mut to_spawn = 0;
        let mut pos = Vec2::ZERO;
        if world.contains(event.entity_a) {
            // remove physics body
            let shape = world.get::<Shape>(event.entity_a).unwrap();
            let size = physics.read().ball_size(shape.read().rigidbody);
            to_spawn += (size - 0.5) as i32;
            if to_spawn <= 0 {
                to_spawn = 1;
            }
            pos = physics.read().get_pos(shape.read().rigidbody);
            physics.write().remove(shape.read().rigidbody);
            world.despawn(event.entity_a).unwrap();
        }
        if world.contains(event.entity_b) {
            // remove physics body
            let shape = world.get::<Shape>(event.entity_b).unwrap();
            let size = physics.read().ball_size(shape.read().rigidbody);
            to_spawn += (size - 0.5) as i32;
            if to_spawn <= 0 {
                to_spawn = 1;
            }
            pos = physics.read().get_pos(shape.read().rigidbody);
            physics.write().remove(shape.read().rigidbody);
            world.despawn(event.entity_b).unwrap();
        }
        if to_spawn > 0 {
            world.spawn((Explosion {
                pos,
                start_time: get_time(),
            },));
            physics
                .read()
                .get_entities_within(pos, 500.0, |collider, entity| {
                    if entity != event.entity_a && entity != event.entity_b {
                        push_from_explosion(world, entity, pos);
                    }
                    true
                });

            // spawn new dudes in random directions
            for _ in 0..to_spawn {
                let dir = Vec2::new(gen_range(-1.0, 1.0), gen_range(-1.0, 1.0)).normalize_or_zero();

                spawn_dude(
                    world,
                    &mut *physics.write(),
                    pos + dir * 5.0,
                    dir * gen_range(0.1 * MAX_SPEED, MAX_SPEED),
                    1.0,
                );
            }
        }
    }
}

fn push_from_explosion(world: &ErgoScope, entity: Entity, pos: Vec2) {
    let mut movement = world.get::<Movement>(entity).unwrap();
    let transform = world.get::<Transform>(entity).unwrap();
    let diff = transform.read().pos - pos;
    let distance = diff.length();
    let dir = diff.normalize_or_zero();
    let push_power = 10000.0 / (distance * distance).max(0.01);
    movement.write().velocity += dir * push_power;
    movement.write().velocity.clamp_length_max(MAX_SPEED);
}

fn update_game(world: &ErgoScope<'_>) {
    let mut physics = get_physics(world);
    // sync velocities to physics state and randomize new velocity if outside screen
    for (_, (transform, mut movement, shape)) in
        world.query::<(&Transform, &Movement, &Shape)>().iter()
    {
        if !Rect::new(0.0, 0.0, WORLD_SIZE.x, WORLD_SIZE.y).contains(transform.read().pos) {
            movement.write().velocity = Vec2::new(gen_range(-1.0, 1.0), gen_range(-1.0, 1.0))
                .normalize_or_zero()
                * gen_range(MAX_SPEED * 0.1, MAX_SPEED);
        }
        physics
            .write()
            .set_ball_velocity(shape.read().rigidbody, movement.read().velocity);
    }

    // step physics and handle collisions
    let collisions = physics.write().step();
    // read physics position to transform component
    for (_, (mut transform, shape)) in world.query::<(&Transform, &Shape)>().iter() {
        transform.write().pos = physics.read().get_pos(shape.read().rigidbody);
    }
    // handle collisions
    handle_collisions(world, collisions);

    // despawn finished explosions
    for (e, exploding) in world.query::<(&Explosion)>().iter() {
        if get_time() - exploding.read().start_time > EXPLODE_TIME {
            world.despawn(e).unwrap();
        }
    }
}

fn get_physics(world: &ErgoScope<'_>) -> ComponentRef<PhysicsState> {
    world.query::<&PhysicsState>().iter().next().unwrap().1
}

fn draw_game(world: &ErgoScope<'_>) {
    let projection = macroquad::math::Mat4::orthographic_lh(
        0.0,
        WORLD_SIZE.x * 2.0,
        0.0,
        WORLD_SIZE.y * 2.0,
        0.0,
        1.0,
    ) * Mat4::from_scale(Vec3::new(screen_width(), screen_height(), 1.0));
    clear_background(BLACK);
    // draw dudes
    for (_, (transform, shape)) in world.query::<(&Transform, &Shape)>().iter() {
        let draw_pos = projection.transform_point3(transform.read().pos.extend(0.0));
        let ball_size = get_physics(world).read().ball_size(shape.read().rigidbody);
        draw_circle(draw_pos.x, draw_pos.y, ball_size, RED);
    }
    // draw explosions
    for (_, (explosion,)) in world.query::<(&Explosion,)>().iter() {
        let draw_pos = projection.transform_point3(explosion.read().pos.extend(0.0));
        let t = ((get_time() - explosion.read().start_time) / EXPLODE_TIME) as f32;
        draw_circle_lines(
            draw_pos.x,
            draw_pos.y,
            t * 100.0,
            1.0,
            Color::from_vec(YELLOW.to_vec() * (1.0 - t)),
        );
    }
}

fn init_game() -> World {
    let mut world = World::new();
    {
        let ergo = ErgoScope::new(&mut world);
        let mut physics = PhysicsState::default();
        let x_dudes = (NUM_DUDES as f32).sqrt() as i32;
        for i in 0..NUM_DUDES {
            // spawn dudes in a grid
            let x = (i / x_dudes) as f32 / x_dudes as f32;
            let y = (i % x_dudes) as f32 / x_dudes as f32;
            let size = gen_range(2.0, 5.0);
            let pos = Vec2::new(x as f32 * WORLD_SIZE.x, y * WORLD_SIZE.y);
            let velocity = Vec2::new(gen_range(-1.0, 1.0), gen_range(-1.0, 1.0))
                .normalize_or_zero()
                * gen_range(MAX_SPEED * 0.1, MAX_SPEED);
            spawn_dude(&ergo, &mut physics, pos, velocity, size);
        }
        ergo.spawn((physics,));
    }

    world
}

fn spawn_dude(world: &ErgoScope, physics: &mut PhysicsState, pos: Vec2, velocity: Vec2, size: f32) {
    let entity = world.reserve_entity();
    let rigidbody = physics.add_ball(entity, pos, size);
    world.spawn_at(
        entity,
        (
            Transform { pos },
            Shape { rigidbody },
            Movement { velocity },
        ),
    );
}

#[macroquad::main("cogito")]
async fn main() {
    let mut world = init_game();
    loop {
        let ergo = ErgoScope::new(&mut world);
        update_game(&ergo);
        draw_game(&ergo);

        next_frame().await
    }
}
