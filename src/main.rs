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
struct Exploding {
    start_time: f64,
}

const WORLD_SIZE: Vec2 = const_vec2!([1000.0, 1000.0]);
const NUM_DUDES: i32 = 3000;
const EXPLODE_TIME: f64 = 1.5;
const MAX_SPEED: f32 = 50.0;

fn handle_collisions(world: &ErgoScope<'_>, collisions: Vec<EntityCollision>) {
    let mut physics = get_physics(world);
    for event in collisions {
        let explode = Exploding {
            start_time: get_time(),
        };
        if world.contains(event.entity_a) && world.get::<Exploding>(event.entity_a).is_err() {
            // remove physics body
            let shape = world.get::<Shape>(event.entity_a).unwrap();
            physics.write().remove(shape.read().rigidbody);
            world.remove::<(Shape,)>(event.entity_a).unwrap();
            // add the Exploding component
            world.insert(event.entity_a, (explode,)).unwrap();
        }
        if world.contains(event.entity_b) && world.get::<Exploding>(event.entity_b).is_err() {
            // remove physics body
            let shape = world.get::<Shape>(event.entity_b).unwrap();
            physics.write().remove(shape.read().rigidbody);
            world.remove::<(Shape,)>(event.entity_b).unwrap();
            // add the Exploding component
            world.insert(event.entity_b, (explode,)).unwrap();
        }
    }
}

async fn update_game(world: &ErgoScope<'_>) {
    let mut physics = get_physics(world);
    // sync velocities to physics state and randomize new velocity if outside screen
    for (_, (transform, mut movement, shape)) in world
        .query::<(&Transform, &Movement, &Shape)>()
        .without::<&Exploding>()
        .iter()
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
    handle_collisions(world, collisions);

    // despawn finished explosions
    for (e, exploding) in world.query::<(&Exploding)>().iter() {
        if get_time() - exploding.read().start_time > EXPLODE_TIME {
            world.despawn(e).unwrap();
        }
    }

    // read physics position to transform component
    for (_, (mut transform, shape)) in world
        .query::<(&Transform, &Shape)>()
        .without::<&Exploding>()
        .iter()
    {
        transform.write().pos = physics.read().get_pos(shape.read().rigidbody);
    }
}

fn get_physics(world: &ErgoScope<'_>) -> ComponentRef<PhysicsState> {
    world.query::<&PhysicsState>().iter().next().unwrap().1
}

async fn draw_game(world: &ErgoScope<'_>) {
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
    for (_, (transform, shape)) in world
        .query::<(&Transform, &Shape)>()
        .without::<&Exploding>()
        .iter()
    {
        let draw_pos = projection.transform_point3(transform.read().pos.extend(0.0));
        let ball_size = get_physics(world).read().ball_size(shape.read().rigidbody);
        draw_circle(draw_pos.x, draw_pos.y, ball_size, RED);
    }
    // draw explosions
    for (_, (transform, exploding)) in world.query::<(&Transform, &Exploding)>().iter() {
        let draw_pos = projection.transform_point3(transform.read().pos.extend(0.0));
        let t = ((get_time() - exploding.read().start_time) / EXPLODE_TIME) as f32;
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
    let mut physics = PhysicsState::default();
    let x_dudes = (NUM_DUDES as f32).sqrt() as i32;
    for i in 0..NUM_DUDES {
        // spawn dudes in a grid
        let x = (i / x_dudes) as f32 / x_dudes as f32;
        let y = (i % x_dudes) as f32 / x_dudes as f32;
        let size = gen_range(2.0, 5.0);
        let pos = Vec2::new(x as f32 * WORLD_SIZE.x, y * WORLD_SIZE.y);
        let entity = world.reserve_entity();
        let rigidbody = physics.add_ball(entity, pos, size);

        world.spawn_at(
            entity,
            (
                Transform { pos },
                Shape { rigidbody },
                Movement {
                    velocity: Vec2::new(gen_range(-1.0, 1.0), gen_range(-1.0, 1.0))
                        .normalize_or_zero()
                        * gen_range(MAX_SPEED * 0.1, MAX_SPEED),
                },
            ),
        );
    }
    world.spawn((physics,));

    world
}

#[macroquad::main("BasicShapes")]
async fn main() {
    let mut world = init_game();
    loop {
        let ergo = ErgoScope::new(&mut world);
        update_game(&ergo).await;
        draw_game(&ergo).await;

        next_frame().await
    }
}
