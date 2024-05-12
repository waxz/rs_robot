use nx_common::common::signal_handler::SignalHandler;
use nx_common::common::thread::Thread;
use nx_message_center::base::{common_message, message_handler, tiny_alloc};

use std::f32::consts::{PI, TAU};
use std::ffi::c_void;
use std::ops::Deref;
use std::slice;
use time::{OffsetDateTime, UtcOffset};

use rand::{thread_rng, Rng};
use std::sync::OnceLock;
use std::sync::{Arc, Mutex, MutexGuard};
// argument
use clap::Parser;

const TA_BUFFER_SIZE: usize = 100000000;
static mut TA_BUFFER: [u8; TA_BUFFER_SIZE] = [0; TA_BUFFER_SIZE];

// gui
use bevy::pbr::CascadeShadowConfigBuilder;
use bevy::prelude::*;
use bevy::render::render_resource::ShaderType;
use bevy::sprite::{MaterialMesh2dBundle, Mesh2dHandle};
use bevy::{
    input::{
        mouse::{MouseButtonInput, MouseMotion, MouseWheel},
        touchpad::{TouchpadMagnify, TouchpadRotate},
    },
    pbr::{MaterialPipeline, MaterialPipelineKey},
    prelude::*,
    reflect::TypePath,
    render::{
        mesh::{MeshVertexBufferLayout, PrimitiveTopology},
        render_asset::RenderAssetUsages,
        render_resource::{
            AsBindGroup, PolygonMode, RenderPipelineDescriptor, ShaderRef,
            SpecializedMeshPipelineError,
        },
    },
};
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin, TouchControls};
use itertools::izip;
use nx_common::common::types::UnsafeSender;
use nx_message_center::base::common_message::shared::PointCloud2;
use nx_message_center::base::message_handler::MessageHandler;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args
{
    #[arg(
        short,
        long,
        default_value = "/home/waxz/CLionProjects/libroscpp/cmake-build-release-host/bin/dds_config.toml"
    )]
    dds_config: String,
}

static GlobalData_BUFFER: OnceLock<Arc<Mutex<(UnsafeSender<*const f32>, usize)>>> = OnceLock::new();
static THE_THING: OnceLock<i32> = OnceLock::new();
static HEIGHT: usize = 480;
static WIDTH: usize = 640;

fn main()
{
    let args = Args::parse();

    let tz_offset = UtcOffset::current_local_offset().expect("should get local offset!");

    println!("hello ros");
    let memory_pool_base = nx_common::common::memory::get_next_aligned_addr(
        unsafe { TA_BUFFER.as_ptr() } as *mut _,
        8,
    );

    let allocator = tiny_alloc::TinyAlloc::new(memory_pool_base, TA_BUFFER_SIZE - 8, 512, 16, 8);
    {
        let allocator = allocator.clone();
        common_message::shared::DefaultAllocatorLock::ALLOCATOR.with(|x| {
            x.get_or_init(move || allocator);
            println!(
                "set nx_message_center::base::ros_message::shared::DefaultAllocator::ALLOCATOR"
            );
        });
    }

    let mut float_vec = allocator.alloc_as_array::<f32>(100);

    {
        let float_vec_ptr = float_vec.as_ptr();
        let float_vec_len = float_vec.len();
        THE_THING.get_or_init(|| 1);
        GlobalData_BUFFER.get_or_init(move || {
            println!("set crate::GlobalData::BUFFER");
            Arc::new(Mutex::new((
                UnsafeSender::new(float_vec_ptr),
                float_vec_len,
            )))
        });

        // GlobalData_BUFFER.get_or_init(move || Arc::new(Mutex::new((    UnsafeSender::new(float_vec_ptr) , float_vec_len))) );
        println!("set crate::GlobalData::BUFFER");
    }

    let mut dds_handler: message_handler::MessageHandler =
        message_handler::MessageHandler::new("dds");
    dds_handler.create(args.dds_config.as_str(), *allocator.cfg.get());

    let mut signal = SignalHandler::default();

    let mut app = App::new();
    app.add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_plugins(PanOrbitCameraPlugin)
        .add_systems(Update, toggle_camera_controls_system)
        // .init_gizmo_group::<MyRoundGizmos>()
        .add_systems(Startup, setup)
        .add_systems(Update, ui_example_system)
        // .add_systems(Update, mouse_click_system)
        // .add_systems(Update, print_mouse_events_system)
        // .add_systems(Update, draw_example_collection)
        .add_systems(Update, bounce_spheres);

    let mut dds_thread = Thread::default();

    {
        dds_thread = Thread::new(move || {
            while signal.is_run() {
                let recv_cloud = dds_handler.read_data(c"cloud_sub");
                for m in recv_cloud.iter() {
                    let data: PointCloud2 = PointCloud2::from_ptr(*m);

                    let timestamp_u64 = data.get_stamp();
                    let stamp =
                        OffsetDateTime::from_unix_timestamp_nanos(timestamp_u64 as i128).unwrap();

                    let current_stamp = OffsetDateTime::now_utc();

                    let stamp_diff = current_stamp - stamp;

                    let stamp_diff_s = stamp_diff.as_seconds_f32();
                    let stamp_local = stamp.to_offset(tz_offset);
                    let stamp_valid = (stamp_diff_s.abs() < 1.0);

                    let frame_id = data.get_frame_id();
                    let cloud_float_vec = data.get_data();

                    println!(
                        "recv data [{:?}] at {:?},cloud_float_vec.len : {}",
                        frame_id,
                        stamp,
                        cloud_float_vec.len()
                    );
                    // println!("float_vec: {:?}", float_vec);

                    float_vec = allocator.realloc_as_array::<f32>(float_vec, cloud_float_vec.len());

                    for (i, j) in izip!(cloud_float_vec.iter(), float_vec.iter_mut()) {
                        *j = if (i.is_finite()) { *i } else { 0.0 };
                    }

                    {
                        let float_vec_ptr = float_vec.as_ptr();
                        let float_vec_len = float_vec.len();
                        *GlobalData_BUFFER.get().unwrap().lock().unwrap() =
                            (UnsafeSender::new(float_vec_ptr), float_vec_len);
                    }
                }

                std::thread::sleep(std::time::Duration::from_millis(100));
            }
        });
    }
    app.run();

    println!("exit");
}

#[derive(Component)]
pub struct Ball
{
    velocity: Vec3,
}

#[derive(Resource)]
pub struct BallsCounter
{
    pub balls: i32,
}

// We can create our own gizmo config group!
#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

#[derive(Component)]
struct Bouncing;

fn bounce_spheres(time: Res<Time>, mut query: Query<&mut Transform, With<Bouncing>>)
{
    {
        if let Some(buffer) = GlobalData_BUFFER.get() {
            if let Ok(ref mut mutex) = buffer.try_lock() {
                let (ptr, len) = **mutex;
                println!("GlobalData_BUFFER {:?}, {}", ptr, len);

                let vec3_data: &[[f32; 3]] =
                    unsafe { slice::from_raw_parts_mut(*ptr.get() as *mut [f32; 3], len / 3) };

                for (i, mut transform) in query.iter_mut().enumerate() {
                    transform.translation.x = vec3_data[i][0];
                    transform.translation.y = vec3_data[i][1];
                    transform.translation.z = vec3_data[i][2];
                }
            } else {
                println!("GlobalData_BUFFER try_lock failed");
            }
        } else {
            println!("GlobalData_BUFFER empty");
        }
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
)
{
    // Ground
    commands.spawn(PbrBundle {
        mesh: meshes.add(Plane3d::default().mesh().size(25.0, 25.0)),
        material: materials.add(Color::rgba(1.0, 1.0, 1.0, 0.9)),
        ..default()
    });

    let sphere_radius = 0.01;
    let sphere_handle = meshes.add(Sphere::new(sphere_radius));

    let material = materials.add(Color::rgba(0.9, 0.3, 0.6, 0.9));

    for i in 0..HEIGHT * WIDTH {
        commands.spawn((
            PbrBundle {
                mesh: sphere_handle.clone(),
                material: material.clone(),
                transform: Transform::from_xyz(0.001 * i as f32, 0.0, 0.1),
                ..default()
            },
            Bouncing,
        ));
    }
    // // Cube
    // commands.spawn(PbrBundle {
    //     mesh: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
    //     material: materials.add(Color::rgb(0.8, 0.7, 0.6)),
    //     transform: Transform::from_xyz(0.0, 0.5, 0.0),
    //     ..default()
    // });
    // Light

    #[cfg(light)]
    {
        commands.spawn(PointLightBundle {
            point_light: PointLight {
                shadows_enabled: false,
                ..default()
            },
            transform: Transform::from_xyz(0.0, 0.0, 1.0),
            ..default()
        });
    }

    {
        // ambient light
        commands.insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 0.5,
        });

        // directional 'sun' light
        commands.spawn(DirectionalLightBundle {
            directional_light: DirectionalLight {
                illuminance: light_consts::lux::OVERCAST_DAY,
                shadows_enabled: false,
                ..default()
            },
            transform: Transform {
                translation: Vec3::new(0.0, 2.0, 0.0),
                rotation: Quat::from_rotation_x(-PI / 4.),
                ..default()
            },
            // The default cascade config is designed to handle large scenes.
            // As this example has a much smaller world, we can tighten the shadow
            // bounds for better visual quality.
            cascade_shadow_config: CascadeShadowConfigBuilder {
                first_cascade_far_bound: 4.0,
                maximum_distance: 10.0,
                ..default()
            }
            .into(),
            ..default()
        });
    }
    #[cfg(light)]
    commands.spawn(SpotLightBundle {
        spot_light: SpotLight {
            color: Default::default(),
            intensity: 0.0,
            range: 10.0,
            radius: 10.0,
            shadows_enabled: false,
            shadow_depth_bias: 0.0,
            shadow_normal_bias: 0.0,
            outer_angle: 0.0,
            inner_angle: 0.0,
        },
        visible_entities: Default::default(),
        frustum: Default::default(),
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        global_transform: Default::default(),
        visibility: Default::default(),
        inherited_visibility: Default::default(),
        view_visibility: Default::default(),
    });

    #[cfg(light)]
    commands.spawn(DirectionalLightBundle {
        directional_light: Default::default(),
        frusta: Default::default(),
        cascades: Default::default(),
        cascade_shadow_config: Default::default(),
        visible_entities: Default::default(),
        transform: Default::default(),
        global_transform: Default::default(),
        visibility: Visibility::Visible,
        inherited_visibility: Default::default(),
        view_visibility: Default::default(),
    });
    // Camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_translation(Vec3::new(1.0, 1.1, 5.0)),
            ..default()
        },
        PanOrbitCamera::default(),
    ));
    #[cfg(cam)]
    {
        commands.spawn((
            // Note we're setting the initial position below with yaw, pitch, and radius, hence
            // we don't set transform on the camera.
            Camera3dBundle::default(),
            PanOrbitCamera {
                // Set focal point (what the camera should look at)
                focus: Vec3::new(0.0, 1.0, 0.0),
                // Set the starting position, relative to focus (overrides camera's transform).
                yaw: Some(TAU / 8.0),
                pitch: Some(TAU / 8.0),
                radius: Some(5.0),
                // Set limits on rotation and zoom
                yaw_upper_limit: Some(TAU / 4.0),
                yaw_lower_limit: Some(-TAU / 4.0),
                pitch_upper_limit: Some(TAU / 3.0),
                pitch_lower_limit: Some(-TAU / 3.0),
                zoom_upper_limit: Some(5.0),
                zoom_lower_limit: Some(1.0),
                // Adjust sensitivity of controls
                orbit_sensitivity: 1.5,
                pan_sensitivity: 0.5,
                zoom_sensitivity: 0.5,
                // Allow the camera to go upside down
                allow_upside_down: true,
                // Change the controls (these match Blender)
                button_orbit: MouseButton::Middle,
                button_pan: MouseButton::Middle,
                modifier_pan: Some(KeyCode::ShiftLeft),
                // Reverse the zoom direction
                reversed_zoom: true,
                // Use alternate touch controls
                touch_controls: TouchControls::TwoFingerOrbit,
                ..default()
            },
        ));
    }

    commands.insert_resource(BallsCounter { balls: 0 });
}

fn move_balls(mut query: Query<(&mut Transform, &Ball)>, time: Res<Time>)
{
    for (mut tramsform, ball) in query.iter_mut() {
        tramsform.translation += ball.velocity * time.delta_seconds();
    }
}

fn remove_ball(
    mut commands: Commands,
    mut query: Query<(Entity, &Transform), With<Ball>>,
    mut ball_counter: ResMut<BallsCounter>,
)
{
    for (entity, transform) in query.iter_mut() {
        if distance(transform.translation.x, transform.translation.y) > 200.0 {
            commands.entity(entity).despawn();
            ball_counter.balls -= 1;
        }
    }
}

fn distance(x: f32, y: f32) -> f32
{
    let distance = ((x * x) + (y * y)).sqrt();
    distance
}

// This is how you can change config at runtime.
// Press 'T' to toggle the camera controls.
fn toggle_camera_controls_system(
    key_input: Res<ButtonInput<KeyCode>>,
    mut pan_orbit_query: Query<&mut PanOrbitCamera>,
)
{
    if key_input.just_pressed(KeyCode::KeyT) {
        println!("t pressed, toggle_camera_controls_system");
        for mut pan_orbit in pan_orbit_query.iter_mut() {
            pan_orbit.enabled = !pan_orbit.enabled;
        }
    }
}

fn ui_example_system(mut contexts: EguiContexts)
{
    egui::Window::new("Hello").show(contexts.ctx_mut(), |ui| {
        ui.label("world");
        if ui.selectable_label(true, "click").clicked() {
            println!("click label is clicked!!");
        }
    });
}

fn draw_example_collection(
    mut gizmos: Gizmos,
    mut my_gizmos: Gizmos<MyRoundGizmos>,
    time: Res<Time>,
)
{
    {
        if let Some(buffer) = GlobalData_BUFFER.get() {
            if let Ok(ref mut mutex) = buffer.try_lock() {
                let (ptr, len) = **mutex;
                println!("GlobalData_BUFFER {:?}, {}", ptr, len);

                let vec3_data: &[[f32; 3]] =
                    unsafe { slice::from_raw_parts_mut(*ptr.get() as *mut [f32; 3], len / 3) };

                for vec3 in vec3_data {
                    // gizmos.sphere(
                    //     Vec3::new(vec3[0], vec3[1], vec3[2]),
                    //     Default::default(),
                    //     0.01,
                    //     Default::default(),
                    // );
                    // gizmos.cuboid(
                    //     Transform::from_xyz(vec3[0], vec3[1], vec3[2])
                    //         .with_scale(Vec3::splat(0.01)),
                    //     Default::default(),
                    // );
                }
            } else {
                println!("GlobalData_BUFFER try_lock failed");
            }
        } else {
            println!("GlobalData_BUFFER empty");
        }
    }

    #[cfg(plot)]
    {
        gizmos.cuboid(
            Transform::from_translation(Vec3::Y * 0.5).with_scale(Vec3::splat(1.25)),
            Color::BLACK,
        );
        gizmos.rect(
            Vec3::new(time.elapsed_seconds().cos() * 2.5, 1., 0.),
            Quat::from_rotation_y(PI / 2.),
            Vec2::splat(2.),
            Color::GREEN,
        );

        my_gizmos.sphere(Vec3::new(1., 0.5, 0.), Quat::IDENTITY, 0.5, Color::RED);

        for y in [0., 0.5, 1.] {
            gizmos.ray(
                Vec3::new(1., y, 0.),
                Vec3::new(-3., (time.elapsed_seconds() * 3.).sin(), 0.),
                Color::BLUE,
            );
        }

        my_gizmos
            .arc_3d(
                180.0_f32.to_radians(),
                0.2,
                Vec3::ONE,
                Quat::from_rotation_arc(Vec3::Y, Vec3::ONE.normalize()),
                Color::ORANGE,
            )
            .segments(10);

        // Circles have 32 line-segments by default.
        my_gizmos.circle(Vec3::ZERO, Direction3d::Y, 3., Color::BLACK);
        // You may want to increase this for larger circles or spheres.
        my_gizmos
            .circle(Vec3::ZERO, Direction3d::Y, 3.1, Color::NAVY)
            .segments(64);
        my_gizmos
            .sphere(Vec3::ZERO, Quat::IDENTITY, 3.2, Color::BLACK)
            .circle_segments(64);

        gizmos.arrow(Vec3::ZERO, Vec3::ONE * 1.5, Color::YELLOW);
    }
}

// This system prints messages when you press or release the left mouse button:
fn mouse_click_system(mouse_button_input: Res<ButtonInput<MouseButton>>)
{
    if mouse_button_input.pressed(MouseButton::Left) {
        info!("left mouse currently pressed");
    }

    if mouse_button_input.just_pressed(MouseButton::Left) {
        info!("left mouse just pressed");
    }

    if mouse_button_input.just_released(MouseButton::Left) {
        info!("left mouse just released");
    }
}

/// This system prints out all mouse events as they come in
fn print_mouse_events_system(
    mut mouse_button_input_events: EventReader<MouseButtonInput>,
    mut mouse_motion_events: EventReader<MouseMotion>,
    mut cursor_moved_events: EventReader<CursorMoved>,
    mut mouse_wheel_events: EventReader<MouseWheel>,
    mut touchpad_magnify_events: EventReader<TouchpadMagnify>,
    mut touchpad_rotate_events: EventReader<TouchpadRotate>,
)
{
    for event in mouse_button_input_events.read() {
        info!("{:?}", event);
    }

    for event in mouse_motion_events.read() {
        info!("{:?}", event);
    }

    for event in cursor_moved_events.read() {
        info!("{:?}", event);
    }

    for event in mouse_wheel_events.read() {
        info!("{:?}", event);
    }

    // This event will only fire on macOS
    for event in touchpad_magnify_events.read() {
        info!("{:?}", event);
    }

    // This event will only fire on macOS
    for event in touchpad_rotate_events.read() {
        info!("{:?}", event);
    }
}
fn rotate_camera(mut query: Query<&mut Transform, With<Camera>>, time: Res<Time>)
{
    let mut transform = query.single_mut();

    transform.rotate_around(Vec3::ZERO, Quat::from_rotation_y(time.delta_seconds() / 2.));
}
