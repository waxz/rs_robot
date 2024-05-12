use nx_common::common::signal_handler::SignalHandler;
use nx_common::common::thread::Thread;
use nx_message_center::base::{common_message, message_handler, tiny_alloc};

use nx_common::common::math::{Point3f};

use itertools::{izip, Itertools};
use nx_common::common::types::UnsafeSender;
use nx_message_center::base::common_message::shared::PointCloud2;

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

    #[arg(long, default_value_t = 480)]
    height: u32,

    #[arg(long, default_value_t = 640)]
    width: u32,
}

static GlobalData_BUFFER: OnceLock<Arc<Mutex<(UnsafeSender<*const f32>, usize)>>> = OnceLock::new();
static THE_THING: OnceLock<i32> = OnceLock::new();
static HEIGHT: OnceLock<u32> = OnceLock::new();
static WIDTH: OnceLock<u32> = OnceLock::new();

//BEVY
use bevy_mod_raycast::prelude::*;

use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin, TouchControls};
use bevy_mod_picking::prelude::*;

use bevy::pbr::CascadeShadowConfigBuilder;
use bevy::{
    utils::Duration,
    window::{PresentMode, RequestRedraw, WindowPlugin},
    winit::WinitSettings,

    core_pipeline::core_3d::Transparent3d,
    ecs::{
        query::QueryItem,
        system::{lifetimeless::*, SystemParamItem},
    },
    pbr::{
        MeshPipeline, MeshPipelineKey, RenderMeshInstances, SetMeshBindGroup, SetMeshViewBindGroup,
    },
    prelude::*,
    render::{
        extract_component::{ExtractComponent, ExtractComponentPlugin},
        mesh::{GpuBufferInfo, MeshVertexBufferLayout},
        render_asset::RenderAssets,
        render_phase::{
            AddRenderCommand, DrawFunctions, PhaseItem, RenderCommand, RenderCommandResult,
            RenderPhase, SetItemPipeline, TrackedRenderPass,
        },
        render_resource::*,
        renderer::RenderDevice,
        view::{ExtractedView, NoFrustumCulling},
        Render, RenderApp, RenderSet,
    },
};
use bevy_egui::{egui, EguiContexts, EguiPlugin};

use bytemuck::{Pod, Zeroable};

fn main()
{
    let tz_offset = UtcOffset::current_local_offset().expect("should get local offset!");
    let memory_pool_base = nx_common::common::memory::get_next_aligned_addr(
        unsafe { TA_BUFFER.as_ptr() } as *mut _,
        8,
    );
    let allocator = tiny_alloc::TinyAlloc::new(memory_pool_base, TA_BUFFER_SIZE - 8, 512, 16, 8);

    let args = Args::parse();

    WIDTH.get_or_init(|| args.width);
    HEIGHT.get_or_init(|| args.height);

    let mut float_vec = allocator.alloc_as_array::<f32>((args.width * args.height) as usize);

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
    app.insert_resource(ClearColor(Color::rgb(0.5, 0.5, 0.5)))
        .insert_resource(WinitSettings {
            // focused_mode: bevy::winit::UpdateMode::Continuous,

            focused_mode: bevy::winit::UpdateMode::ReactiveLowPower {
                wait: Duration::from_millis(100),
            },

            unfocused_mode: bevy::winit::UpdateMode::ReactiveLowPower {
                wait: Duration::from_millis(500),
            },
        })

        .add_plugins((DefaultPlugins, CustomMaterialPlugin))
        .add_plugins(EguiPlugin)
        //
        // .add_plugins(DefaultPickingPlugins
        //     .build()
        //     .disable::<DebugPickingPlugin>()
        // )
        // .insert_resource(DebugPickingMode::Normal)
        // .add_systems(Update, make_pickable)
         //

        .add_plugins(PanOrbitCameraPlugin)
        .add_systems(Startup, setup_camera)
        .add_systems(Update, camera_keyboard_controls)
        //
        // .add_plugins(LookTransformPlugin)
        // .add_systems(Startup, setup_look_cam)

        //
        // .add_systems(Startup,setup_plane)
        //
        // .add_systems(Startup, create_pontcloud)
        // .add_systems(Update, replot_cloud)
        //
        .add_systems(Startup, create_1million_cube)
        .add_systems(Update, replot_1million_instance)
        //
        .add_systems(Update, draw_axis)
        .init_resource::<OccupiedScreenSpace>()
        .add_systems(Update, create_egui_sidebar)
        // .add_systems(Startup,setup_pick)

    // cast
        .add_plugins(DefaultRaycastingPlugin)
        .add_systems(Update, raycast)

    ;

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

                    float_vec = allocator.realloc_as_array::<f32>(float_vec, cloud_float_vec.len());

                    for (i, j) in izip!(cloud_float_vec.iter(), float_vec.iter_mut()) {
                        *j = if (i.is_finite()) { *i } else { 0.0 };
                    }
                    // println!("cloud_float_vec: {:?}", cloud_float_vec);
                    // println!("float_vec: {:?}", float_vec);

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
}



fn raycast(cursor_ray: Res<CursorRay>, mut query: Query<&mut InstanceMaterialData>) {
    if let Some(ray) = **cursor_ray {
        let origin = Point3f{
            x : ray.origin.x,
            y : ray.origin.y,
            z : ray.origin.z,

        };
        let direction = Point3f{
            x : ray.direction.x,
            y : ray.direction.y,
            z : ray.direction.z,

        };
        // println!("ray : {:?}, {:?}", origin, direction);
        let rotate = 0.01;
        for (i, mut data) in query.iter_mut().enumerate() {
            let mut data = &mut data.0;
            for (i, v) in data.iter_mut().enumerate() {

                let entity =  Point3f{
                    x : v.position.x,
                    y : v.position.y,
                    z : v.position.z,

                };
                let distance = Point3f::distance_to_line(origin,direction, entity);

                if(distance < 1.0){

                    if v.selected ==0{
                        v.selected =1;
                    }else
                    if v.selected ==1{
                        v.selected = 0;
                    }

                }else {
                    // v.color[0] = 0.0;
                    // v.selected = 0;

                }
                if v.selected > 0{
                    let r = (v.position.x * v.position.x + v.position.y * v.position.y).sqrt();
                    let a = v.position.y.atan2(v.position.x);
                    v.position.x = r * (a + rotate).cos();
                    v.position.y = r * (a + rotate).sin();
                    // v.color[0] = 1.0;
                    v.color[3] = 1.0;

                }


            }
        }
    }

}

fn setup_pick(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
){


    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::default()),
            material: materials.add(Color::rgb(0.8, 0.7, 0.6)),
            transform: Transform::from_xyz(5.0, 2.5, 2.0),
            ..default()
        },
        // Rotate an entity when dragged:
        On::<Pointer<Drag>>::target_component_mut::<Transform>(|drag, transform| {
            transform.rotate_local_y(drag.delta.x / 50.0)
        })
    ));
}


#[derive(Default, Resource)]
struct OccupiedScreenSpace {
    left: f32,
    top: f32,
    right: f32,
    bottom: f32,
}


fn create_egui_sidebar(
    mut contexts: EguiContexts,
    mut occupied_screen_space: ResMut<OccupiedScreenSpace>,
) {
    let ctx = contexts.ctx_mut();

    occupied_screen_space.left = egui::SidePanel::left("left_panel")
        .resizable(true)
        .show(ctx, |ui| {
            ui.label("Left resizeable panel");
            ui.allocate_rect(ui.available_rect_before_wrap(), egui::Sense::hover());
        })
        .response
        .rect
        .width();
    occupied_screen_space.right = egui::SidePanel::right("right_panel")
        .resizable(true)
        .show(ctx, |ui| {
            ui.label("Right resizeable panel");
            ui.allocate_rect(ui.available_rect_before_wrap(), egui::Sense::hover());
        })
        .response
        .rect
        .width();
    occupied_screen_space.top = egui::TopBottomPanel::top("top_panel")
        .resizable(true)
        .show(ctx, |ui| {
            ui.label("Top resizeable panel");
            ui.allocate_rect(ui.available_rect_before_wrap(), egui::Sense::hover());
        })
        .response
        .rect
        .height();
    occupied_screen_space.bottom = egui::TopBottomPanel::bottom("bottom_panel")
        .resizable(true)

        .show(ctx, |ui| {
            ui.label("Bottom resizeable panel");
            ui.allocate_rect(ui.available_rect_before_wrap(), egui::Sense::hover());
        })
        .response
        .rect
        .height();
}



fn ui_example_system(mut contexts: EguiContexts)
{
    egui::Window::new("Hello").show(contexts.ctx_mut(), |ui| {
        ui.label("world");
        if ui.selectable_label(true, "click").clicked() {
            println!("click label is clicked!!");
        }
    });
    egui::Window::new("Menu").show(contexts.ctx_mut(), |ui| {
        ui.label("Tool");
        if ui.selectable_label(true, "Exit").clicked() {
            println!("Exit label is clicked!!");
        }
    });

}


// This is how you can change config at runtime.
// Press 'T' to toggle the camera controls.

fn setup_camera(mut commands: Commands)
{

    // Camera
    commands.spawn((
        Camera3dBundle {
            // transform: Transform::from_translation(Vec3::new(1., 1., 1.0)),
            transform: Transform::from_xyz(1.0, 1.0, 1.0).looking_at(Vec3::ZERO, Vec3::Z),
            ..default()
        },
        PanOrbitCamera{
            radius: Some(10.0),
            zoom_upper_limit: Some(100.0),
            zoom_lower_limit: Some(0.01),
            ..default()
        },
    ));

    //light
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
}


fn camera_keyboard_controls(
    time: Res<Time>,
    key_input: Res<ButtonInput<KeyCode>>,
    mut pan_orbit_query: Query<(&mut PanOrbitCamera, &mut Transform)>,
) {





    for (mut pan_orbit, mut transform) in pan_orbit_query.iter_mut() {



        if key_input.pressed(KeyCode::ControlLeft) {
            if key_input.just_pressed(KeyCode::KeyT) {
                pan_orbit.enabled = !pan_orbit.enabled;
                return;
            }

            // Jump focus point 1m using Ctrl+Shift + Arrows
            if key_input.pressed(KeyCode::ShiftLeft) {

                //    z+  y+
                //x-  []  x+
                //y-  z-

                let step = 0.5;
                if key_input.just_pressed(KeyCode::Numpad1) {
                    pan_orbit.target_focus -= step* Vec3::Y;
                }
                if key_input.just_pressed(KeyCode::Numpad9) {
                    pan_orbit.target_focus += step* Vec3::Y;
                }

                if key_input.just_pressed(KeyCode::Numpad2) {
                    pan_orbit.target_focus -= step* Vec3::Z;
                }
                if key_input.just_pressed(KeyCode::Numpad8) {
                    pan_orbit.target_focus += step* Vec3::Z;
                }

                if key_input.just_pressed(KeyCode::Numpad4) {
                    pan_orbit.target_focus -= step* Vec3::X;
                }
                if key_input.just_pressed(KeyCode::Numpad6) {
                    pan_orbit.target_focus += step* Vec3::X;
                }
            } else {
                // Jump by 45 degrees using Left Ctrl + Arrows
                if key_input.just_pressed(KeyCode::ArrowRight) {
                    pan_orbit.target_yaw += 45f32.to_radians();
                }
                if key_input.just_pressed(KeyCode::ArrowLeft) {
                    pan_orbit.target_yaw -= 45f32.to_radians();
                }
                if key_input.just_pressed(KeyCode::ArrowUp) {
                    pan_orbit.target_pitch += 45f32.to_radians();
                }
                if key_input.just_pressed(KeyCode::ArrowDown) {
                    pan_orbit.target_pitch -= 45f32.to_radians();
                }
            }
        }
        // Pan using Left Shift + Arrows
        else if key_input.pressed(KeyCode::ShiftLeft) {
            let mut delta_translation = Vec3::ZERO;
            if key_input.pressed(KeyCode::ArrowRight) {
                delta_translation += transform.rotation * Vec3::X * time.delta_seconds();
            }
            if key_input.pressed(KeyCode::ArrowLeft) {
                delta_translation += transform.rotation * Vec3::NEG_X * time.delta_seconds();
            }
            if key_input.pressed(KeyCode::ArrowUp) {
                delta_translation += transform.rotation * Vec3::Y * time.delta_seconds();
            }
            if key_input.pressed(KeyCode::ArrowDown) {
                delta_translation += transform.rotation * Vec3::NEG_Y * time.delta_seconds();
            }
            transform.translation += delta_translation;
            pan_orbit.target_focus += delta_translation;
        }
        // Smooth rotation using arrow keys without modifier
        else {
            if key_input.pressed(KeyCode::ArrowRight) {
                pan_orbit.target_yaw += 50f32.to_radians() * time.delta_seconds();
            }
            if key_input.pressed(KeyCode::ArrowLeft) {
                pan_orbit.target_yaw -= 50f32.to_radians() * time.delta_seconds();
            }
            if key_input.pressed(KeyCode::ArrowUp) {
                pan_orbit.target_pitch += 50f32.to_radians() * time.delta_seconds();
            }
            if key_input.pressed(KeyCode::ArrowDown) {
                pan_orbit.target_pitch -= 50f32.to_radians() * time.delta_seconds();
            }

            // Zoom with Z and X
            if key_input.pressed(KeyCode::KeyZ) {
                pan_orbit.target_radius -= 5.0 * time.delta_seconds();
            }
            if key_input.pressed(KeyCode::KeyX) {
                pan_orbit.target_radius += 5.0 * time.delta_seconds();
            }
        }

        // Force camera to update its transform
        pan_orbit.force_update = true;
    }
}

fn draw_axis(
    mut gizmos: Gizmos,
    time: Res<Time>
){
    gizmos.ray(
        Vec3::new(0.,0.,0.),
        Vec3::new(1.,0.,0.),
        Color::RED,
    );
    gizmos.ray(
        Vec3::new(0.,0.,0.),
        Vec3::new(0.,1.,0.),
        Color::GREEN,
    );
    gizmos.ray(
        Vec3::new(0.,0.,0.),
        Vec3::new(0.,0.,1.),
        Color::BLUE,
    );

}
fn setup_plane(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
){
    // Ground
    commands.spawn(PbrBundle {
        mesh: meshes.add(Plane3d::default().mesh().size(10.0, 10.0)),
        material: materials.add(Color::rgba(1.0, 1.0, 1.0, 0.9)),
        transform: Transform {
            translation: Vec3::new(0.0, 0.0, 0.0),
            rotation: Quat::from_rotation_x(0.0),
            ..default()
        },
        ..default()
    });
    commands.spawn(PbrBundle {
        mesh: meshes.add(Plane3d::default().mesh().size(10.0, 10.0)),
        material: materials.add(Color::rgba(1.0, 1.0, 1.0, 0.9)),
        transform: Transform {
            translation: Vec3::new(0.0, 0.0, 0.0),
            rotation: Quat::from_rotation_x(-PI / 2.),
            ..default()
        },
        ..default()
    });
}
fn create_1million_cube(mut commands: Commands,
                        mut meshes: ResMut<Assets<Mesh>>,
                        mut materials: ResMut<Assets<StandardMaterial>>,
)
{



    let num_particles = 10;
    let extent = 10.0 / 2.0;

    let jump = extent / num_particles as f32;

    let instances = (0..=num_particles)
        .cartesian_product(0..=num_particles)
        .cartesian_product(0..=num_particles)
        .map(|((z, y), x)| {
            let xf = x as f32 * jump - 0.5 * extent;
            let yf = y as f32 * jump - 0.5 * extent;
            let zf = z as f32 * jump - 0.5 * extent;

            InstanceData {
                position: Vec3::new(xf, yf, zf),
                scale: 0.5,
                color: [
                    if (x % 3 == 0) { 0.0 } else { 1.0 },
                    if (y % 4 == 0) { 0.0 } else { 1.0 },
                    if (z % 5 == 0) { 0.0 } else { 1.0 },
                    0.1,
                ],
                selected: 0,
            }
        })
        .collect::<Vec<_>>();

    commands.spawn((
        meshes.add(Cuboid::new(0.5, 0.5, 0.5)),
        materials.add(Color::GRAY),

        SpatialBundle::INHERITED_IDENTITY,
        InstanceMaterialData(instances),
        // NOTE: Frustum culling is done based on the Aabb of the Mesh and the GlobalTransform.
        // As the cube is at the origin, if its Aabb moves outside the view frustum, all the
        // instanced cubes will be culled.
        // The InstanceMaterialData contains the 'GlobalTransform' information for this custom
        // instancing, and that is not taken into account with the built-in frustum culling.
        // We must disable the built-in frustum culling by adding the `NoFrustumCulling` marker
        // component to avoid incorrect culling.
        NoFrustumCulling,
        // Events that target children of the scene will bubble up to this level and will fire off a
        // `HelmetClicked` event.
        PickableBundle::default(),
    ));
}

/// Makes everything in the scene with a mesh pickable
fn make_pickable(
    mut commands: Commands,
    meshes: Query<Entity, (With<Handle<Mesh>>, Without<Pickable>)>,
) {
    for entity in meshes.iter() {
        commands
            .entity(entity)
            .insert((PickableBundle::default(), HIGHLIGHT_TINT.clone()));
    }
}

/// Used to tint the mesh instead of simply replacing the mesh's material with a single color. See
/// `tinted_highlight` for more details.
const HIGHLIGHT_TINT: Highlight<StandardMaterial> = Highlight {
    hovered: Some(HighlightKind::new_dynamic(|matl| StandardMaterial {
        base_color: matl.base_color + Color::rgba(-0.5, -0.3, 0.9, 0.8), // hovered is blue
        ..matl.to_owned()
    })),
    pressed: Some(HighlightKind::new_dynamic(|matl| StandardMaterial {
        base_color: matl.base_color + Color::rgba(-0.4, -0.4, 0.8, 0.8), // pressed is a different blue
        ..matl.to_owned()
    })),
    selected: Some(HighlightKind::new_dynamic(|matl| StandardMaterial {
        base_color: matl.base_color + Color::rgba(-0.4, 0.8, -0.4, 0.0), // selected is green
        ..matl.to_owned()
    })),
};

fn create_pontcloud(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>)
{
    let height = HEIGHT.get().unwrap();
    let width = WIDTH.get().unwrap();

    let particles = height * width;
    let instances: Vec<_> = (0..particles)
        .map(|_| InstanceData {
            position: Vec3::new(0.0, 0.0, 0.0),
            scale: 0.01,
            color: [1.0, 0.0, 0.0, 0.9],
            selected: 0,
        })
        .collect();

    commands.spawn((
        meshes.add(Cuboid::new(0.5, 0.5, 0.5)),
        SpatialBundle::INHERITED_IDENTITY,
        InstanceMaterialData(instances),
        // NOTE: Frustum culling is done based on the Aabb of the Mesh and the GlobalTransform.
        // As the cube is at the origin, if its Aabb moves outside the view frustum, all the
        // instanced cubes will be culled.
        // The InstanceMaterialData contains the 'GlobalTransform' information for this custom
        // instancing, and that is not taken into account with the built-in frustum culling.
        // We must disable the built-in frustum culling by adding the `NoFrustumCulling` marker
        // component to avoid incorrect culling.
        NoFrustumCulling,

    ));
}

#[derive(Component, Deref)]
struct InstanceMaterialData(Vec<InstanceData>);

impl ExtractComponent for InstanceMaterialData
{
    type QueryData = &'static InstanceMaterialData;
    type QueryFilter = ();
    type Out = Self;

    fn extract_component(item: QueryItem<'_, Self::QueryData>) -> Option<Self>
    {
        Some(InstanceMaterialData(item.0.clone()))
    }
}

struct CustomMaterialPlugin;

impl Plugin for CustomMaterialPlugin
{
    fn build(&self, app: &mut App)
    {
        app.add_plugins(ExtractComponentPlugin::<InstanceMaterialData>::default());
        app.sub_app_mut(RenderApp)
            .add_render_command::<Transparent3d, DrawCustom>()
            .init_resource::<SpecializedMeshPipelines<CustomPipeline>>()
            .add_systems(
                Render,
                (
                    queue_custom.in_set(RenderSet::QueueMeshes),
                    prepare_instance_buffers.in_set(RenderSet::PrepareResources),
                ),
            );
    }

    fn finish(&self, app: &mut App)
    {
        app.sub_app_mut(RenderApp).init_resource::<CustomPipeline>();
    }
}

#[derive(Clone, Copy, Pod, Zeroable)]
#[repr(C)]
struct InstanceData
{
    position: Vec3,
    scale: f32,
    color: [f32; 4],
    selected:u32,
}

#[allow(clippy::too_many_arguments)]
fn queue_custom(
    transparent_3d_draw_functions: Res<DrawFunctions<Transparent3d>>,
    custom_pipeline: Res<CustomPipeline>,
    msaa: Res<Msaa>,
    mut pipelines: ResMut<SpecializedMeshPipelines<CustomPipeline>>,
    pipeline_cache: Res<PipelineCache>,
    meshes: Res<RenderAssets<Mesh>>,
    render_mesh_instances: Res<RenderMeshInstances>,
    material_meshes: Query<Entity, With<InstanceMaterialData>>,
    mut views: Query<(&ExtractedView, &mut RenderPhase<Transparent3d>)>,
)
{
    let draw_custom = transparent_3d_draw_functions.read().id::<DrawCustom>();

    let msaa_key = MeshPipelineKey::from_msaa_samples(msaa.samples());

    for (view, mut transparent_phase) in &mut views {
        let view_key = msaa_key | MeshPipelineKey::from_hdr(view.hdr);
        let rangefinder = view.rangefinder3d();
        for entity in &material_meshes {
            let Some(mesh_instance) = render_mesh_instances.get(&entity) else {
                continue;
            };
            let Some(mesh) = meshes.get(mesh_instance.mesh_asset_id) else {
                continue;
            };
            let key = view_key | MeshPipelineKey::from_primitive_topology(mesh.primitive_topology);
            let pipeline = pipelines
                .specialize(&pipeline_cache, &custom_pipeline, key, &mesh.layout)
                .unwrap();
            transparent_phase.add(Transparent3d {
                entity,
                pipeline,
                draw_function: draw_custom,
                distance: rangefinder
                    .distance_translation(&mesh_instance.transforms.transform.translation),
                batch_range: 0..1,
                dynamic_offset: None,
            });
        }
    }
}

#[derive(Component)]
struct InstanceBuffer
{
    buffer: Buffer,
    length: usize,
}

fn prepare_instance_buffers(
    mut commands: Commands,
    query: Query<(Entity, &InstanceMaterialData)>,
    render_device: Res<RenderDevice>,
)
{
    for (entity, instance_data) in &query {
        let buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("instance data buffer"),
            contents: bytemuck::cast_slice(instance_data.as_slice()),
            usage: BufferUsages::VERTEX | BufferUsages::COPY_DST,
        });
        commands.entity(entity).insert(InstanceBuffer {
            buffer,
            length: instance_data.len(),
        });
    }
}

#[derive(Resource)]
struct CustomPipeline
{
    shader: Handle<Shader>,
    mesh_pipeline: MeshPipeline,
}

impl FromWorld for CustomPipeline
{
    fn from_world(world: &mut World) -> Self
    {
        let asset_server = world.resource::<AssetServer>();
        let shader = asset_server.load("shaders/instancing.wgsl");

        let mesh_pipeline = world.resource::<MeshPipeline>();

        CustomPipeline {
            shader,
            mesh_pipeline: mesh_pipeline.clone(),
        }
    }
}

impl SpecializedMeshPipeline for CustomPipeline
{
    type Key = MeshPipelineKey;

    fn specialize(
        &self,
        key: Self::Key,
        layout: &MeshVertexBufferLayout,
    ) -> Result<RenderPipelineDescriptor, SpecializedMeshPipelineError>
    {
        let mut descriptor = self.mesh_pipeline.specialize(key, layout)?;

        descriptor.vertex.shader = self.shader.clone();
        descriptor.vertex.buffers.push(VertexBufferLayout {
            array_stride: std::mem::size_of::<InstanceData>() as u64,
            step_mode: VertexStepMode::Instance,
            attributes: vec![
                VertexAttribute {
                    format: VertexFormat::Float32x4,
                    offset: 0,
                    shader_location: 3, // shader locations 0-2 are taken up by Position, Normal and UV attributes
                },
                VertexAttribute {
                    format: VertexFormat::Float32x4,
                    offset: VertexFormat::Float32x4.size(),
                    shader_location: 4,
                },
            ],
        });
        descriptor.fragment.as_mut().unwrap().shader = self.shader.clone();
        Ok(descriptor)
    }
}

type DrawCustom = (
    SetItemPipeline,
    SetMeshViewBindGroup<0>,
    SetMeshBindGroup<1>,
    DrawMeshInstanced,
);

struct DrawMeshInstanced;

impl<P: PhaseItem> RenderCommand<P> for DrawMeshInstanced
{
    type Param = (SRes<RenderAssets<Mesh>>, SRes<RenderMeshInstances>);
    type ViewQuery = ();
    type ItemQuery = Read<InstanceBuffer>;

    #[inline]
    fn render<'w>(
        item: &P,
        _view: (),
        instance_buffer: Option<&'w InstanceBuffer>,
        (meshes, render_mesh_instances): SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult
    {
        let Some(mesh_instance) = render_mesh_instances.get(&item.entity()) else {
            return RenderCommandResult::Failure;
        };
        let Some(gpu_mesh) = meshes.into_inner().get(mesh_instance.mesh_asset_id) else {
            return RenderCommandResult::Failure;
        };
        let Some(instance_buffer) = instance_buffer else {
            return RenderCommandResult::Failure;
        };

        pass.set_vertex_buffer(0, gpu_mesh.vertex_buffer.slice(..));
        pass.set_vertex_buffer(1, instance_buffer.buffer.slice(..));

        match &gpu_mesh.buffer_info {
            GpuBufferInfo::Indexed {
                buffer,
                index_format,
                count,
            } => {
                pass.set_index_buffer(buffer.slice(..), 0, *index_format);
                pass.draw_indexed(0..*count, 0, 0..instance_buffer.length as u32);
            }
            GpuBufferInfo::NonIndexed => {
                pass.draw(0..gpu_mesh.vertex_count, 0..instance_buffer.length as u32);
            }
        }
        RenderCommandResult::Success
    }
}

fn replot_cloud(time: Res<Time>, mut query: Query<&mut InstanceMaterialData>)
{

    println!("query.iter().len() = {:?} ", query.iter().len());

    {
        if let Some(buffer) = GlobalData_BUFFER.get() {
            if let Ok(ref mut mutex) = buffer.try_lock() {
                let (ptr, len) = **mutex;
                println!("GlobalData_BUFFER {:?}, {}", ptr, len);

                let vec3_data: &[[f32; 3]] =
                    unsafe { slice::from_raw_parts_mut(*ptr.get() as *mut [f32; 3], len / 3) };
                // println!("vec3_data {:?}, {:?}", vec3_data.len(), vec3_data);

                for (_, mut data) in query.iter_mut().enumerate() {
                    let mut data = &mut data.0;
                    println!(
                        "data.len: {}, vec3_data.len: {}",
                        data.len(),
                        vec3_data.len()
                    );

                    for (a, b) in izip!(vec3_data, data.iter_mut()) {
                        b.position.x = a[0];
                        b.position.y = a[1];
                        b.position.z = a[2];
                        let r = (a[0]*a[0] + a[1]*a[1] + a[2]*a[2] ).sqrt();

                        let h = Color::hsl(r*60.0,0.5,0.5).as_rgba();
                        b.color[0] = h.r();
                        b.color[1] = h.g();
                        b.color[2] = h.b();
                        b.color[3] = h.a();


                    }

                    // for (i, v) in data.iter_mut().enumerate() {
                    //     println!("{}",i);
                    //     let r = (v.position.x * v.position.x + v.position.y * v.position.y).sqrt();
                    //     let a = v.position.y.atan2(v.position.x);
                    //     v.position.x = vec3_data[i][0];
                    //     v.position.y = vec3_data[i][1];
                    //     v.position.z = vec3_data[i][2];
                    //
                    // }
                    break;
                }
            } else {
                println!("GlobalData_BUFFER try_lock failed");
            }
        } else {
            println!("GlobalData_BUFFER empty");
        }
    }
}
fn replot_1million_instance(time: Res<Time>, mut query: Query<&mut InstanceMaterialData>)
{
    let rotate = 0.0;
    for (i, mut data) in query.iter_mut().enumerate() {
        let mut data = &mut data.0;
        for (i, v) in data.iter_mut().enumerate() {
            let r = (v.position.x * v.position.x + v.position.y * v.position.y).sqrt();
            let a = v.position.y.atan2(v.position.x);
            v.position.x = r * (a + rotate).cos();
            v.position.y = r * (a + rotate).sin();
        }
    }
}
