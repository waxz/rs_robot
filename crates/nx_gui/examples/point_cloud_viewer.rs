use nx_common::common::signal_handler::SignalHandler;
use nx_common::common::thread::Thread;
use nx_message_center::base::{common_message, message_handler, tiny_alloc};

use nx_common::common::math::Point3f;

use itertools::{izip, Itertools};
use nx_common::common::types::UnsafeSender;
use nx_message_center::base::common_message::shared::PointCloud2;

use std::f32::consts::{PI, TAU};
use std::f64::consts::FRAC_PI_2;
use std::ffi::c_void;
use std::ops::Deref;
use std::slice;
use time::{OffsetDateTime, UtcOffset};

use bevy::app::{Startup, Update};
use bevy::log::{debug, debug_once, error, info, warn};
use bevy::math::quat;
use bevy::prelude::{
    ButtonInput, Color, Commands, Gizmos, KeyCode, MouseButton, PositionType, Quat, Query, Res,
    ResMut, Resource, Style, TextBundle, TextStyle, Transform, Val, Vec2, Vec3,
};
use bevy_egui::egui::{ComboBox, ScrollArea, Slider};
use bevy_egui::{egui, EguiContexts};
use bevy_mod_raycast::CursorRay;
use rand::{thread_rng, Rng};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;
use std::sync::{Arc, Mutex, MutexGuard};
// argument
use clap::Parser;
use nx_gui::app_builder::{create_bevy_app, CameraFocusRay};
use nx_gui::shaders::{setup_shaders_render, InstanceMaterialData, ShaderResConfig};
use nx_message_center::base::pointcloud_process::perception::{
    pointcloud_clip, pointcloud_transform,
};
use serde::Deserialize;

//-------------

const TA_BUFFER_SIZE: usize = 100000000;
static mut TA_BUFFER: [u8; TA_BUFFER_SIZE] = [0; TA_BUFFER_SIZE];

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args
{
    #[arg(
        short,
        long,
        default_value = "/home/waxz/RustroverProjects/rust_practice/crates/nx_gui/config/gui_config.toml"
    )]
    dds_config: String,

    #[arg(long, default_value_t = 480)]
    height: usize,

    #[arg(long, default_value_t = 640)]
    width: usize,
}

//------------------
struct SimpleState
{
    pub enable_dds_update: bool,
    pub dds_msg_count: usize,
    // select point scale need be larger
    pub point_default_scale: f32,
    pub point_highlight_selected_scale: f32,
    pub point_highlight_unselected_scale: f32,
    pub tool_ray_radius: f32,
    pub tool_ray_distance: f32,

    pub start_select: bool,
    pub reset_vertex: bool,
    pub add_selected_to_vec: bool,

    pub vertex_color_hsv_ratio: f32,

    pub exit: bool,

    pub two_cluster_distance_index: [usize; 2],
    pub filter: CloudFilterConfig,
    pub operator_mode : u32,
    pub operator_run : bool,
}
#[derive(Copy, Clone)]
struct CloudFilterConfig
{
    enable: bool,
    filter_height_min: u64,
    filter_height_max: u64,
    filter_width_min: u64,
    filter_width_max: u64,
    point_num: u32,
}

#[derive(Debug, Deserialize, Copy, Clone)]
struct Pose
{
    tx: f32,
    ty: f32,
    tz: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
}
#[derive(Debug, Deserialize, Copy, Clone)]
struct PoseMarker
{
    pose: Pose,
    enable: bool,
}

struct StaticSharedData
{
    pub cloud_buffer: Arc<Mutex<(UnsafeSender<*const f32>, usize)>>,
    pub cloud_dim: [usize; 2],
    pub state: Arc<Mutex<SimpleState>>,

    // col and row filter
    // x,y,z range filter
    // multiple pick group 0
    // multiple pick group 1
    // multiple pick group 2
    // multiple pick group 3
    pub tool_selected_points_index: Arc<Mutex<Vec<Vec<usize>>>>,
    pub marker_pose: Arc<Mutex<Vec<PoseMarker>>>,
}

static GLOBAL_DATA: OnceLock<StaticSharedData> = OnceLock::new();

fn main()
{
    let tz_offset = nx_common::common::time::get_timezone_offset();
    let memory_pool_base = nx_common::common::memory::get_next_aligned_addr(
        unsafe { TA_BUFFER.as_ptr() } as *mut _,
        8,
    );
    let allocator = tiny_alloc::TinyAlloc::new(memory_pool_base, TA_BUFFER_SIZE - 8, 512, 16, 8);

    let args = Args::parse();

    let mut float_vec = allocator.alloc_as_array::<f32>((args.width * args.height * 3) as usize);

    {
        let float_vec_ptr = float_vec.as_ptr();
        let float_vec_len = float_vec.len();
        GLOBAL_DATA.get_or_init(move || {
            println!("set crate::GLOBAL_DATA::BUFFER");
            StaticSharedData {
                cloud_dim: [args.height, args.width],

                cloud_buffer: Arc::new(Mutex::new((
                    UnsafeSender::new(float_vec_ptr),
                    float_vec_len,
                ))),
                tool_selected_points_index: Arc::new(Mutex::new(vec![])),
                state: Arc::new(Mutex::new(SimpleState {
                    enable_dds_update: true,
                    dds_msg_count: 0,
                    point_default_scale: 0.002,
                    point_highlight_selected_scale: 0.0035,
                    point_highlight_unselected_scale: 0.002,
                    tool_ray_radius: 0.005,
                    tool_ray_distance: 0.1,
                    start_select: false,
                    reset_vertex: false,
                    add_selected_to_vec: false,
                    vertex_color_hsv_ratio: 5.0,
                    exit: false,
                    two_cluster_distance_index: [0; 2],
                    filter: CloudFilterConfig {
                        enable: false,
                        filter_height_min: 0,
                        filter_height_max: 0,
                        filter_width_min: 0,
                        filter_width_max: 0,
                        point_num: 0,
                    },
                    operator_mode: 0,
                    operator_run: false,
                })),
                marker_pose: Arc::new(Mutex::new(vec![])),
            }
        });

        // GLOBAL_DATA.get_or_init(move || Arc::new(Mutex::new((    UnsafeSender::new(float_vec_ptr) , float_vec_len))) );
        println!("set crate::GLOBAL_DATA::BUFFER");
    }

    let mut dds_handler: message_handler::MessageHandler =
        message_handler::MessageHandler::new("dds");
    dds_handler.create(args.dds_config.as_str(), *allocator.cfg.get());

    let mut signal = SignalHandler::default();

    let mut app = create_bevy_app([0.5, 0.5, 0.5], [500, 100]);

    setup_shaders_render(&mut app, set_shader_render_demo);

    app.add_systems(Update, update_shader_render_demo);
    app.add_systems(Update, create_egui_windows);

    app.add_systems(Update, raycast_picker);
    app.add_systems(Startup, setup_help_info);
    app.add_systems(Update, plot_marker);

    let mut dds_thread = Thread::default();

    {
        let signal = signal.clone();
        dds_thread = Thread::new(move || {
            while signal.is_run() {
                let enable_dds_update = GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .enable_dds_update;
                let mut dds_msg_count = GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .dds_msg_count;

                if enable_dds_update || dds_msg_count == 0 {
                    let recv_cloud = dds_handler.read_data(c"cloud_sub");
                    for m in recv_cloud.iter() {
                        dds_msg_count += 1;
                        let data: PointCloud2 = PointCloud2::from_ptr(*m);

                        let timestamp_u64 = data.get_stamp();
                        let stamp =
                            OffsetDateTime::from_unix_timestamp_nanos(timestamp_u64 as i128)
                                .unwrap();

                        let current_stamp = OffsetDateTime::now_utc();

                        let stamp_diff = current_stamp - stamp;

                        let stamp_diff_s = stamp_diff.as_seconds_f32();
                        let stamp_local = stamp.to_offset(tz_offset);
                        let stamp_valid = (stamp_diff_s.abs() < 1.0);

                        let frame_id = data.get_frame_id();
                        let cloud_float_vec = data.get_data();

                        let cloud_height_width = data.get_height_width();

                        println!(
                            "recv data [{:?}] at {:?},cloud_float_vec.len : {}",
                            frame_id,
                            stamp,
                            cloud_float_vec.len()
                        );

                        let filter = GLOBAL_DATA.get().unwrap().state.lock().unwrap().filter;

                        if filter.enable {
                            let float_vec_len = (filter.filter_height_max
                                - filter.filter_height_min)
                                * (filter.filter_width_max - filter.filter_width_min)
                                * 3;
                            float_vec = allocator
                                .realloc_as_array::<f32>(float_vec, float_vec_len as usize);

                            info!("clip cloud to float_vec_len {} ", float_vec_len);
                            // use nx_message_center::base::pointcloud_process::perception::pointcloud_clip;
                            pointcloud_clip(
                                cloud_float_vec.as_ptr() as *mut _,
                                cloud_height_width[0] as u64,
                                cloud_height_width[1] as u64,
                                float_vec.as_mut_ptr(),
                                filter.filter_height_min,
                                filter.filter_height_max,
                                filter.filter_width_min,
                                filter.filter_width_max,
                            );

                            {
                                let float_vec_ptr = float_vec.as_ptr();
                                *GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap() =
                                    (UnsafeSender::new(float_vec_ptr), float_vec_len as usize);
                            }
                        } else {
                            float_vec =
                                allocator.realloc_as_array::<f32>(float_vec, cloud_float_vec.len());

                            unsafe {
                                std::ptr::copy(
                                    cloud_float_vec.as_ptr(),
                                    float_vec.as_mut_ptr(),
                                    cloud_float_vec.len(),
                                );
                            }

                            {
                                let float_vec_ptr = float_vec.as_ptr();
                                let float_vec_len = float_vec.len();
                                *GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap() =
                                    (UnsafeSender::new(float_vec_ptr), float_vec_len);
                            }
                        }
                    }
                }
                GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .dds_msg_count = dds_msg_count;

                std::thread::sleep(std::time::Duration::from_millis(100));
            }

            GLOBAL_DATA.get().unwrap().state.lock().unwrap().exit = true;
            if GLOBAL_DATA.get().unwrap().state.lock().unwrap().exit {
                info!("send exit signal");
            }
        });
    }

    app.run();

    signal.stop();
}

fn setup_help_info(mut commands: Commands)
{
    // example instructions
    commands.spawn(
        TextBundle::from_section(
            r##"
Created by ZHANGE.WEI
            "##,
            TextStyle {
                font_size: 20.,
                ..TextStyle::default()
            },
        )
        .with_style(Style {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..Style::default()
        }),
    );
}

fn set_shader_render_demo(mut config: ResMut<ShaderResConfig>)
{
    let cloud_dim = GLOBAL_DATA.get().unwrap().cloud_dim;
    config.enable = true;
    config.number = cloud_dim[0] * cloud_dim[1];
    config.run_count = 0;
    info!("set_shader_render_demo: {:?}", config);
}
fn update_shader_render_demo(
    mut query: Query<&mut InstanceMaterialData>,
    mut config: ResMut<ShaderResConfig>,
)
{
    // info!("update_shader_render_demo : {:?}", config);

    if !config.enable || config.number == 0 || config.run_count == 0 {
        // info!("update_shader_render_demo skip : {:?}", config);

        return;
    }

    //-----
    let mut reset_vertex = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .reset_vertex;

    GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .reset_vertex = false;
    let vertex_color_hsv_ratio = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .vertex_color_hsv_ratio;

    let mut add_selected_to_vec = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .add_selected_to_vec;
    GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .add_selected_to_vec = false;

    let distance_ratio = 360.0 / vertex_color_hsv_ratio;

    let mut point_default_scale = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .point_default_scale;
    let mut point_highlight_selected_scale = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .point_highlight_selected_scale;

    let mut point_highlight_unselected_scale = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .point_highlight_unselected_scale;

    let start_select = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .start_select;

    //----

    {
        if let Some(buffer) = GLOBAL_DATA.get() {
            if let Ok(ref mut mutex) = buffer.cloud_buffer.try_lock() {
                let (ptr, len) = **mutex;

                let vec3_data: &[[f32; 3]] =
                    unsafe { slice::from_raw_parts_mut(*ptr.get() as *mut [f32; 3], len / 3) };
                let vec3_data_len = vec3_data.len();

                for (_, mut data) in query.iter_mut().enumerate() {
                    let mut data = &mut data.0;
                    let data_len = data.len();
                    println!(
                        "update cloud scale, vec3_data_len {:?}, data_len {:?}",
                        vec3_data_len, data_len
                    );

                    if vec3_data_len > data_len {
                        warn!(
                            "vec3_data_len: [{}] > allocate data_len: [{}]",
                            vec3_data_len, data_len
                        );
                        return;
                    }
                    for i in 0..vec3_data_len {
                        let a = vec3_data[i];
                        let b = &mut data[i];
                        b.position.x = a[0];
                        b.position.y = a[1];
                        b.position.z = a[2];
                        let r = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();

                        let h = Color::hsl(r * distance_ratio, 0.5, 0.5).as_rgba();
                        b.color[0] = h.r();
                        b.color[1] = h.g();
                        b.color[2] = h.b();
                        b.color[3] = h.a();
                        b.scale = point_default_scale;
                    }
                    for i in vec3_data_len..data_len {
                        data[i].scale = 0.0;
                    }

                    #[cfg(notuse)]
                    for (i, b) in data.iter_mut().enumerate() {
                        if (i < vec3_data_len) {
                            let a = vec3_data[i];
                            b.position.x = a[0];
                            b.position.y = a[1];
                            b.position.z = a[2];
                            let r = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();

                            let h = Color::hsl(r * distance_ratio, 0.5, 0.5).as_rgba();
                            b.color[0] = h.r();
                            b.color[1] = h.g();
                            b.color[2] = h.b();
                            b.color[3] = h.a();
                            b.scale = point_default_scale;
                        } else {
                            b.scale = 0.0;
                        }
                    }

                    if start_select {
                        for i in 0..vec3_data_len {
                            let a = vec3_data[i];
                            let b = &mut data[i];
                            b.scale = if (b.selected > 0) {
                                point_highlight_selected_scale
                            } else {
                                point_highlight_unselected_scale
                            }
                        }
                        #[cfg(notuse)]
                        for b in data.iter_mut() {
                            b.scale = if (b.selected > 0) {
                                point_highlight_selected_scale
                            } else {
                                point_highlight_unselected_scale
                            }
                        }

                        if add_selected_to_vec {
                            let index_vec: Vec<_> = data
                                .iter()
                                .enumerate()
                                .take(vec3_data_len)
                                .filter_map(|(i, b)| if b.selected > 0 { Some(i) } else { None })
                                .collect();

                            // let index_vec:Vec<_> = data.iter().enumerate().take_while(|(i,b)|{b.selected > 0}).map(|(i,b)| i).collect();
                            // info!("index_vec len: {}, {:?}",index_vec.len(),index_vec);
                            if !index_vec.is_empty() {
                                GLOBAL_DATA
                                    .get()
                                    .unwrap()
                                    .tool_selected_points_index
                                    .lock()
                                    .unwrap()
                                    .push(index_vec);
                                reset_vertex = true;
                            }
                        }

                        if reset_vertex {
                            for b in data.iter_mut() {
                                b.selected = 0;
                            }
                        }
                    }
                }
            } else {
                println!("GLOBAL_DATA try_lock failed");
            }
        } else {
            println!("GLOBAL_DATA empty");
        }
    }
}

fn raycast_picker(
    cursor_ray: Res<CursorRay>,
    mut query: Query<&mut InstanceMaterialData>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    key_input: Res<ButtonInput<KeyCode>>,
    mut camera_ray: ResMut<CameraFocusRay>,
)
{
    // start select

    let start_select = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .start_select;
    let camera_focus = camera_ray.camera_focus;

    let tool_ray_radius = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .tool_ray_radius;

    let tool_ray_distance = GLOBAL_DATA
        .get()
        .unwrap()
        .state
        .lock()
        .unwrap()
        .tool_ray_distance;
    camera_ray.tool_ray_distance = tool_ray_distance;
    if !start_select {
        return;
    }

    {
        if let Some(ray) = **cursor_ray {
            if key_input.pressed(KeyCode::ControlLeft) && key_input.pressed(KeyCode::ShiftLeft)
            // && mouse_button_input.pressed(MouseButton::Left)
            {
                let origin = Point3f {
                    x: ray.origin.x,
                    y: ray.origin.y,
                    z: ray.origin.z,
                };
                let direction = Point3f {
                    x: ray.direction.x,
                    y: ray.direction.y,
                    z: ray.direction.z,
                };
                let camera_focus = Point3f {
                    x: camera_focus[0],
                    y: camera_focus[1],
                    z: camera_focus[2],
                };

                for (_, mut data) in query.iter_mut().enumerate() {
                    let mut data = &mut data.0;

                    let vec3_len = GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap().1;
                    let vec3_len = vec3_len / 3;

                    for i in 0..vec3_len {
                        let v = &mut data[i];
                        let entity = Point3f {
                            x: v.position.x,
                            y: v.position.y,
                            z: v.position.z,
                        };
                        let distance = Point3f::distance_to_line(origin, direction, entity);
                        let distance_to_origin = (entity - camera_focus).length();

                        if (distance < tool_ray_radius && distance_to_origin < tool_ray_distance) {
                            v.selected = 1;
                        }
                    }
                }
            }
        }
    }
}

fn plot_marker(mut gizmos: Gizmos)
{
    let marker_len = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap().len();
    for i in 0..marker_len {
        let marker = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap()[i];
        if marker.enable {
            let mut p1 = [1.0f32, 0.0, 0.0];

            let rect_len = 0.3;

            let mut p2 = [0.0f32; 3];
            let mut plain_points: [f32; 12] = [
                0.0, -rect_len, -rect_len, 0.0, -rect_len, rect_len, 0.0, rect_len, rect_len, 0.0,
                rect_len, -rect_len,
            ];
            let mut plain_points_abs = [0.0; 12];
            pointcloud_transform(
                p1.as_mut_ptr(),
                1,
                p2.as_mut_ptr(),
                0.0,
                0.0,
                0.0,
                marker.pose.roll,
                marker.pose.pitch,
                marker.pose.yaw,
            );
            pointcloud_transform(
                plain_points.as_mut_ptr(),
                4,
                plain_points_abs.as_mut_ptr(),
                marker.pose.tx,
                marker.pose.ty,
                marker.pose.tz,
                marker.pose.roll,
                marker.pose.pitch,
                marker.pose.yaw,
            );

            let [qw, qx, qy, qz] = nx_common::common::transform2d::euler_to_quaternion(
                marker.pose.roll as f64,
                marker.pose.pitch as f64,
                marker.pose.yaw as f64,
            );

            gizmos.ray(
                Vec3 {
                    x: marker.pose.tx,
                    y: marker.pose.ty,
                    z: marker.pose.tz,
                },
                Vec3 {
                    x: p2[0],
                    y: p2[1],
                    z: p2[2],
                },
                Color::Rgba {
                    red: 0.0,
                    green: 1.0,
                    blue: 0.0,
                    alpha: 0.5,
                },
            );

            gizmos.linestrip(
                [
                    Vec3 {
                        x: plain_points_abs[0],
                        y: plain_points_abs[1],
                        z: plain_points_abs[2],
                    },
                    Vec3 {
                        x: plain_points_abs[3],
                        y: plain_points_abs[4],
                        z: plain_points_abs[5],
                    },
                    Vec3 {
                        x: plain_points_abs[6],
                        y: plain_points_abs[7],
                        z: plain_points_abs[8],
                    },
                    Vec3 {
                        x: plain_points_abs[9],
                        y: plain_points_abs[10],
                        z: plain_points_abs[11],
                    },
                    Vec3 {
                        x: plain_points_abs[0],
                        y: plain_points_abs[1],
                        z: plain_points_abs[2],
                    },Vec3 {
                    x: plain_points_abs[6],
                    y: plain_points_abs[7],
                    z: plain_points_abs[8],
                },
                ],
                Color::Rgba {
                    red: 0.0,
                    green: 1.0,
                    blue: 0.0,
                    alpha: 0.5,
                },
            );
        }
    }
}

fn create_egui_windows(mut contexts: EguiContexts)
{
    let ctx = contexts.ctx_mut();
    // Get current context style
    let mut style = (*ctx.style()).clone();

    style.visuals = egui::Visuals::light();

    // Mutate global style with above changes
    ctx.set_style(style);

    egui::Window::new("Selector").resizable(true).show(ctx, |ui| {
        ui.group(|ui| {
            ui.vertical(|ui| {

                ui.collapsing("Instructions", |ui| {
                    ui.label(r##"
Esc : quit
KeyControl + KeyT : toogle camera orbit control
KeyControl + MouseButtonLeft : rotate camera
MouseButtonMiddle : zoom camera
MouseButtonRight : pan camera

Press KeyControl + KeyShift, move mouse to select points.
Only points within tool_ray_radius can be selected.
Click Add to save indexes.
Click Reser to clear current indexes.
                    "##);
                });

                ui.horizontal(|ui| {
                    let mut start_select = GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .start_select;
                    ui.checkbox(&mut start_select, "Enable");
                    GLOBAL_DATA
                        .get()
                        .unwrap()
                        .state
                        .lock()
                        .unwrap()
                        .start_select = start_select;
                    if (start_select) {
                        GLOBAL_DATA
                            .get()
                            .unwrap()
                            .state
                            .lock()
                            .unwrap()
                            .enable_dds_update = false;
                    }

                    if ui.button("Reset").clicked() {
                        GLOBAL_DATA
                            .get()
                            .unwrap()
                            .state
                            .lock()
                            .unwrap()
                            .reset_vertex = true;
                    }

                    if ui.button("Add").clicked() {
                        GLOBAL_DATA
                            .get()
                            .unwrap()
                            .state
                            .lock()
                            .unwrap()
                            .add_selected_to_vec = true;
                        info!("Add is clicked");
                    }
                });

                let mut vertex_color_hsv_ratio = GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .vertex_color_hsv_ratio;
                ui.add(
                    egui::DragValue::new(&mut vertex_color_hsv_ratio)
                        .speed(0.1)
                        .clamp_range(1.0..=15.0)
                        .prefix("vertex_color_hsv_ratio: "),
                );
                GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .vertex_color_hsv_ratio = vertex_color_hsv_ratio;

                //=====
                let mut point_default_scale = GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .point_default_scale;
                ui.add(
                    egui::DragValue::new(&mut point_default_scale)
                        .speed(0.001)
                        .clamp_range(0.001..=0.2)
                        .prefix("scale_default: "),
                );
                GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .point_default_scale = point_default_scale;

                //====
                //=====
                let mut point_highlight_selected_scale = GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .point_highlight_selected_scale;
                ui.add(
                    egui::DragValue::new(&mut point_highlight_selected_scale)
                        .speed(0.001)
                        .clamp_range(0.001..=0.2)
                        .prefix("scale_select: "),
                );
                GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .point_highlight_selected_scale = point_highlight_selected_scale;

                //====
                //=====
                let mut point_highlight_unselected_scale = GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .point_highlight_unselected_scale;
                ui.add(
                    egui::DragValue::new(&mut point_highlight_unselected_scale)
                        .speed(0.001)
                        .clamp_range(0.001..=0.2)
                        .prefix("scale_unselect: "),
                );
                GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .point_highlight_unselected_scale = point_highlight_unselected_scale;

                //====
                let mut tool_ray_radius = GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .tool_ray_radius;
                ui.add(
                    egui::DragValue::new(&mut tool_ray_radius)
                        .speed(0.001)
                        .clamp_range(0.001..=0.1)
                        .prefix("tool_ray_radius: "),
                );
                GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .tool_ray_radius = tool_ray_radius;

                //==tool_ray_distance
                let mut tool_ray_distance = GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .tool_ray_distance;
                ui.add(
                    egui::DragValue::new(&mut tool_ray_distance)
                        .speed(0.001)
                        .clamp_range(0.001..=0.9)
                        .prefix("tool_ray_distance: "),
                );
                GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .tool_ray_distance = tool_ray_distance;
            });
        });
    });

    egui::Window::new("IndexViewer")
        .resizable(true)
        .show(ctx, |ui| {
            let tool_selected_points_index_len = GLOBAL_DATA
                .get()
                .unwrap()
                .tool_selected_points_index
                .lock()
                .unwrap()
                .len();
            ui.label(format!("Count: {}", tool_selected_points_index_len));

            for i in 0..tool_selected_points_index_len {
                ui.push_id(i, |ui| {
                    ui.collapsing("Vec", |ui| {
                        ScrollArea::vertical().show(ui, |ui| {
                            // Add a lot of widgets here.
                            ui.vertical(|ui| {
                                for j in GLOBAL_DATA
                                    .get()
                                    .unwrap()
                                    .tool_selected_points_index
                                    .lock()
                                    .unwrap()[i]
                                    .iter()
                                {
                                    ui.label(format!("{}", j, ));
                                }
                            });
                        });
                    }); // this is fine!
                });
            }
        });

    egui::Window::new("DataReader").resizable(true).show(ctx, |ui| {
        ui.horizontal(|ui| {
            let mut enable_dds_update = GLOBAL_DATA
                .get()
                .unwrap()
                .state
                .lock()
                .unwrap()
                .enable_dds_update;
            ui.checkbox(&mut enable_dds_update, "Enable");
            GLOBAL_DATA
                .get()
                .unwrap()
                .state
                .lock()
                .unwrap()
                .enable_dds_update = enable_dds_update;

            let dds_msg_count = GLOBAL_DATA
                .get()
                .unwrap()
                .state
                .lock()
                .unwrap()
                .dds_msg_count;
            ui.label(format!("Count: {}", dds_msg_count));
        });
    });
    egui::Window::new("Marker").resizable(true).show(ctx, |ui| {
        if ui.button("Add").clicked() {
            GLOBAL_DATA
                .get()
                .unwrap()
                .marker_pose
                .lock()
                .unwrap()
                .push(PoseMarker {
                    pose: Pose {
                        tx: 0.0,
                        ty: 0.0,
                        tz: 0.0,
                        roll: 0.0,
                        pitch: 0.0,
                        yaw: 0.0,
                    },
                    enable: false,
                });
        }
        let marker_pose_len = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap().len();
        for i in 0..marker_pose_len {
            ui.horizontal(|ui| {
                let mut marker = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap()[i];
                ui.checkbox(&mut marker.enable, "Enable");

                ui.add(
                    egui::DragValue::new(&mut marker.pose.tx)
                        .speed(0.001)
                        .clamp_range(-5.0..=5.0)
                        .prefix("tx: "),
                );
                ui.add(
                    egui::DragValue::new(&mut marker.pose.ty)
                        .speed(0.001)
                        .clamp_range(-5.0..=5.0)
                        .prefix("ty: "),
                );
                ui.add(
                    egui::DragValue::new(&mut marker.pose.tz)
                        .speed(0.001)
                        .clamp_range(-5.0..=5.0)
                        .prefix("tz: "),
                );

                ui.add(
                    egui::DragValue::new(&mut marker.pose.roll)
                        .speed(0.001)
                        .clamp_range(-PI..=PI)
                        .prefix("roll: "),
                );
                ui.add(
                    egui::DragValue::new(&mut marker.pose.pitch)
                        .speed(0.001)
                        .clamp_range(-PI..=PI)
                        .prefix("pitch: "),
                );
                ui.add(
                    egui::DragValue::new(&mut marker.pose.yaw)
                        .speed(0.001)
                        .clamp_range(-PI..=PI)
                        .prefix("yaw: "),
                );

                GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap()[i] = marker;
            });
        }
    });

    egui::Window::new("Filter").resizable(true).show(ctx, |ui| {
        let [height, width] = GLOBAL_DATA.get().unwrap().cloud_dim;
        //=====
        let mut filter_height_min = GLOBAL_DATA
            .get()
            .unwrap()
            .state
            .lock()
            .unwrap()
            .filter
            .filter_height_min;

        ui.add(Slider::new(&mut filter_height_min, 0..=height as u64).text("filter_height_min"));
        GLOBAL_DATA
            .get()
            .unwrap()
            .state
            .lock()
            .unwrap()
            .filter
            .filter_height_min = filter_height_min;

        //====
        //=====
        let mut filter_height_max = GLOBAL_DATA
            .get()
            .unwrap()
            .state
            .lock()
            .unwrap()
            .filter
            .filter_height_max;

        ui.add(
            Slider::new(&mut filter_height_max, filter_height_min..=height as u64)
                .text("filter_height_max"),
        );
        GLOBAL_DATA
            .get()
            .unwrap()
            .state
            .lock()
            .unwrap()
            .filter
            .filter_height_max = filter_height_max;

        //====
        //=====
        let mut filter_width_min = GLOBAL_DATA
            .get()
            .unwrap()
            .state
            .lock()
            .unwrap()
            .filter
            .filter_width_min;

        ui.add(Slider::new(&mut filter_width_min, 0..=width as u64).text("filter_width_min"));
        GLOBAL_DATA
            .get()
            .unwrap()
            .state
            .lock()
            .unwrap()
            .filter
            .filter_width_min = filter_width_min;

        //====
        //=====
        let mut filter_width_max = GLOBAL_DATA
            .get()
            .unwrap()
            .state
            .lock()
            .unwrap()
            .filter
            .filter_width_max;

        ui.add(
            Slider::new(&mut filter_width_max, filter_width_min..=width as u64)
                .text("filter_width_max"),
        );
        GLOBAL_DATA
            .get()
            .unwrap()
            .state
            .lock()
            .unwrap()
            .filter
            .filter_width_max = filter_width_max;

        //====
        let range_ok =
            (filter_width_min < filter_width_max && filter_height_min < filter_height_max);

        if range_ok {
            let mut enable = GLOBAL_DATA
                .get()
                .unwrap()
                .state
                .lock()
                .unwrap()
                .filter
                .enable;
            ui.checkbox(&mut enable, "Enable");
            GLOBAL_DATA
                .get()
                .unwrap()
                .state
                .lock()
                .unwrap()
                .filter
                .enable = enable;
        } else {
            GLOBAL_DATA
                .get()
                .unwrap()
                .state
                .lock()
                .unwrap()
                .filter
                .enable = false;
        }
    });

    egui::Window::new("Operator")
        .resizable(true)
        .show(ctx, |ui| {

            ui.collapsing("Instructions", |ui| {
                ui.label("Choose target and operator");
            });

            let tool_selected_points_index_len = GLOBAL_DATA
                .get()
                .unwrap()
                .tool_selected_points_index
                .lock()
                .unwrap()
                .len();


            let marker_pose_len = GLOBAL_DATA.get().unwrap().marker_pose.lock().unwrap().len();

            if (tool_selected_points_index_len == 0){
                ui.label("Please select points with Selector!");

                return;
            }


            let mut operator_panel = GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_mode;

            ui.separator();
            ui.horizontal(|ui| {
                ui.selectable_value(&mut operator_panel,  1 , "cluster");
                ui.selectable_value(&mut operator_panel,  2 , "marker");
                ui.selectable_value(&mut operator_panel,  3 , "norm");

            });
            ui.separator();
            if (operator_panel !=  GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_mode  ){
                GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_run = false;;

                GLOBAL_DATA
                    .get()
                    .unwrap()
                    .state
                    .lock()
                    .unwrap()
                    .two_cluster_distance_index = [0, 0];
            }
            GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_mode = operator_panel;

            let mut operator_run = GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_run;


            let [mut from_index, mut to_index] = GLOBAL_DATA
                .get()
                .unwrap()
                .state
                .lock()
                .unwrap()
                .two_cluster_distance_index;

            match operator_panel{
                1 => {




                     {

                        ComboBox::from_label("from_index")
                            .selected_text(from_index.to_string())
                            .show_ui(ui, |ui| {
                                for i in 0..tool_selected_points_index_len {
                                    ui.selectable_value(&mut from_index, i, i.to_string());
                                }
                            });

                        ComboBox::from_label("to_index")
                            .selected_text(to_index.to_string())
                            .show_ui(ui, |ui| {
                                for i in 0..tool_selected_points_index_len {
                                    ui.selectable_value(&mut to_index, i, i.to_string());
                                }
                            });


                        if (from_index != to_index) {
                            let mut from_center = [0.0, 0.0, 0.0];
                            let mut to_center = [0.0, 0.0, 0.0];

                            ui.checkbox(&mut operator_run, "operator_run");

                            // if ui.button("compute distance").clicked() {}
                            // let mut from_sum =  GLOBAL_DATA.get().unwrap().tool_selected_points_index.lock().unwrap()[from_index].iter().reduce(|acc, e| acc + e).unwrap();
                        }else{
                            operator_run = false;
                        }
                        GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_run = operator_run;

                    }
                },
                2 => {
                  if marker_pose_len  == 0 {
                      ui.label("Please set marker with Marker!");

                  }else{
                      ComboBox::from_label("from_index")
                          .selected_text(from_index.to_string())
                          .show_ui(ui, |ui| {
                              for i in 0..tool_selected_points_index_len {
                                  ui.selectable_value(&mut from_index, i, i.to_string());
                              }
                          });

                      ComboBox::from_label("to_index")
                          .selected_text(to_index.to_string())
                          .show_ui(ui, |ui| {
                              for i in 0..marker_pose_len {
                                  ui.selectable_value(&mut to_index, i, i.to_string());
                              }
                          });

                      ui.checkbox(&mut operator_run, "operator_run");
                      GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_run = operator_run;


                  }

                },
                3 => {
                    ComboBox::from_label("from_index")
                        .selected_text(from_index.to_string())
                        .show_ui(ui, |ui| {
                            for i in 0..tool_selected_points_index_len {
                                ui.selectable_value(&mut from_index, i, i.to_string());
                            }
                        });
                    ui.checkbox(&mut operator_run, "operator_run");
                    GLOBAL_DATA.get().unwrap().state.lock().unwrap().operator_run = operator_run;

                },
                _ => {

                }
            }

            GLOBAL_DATA
                .get()
                .unwrap()
                .state
                .lock()
                .unwrap()
                .two_cluster_distance_index = [from_index, to_index];






        });
}
