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
use std::sync::atomic::{AtomicBool, Ordering};
use bevy::app::Update;
use bevy::log::info;
use bevy::prelude::{ButtonInput, Color, KeyCode, MouseButton, Query, Res, ResMut};
use bevy_egui::{egui, EguiContexts};
use bevy_egui::egui::{ComboBox, ScrollArea};
use bevy_mod_raycast::CursorRay;
// argument
use clap::Parser;
use nx_gui::app_builder::{CameraFocusRay, create_bevy_app};
use nx_gui::shaders::{InstanceMaterialData, setup_shaders_render, ShaderResConfig};


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
struct SimpleState{
    pub enable_dds_update:bool,
    pub dds_msg_count:usize,
    // select point scale need be larger
    pub point_default_scale: f32,
    pub point_highlight_selected_scale: f32,
    pub point_highlight_unselected_scale: f32,
    pub tool_ray_radius : f32,
    pub tool_ray_distance : f32,


    pub start_select: bool,
    pub reset_vertex_select_flag : bool,
    pub add_selected_to_vec : bool,

    pub vertex_color_hsv_ratio: f32,

    pub exit:bool,

    pub two_cluster_distance_index: [usize;2],

    camera_focus: [f32;3]
}

struct StaticSharedData{

    pub cloud_buffer: Arc<Mutex<(UnsafeSender<*const f32>, usize)>>,
    pub cloud_dim: [usize;2],
    pub state: Arc<Mutex<SimpleState>>,

    // col and row filter
    // x,y,z range filter
    // multiple pick group 0
    // multiple pick group 1
    // multiple pick group 2
    // multiple pick group 3
    pub tool_selected_points_index: Arc<Mutex<Vec<Vec<usize> >  >>,
}

static GLOBAL_DATA: OnceLock<StaticSharedData> = OnceLock::new();


fn main(){

    let tz_offset = nx_common::common::time::get_timezone_offset();
    let memory_pool_base = nx_common::common::memory::get_next_aligned_addr(
        unsafe { TA_BUFFER.as_ptr() } as *mut _,
        8,
    );
    let allocator = tiny_alloc::TinyAlloc::new(memory_pool_base, TA_BUFFER_SIZE - 8, 512, 16, 8);

    let args = Args::parse();


    let mut float_vec = allocator.alloc_as_array::<f32>((args.width * args.height*3) as usize);

    {
        let float_vec_ptr = float_vec.as_ptr();
        let float_vec_len = float_vec.len();
        GLOBAL_DATA.get_or_init(move || {
            println!("set crate::GLOBAL_DATA::BUFFER");
            StaticSharedData{
                cloud_dim: [args.height, args.width],

                cloud_buffer: Arc::new(Mutex::new((
                    UnsafeSender::new(float_vec_ptr),
                    float_vec_len,
                ))),
                tool_selected_points_index:  Arc::new(Mutex::new(vec![])),
                state: Arc::new(Mutex::new(SimpleState {
                    enable_dds_update: true,
                    dds_msg_count: 0,
                    point_default_scale: 0.005,
                    point_highlight_selected_scale: 0.005,
                    point_highlight_unselected_scale: 0.002,
                    tool_ray_radius: 0.005,
                    tool_ray_distance: 0.1,
                    start_select: false,
                    reset_vertex_select_flag: false,
                    add_selected_to_vec: false,
                    vertex_color_hsv_ratio: 5.0,
                    exit: false,
                    two_cluster_distance_index: [0;2],
                    camera_focus: [0.0;3],
                })),
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

    app.add_systems(Update,raycast_picker);



    let mut dds_thread = Thread::default();

    {
        let signal = signal.clone();
        dds_thread = Thread::new(move || {
            while signal.is_run() {

                let  enable_dds_update =  GLOBAL_DATA.get().unwrap().state.lock().unwrap().enable_dds_update;
                let  mut dds_msg_count = GLOBAL_DATA.get().unwrap().state.lock().unwrap().dds_msg_count;


                if enable_dds_update || dds_msg_count == 0 {

                    let recv_cloud = dds_handler.read_data(c"cloud_sub");
                    for m in recv_cloud.iter() {
                        dds_msg_count += 1;
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

                        unsafe{
                            std::ptr::copy(cloud_float_vec.as_ptr(), float_vec.as_mut_ptr(), cloud_float_vec.len() );
                        }

                        // for (i, j) in izip!(cloud_float_vec.iter(), float_vec.iter_mut()) {
                        //     *j = if (i.is_finite()) { *i } else { 0.0 };
                        // }


                        // println!("cloud_float_vec: {:?}", cloud_float_vec);
                        // println!("float_vec: {:?}", float_vec);

                        {
                            let float_vec_ptr = float_vec.as_ptr();
                            let float_vec_len = float_vec.len();
                            *GLOBAL_DATA.get().unwrap().cloud_buffer.lock().unwrap() =
                                (UnsafeSender::new(float_vec_ptr), float_vec_len);
                        }
                    }

                }
                GLOBAL_DATA.get().unwrap().state.lock().unwrap().dds_msg_count = dds_msg_count;

                std::thread::sleep(std::time::Duration::from_millis(100));
            }

            GLOBAL_DATA.get().unwrap().state.lock().unwrap().exit = true;
            if  GLOBAL_DATA.get().unwrap().state.lock().unwrap().exit{
                info!("send exit signal");
            }

        });
    }

    app.run();

    signal.stop();


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
){
    // info!("update_shader_render_demo : {:?}", config);

    if !config.enable || config.number ==0 || config.run_count == 0{
        // info!("update_shader_render_demo skip : {:?}", config);

        return;
    }

    //-----
    let  mut reset_vertex_select_flag = GLOBAL_DATA.get().unwrap().state.lock().unwrap().reset_vertex_select_flag;

    GLOBAL_DATA.get().unwrap().state.lock().unwrap().reset_vertex_select_flag = false;
    let vertex_color_hsv_ratio= GLOBAL_DATA.get().unwrap().state.lock().unwrap().vertex_color_hsv_ratio;

    let  mut add_selected_to_vec = GLOBAL_DATA.get().unwrap().state.lock().unwrap().add_selected_to_vec;
    GLOBAL_DATA.get().unwrap().state.lock().unwrap().add_selected_to_vec = false;

    let distance_ratio = 360.0/vertex_color_hsv_ratio;

    let mut point_default_scale  = GLOBAL_DATA.get().unwrap().state.lock().unwrap().point_default_scale;
    let mut point_highlight_selected_scale  = GLOBAL_DATA.get().unwrap().state.lock().unwrap().point_highlight_selected_scale;

    let mut point_highlight_unselected_scale  = GLOBAL_DATA.get().unwrap().state.lock().unwrap().point_highlight_unselected_scale;

    let  start_select = GLOBAL_DATA.get().unwrap().state.lock().unwrap().start_select;

    //----


    {
        if let Some(buffer) = GLOBAL_DATA.get() {
            if let Ok(ref mut mutex) = buffer.cloud_buffer.try_lock() {
                let (ptr, len) = **mutex;

                let vec3_data: &[[f32; 3]] =
                    unsafe { slice::from_raw_parts_mut(*ptr.get() as *mut [f32; 3], len / 3) };
                // println!("vec3_data {:?}, {:?}", vec3_data.len(), vec3_data);

                for (_, mut data) in query.iter_mut().enumerate() {
                    let mut data = &mut data.0;

                    for (i, b) in data.iter_mut().enumerate() {

                        let a = vec3_data[i];
                        b.position.x = a[0];
                        b.position.y = a[1];
                        b.position.z = a[2];
                        let r = (a[0]*a[0] + a[1]*a[1] + a[2]*a[2] ).sqrt();

                        let h = Color::hsl(r*distance_ratio,0.5,0.5).as_rgba();
                        b.color[0] = h.r();
                        b.color[1] = h.g();
                        b.color[2] = h.b();
                        b.color[3] = h.a();
                        b.scale = point_default_scale;

                    }

                    if start_select{
                        for b in data.iter_mut(){
                            b.scale  = if(b.selected > 0) {
                                point_highlight_selected_scale

                            }else {
                                point_highlight_unselected_scale
                            }
                        }

                        if add_selected_to_vec{
                            let index_vec:Vec<_> =  data.iter().enumerate().filter_map(|(i,b)|{
                                if b.selected > 0{
                                    Some(i)
                                }else {
                                    None
                                }
                            }).collect();

                            // let index_vec:Vec<_> = data.iter().enumerate().take_while(|(i,b)|{b.selected > 0}).map(|(i,b)| i).collect();
                            // info!("index_vec len: {}, {:?}",index_vec.len(),index_vec);
                            if !index_vec.is_empty(){
                                GLOBAL_DATA.get().unwrap().tool_selected_points_index.lock().unwrap().push(index_vec);
                                reset_vertex_select_flag = true;
                            }
                        }

                        if reset_vertex_select_flag{
                            for b in data.iter_mut(){
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


fn raycast_picker(cursor_ray: Res<CursorRay>,  mut query: Query<&mut InstanceMaterialData>, mouse_button_input: Res<ButtonInput<MouseButton>>,
           key_input: Res<ButtonInput<KeyCode>>,
           mut camera_ray: ResMut<CameraFocusRay>,)
{


    // start select

    let start_select =  GLOBAL_DATA.get().unwrap().state.lock().unwrap().start_select;
    let camera_focus =  camera_ray.camera_focus;

    if !start_select{
        return;
    }
    let tool_ray_radius =  GLOBAL_DATA.get().unwrap().state.lock().unwrap().tool_ray_radius;

    let tool_ray_distance =  GLOBAL_DATA.get().unwrap().state.lock().unwrap().tool_ray_distance;
    camera_ray.tool_ray_distance = tool_ray_distance;

        {

        if let Some(ray) = **cursor_ray {

            if key_input.pressed(KeyCode::ControlLeft) && key_input.pressed(KeyCode::ShiftLeft)
            // && mouse_button_input.pressed(MouseButton::Left)

            {
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
                let camera_focus = Point3f{
                    x : camera_focus[0],
                    y : camera_focus[1],
                    z : camera_focus[2],

                };

                for (_, mut data) in query.iter_mut().enumerate() {

                    let mut data = &mut data.0;

                    for (i, v) in data.iter_mut().enumerate() {

                        let entity =  Point3f{
                            x : v.position.x,
                            y : v.position.y,
                            z : v.position.z,

                        };
                        let distance = Point3f::distance_to_line(origin,direction, entity);
                        let distance_to_origin = (entity - camera_focus).length();

                        if(distance < tool_ray_radius && distance_to_origin < tool_ray_distance){
                            v.selected =1;
                        }
                    }
                }
            }


        }
    }


}

fn create_egui_windows(mut contexts: EguiContexts)
{

    let ctx = contexts.ctx_mut();
    // Get current context style
    let mut style = (*ctx.style()).clone();

    style.visuals = egui::Visuals::light();
// Redefine text_styles
//     style. text_styles = [
//         (Heading, FontId::new(30.0, Proportional)),
//         (Name("Heading2".into()), FontId::new(25.0, Proportional)),
//         (Name("Context".into()), FontId::new(23.0, Proportional)),
//         (Body, FontId::new(18.0, Proportional)),
//         (Monospace, FontId::new(14.0, Proportional)),
//         (Button, FontId::new(14.0, Proportional)),
//         (Small, FontId::new(10.0, Proportional)),
//     ].into();

// Mutate global style with above changes
    ctx. set_style(style);


    egui::Window::new("Select").resizable(true).show(ctx, |ui| {



        ui.group(|ui| {
            ui.vertical(|ui| {

                ui.horizontal(|ui|{
                    let  mut start_select =   GLOBAL_DATA.get().unwrap().state.lock().unwrap().start_select ;
                    ui.checkbox( &mut start_select, "Enable", );
                    GLOBAL_DATA.get().unwrap().state.lock().unwrap().start_select = start_select;
                    if(start_select){
                        GLOBAL_DATA.get().unwrap().state.lock().unwrap().enable_dds_update = false;

                    }


                    if ui.button("Reset").clicked(){
                        GLOBAL_DATA.get().unwrap().state.lock().unwrap().reset_vertex_select_flag = true;
                    }

                    if ui.button("Add").clicked(){
                        GLOBAL_DATA.get().unwrap().state.lock().unwrap().add_selected_to_vec = true;
                        info!("Add is clicked");
                    }
                });


                let mut vertex_color_hsv_ratio  = GLOBAL_DATA.get().unwrap().state.lock().unwrap().vertex_color_hsv_ratio;
                ui.add(
                    egui::DragValue::new(&mut vertex_color_hsv_ratio)
                        .speed(0.1)
                        .clamp_range(1.0..=15.0)
                        .prefix("vertex_color_hsv_ratio: "),
                );
                GLOBAL_DATA.get().unwrap().state.lock().unwrap().vertex_color_hsv_ratio = vertex_color_hsv_ratio;

                //=====
                let mut point_default_scale  = GLOBAL_DATA.get().unwrap().state.lock().unwrap().point_default_scale;
                ui.add(
                    egui::DragValue::new(&mut point_default_scale)
                        .speed(0.001)
                        .clamp_range(0.001..=0.2)
                        .prefix("scale_default: "),
                );
                GLOBAL_DATA.get().unwrap().state.lock().unwrap().point_default_scale = point_default_scale;

                //====
                //=====
                let mut point_highlight_selected_scale  = GLOBAL_DATA.get().unwrap().state.lock().unwrap().point_highlight_selected_scale;
                ui.add(
                    egui::DragValue::new(&mut point_highlight_selected_scale)
                        .speed(0.001)
                        .clamp_range(0.001..=0.2)
                        .prefix("scale_select: "),
                );
                GLOBAL_DATA.get().unwrap().state.lock().unwrap().point_highlight_selected_scale = point_highlight_selected_scale;

                //====
                //=====
                let mut point_highlight_unselected_scale  = GLOBAL_DATA.get().unwrap().state.lock().unwrap().point_highlight_unselected_scale;
                ui.add(
                    egui::DragValue::new(&mut point_highlight_unselected_scale)
                        .speed(0.001)
                        .clamp_range(0.001..=0.2)
                        .prefix("scale_unselect: "),
                );
                GLOBAL_DATA.get().unwrap().state.lock().unwrap().point_highlight_unselected_scale = point_highlight_unselected_scale;

                //====
                let mut tool_ray_radius  = GLOBAL_DATA.get().unwrap().state.lock().unwrap().tool_ray_radius;
                ui.add(
                    egui::DragValue::new(&mut tool_ray_radius)
                        .speed(0.001)
                        .clamp_range(0.001..=0.1)
                        .prefix("tool_ray_radius: "),
                );
                GLOBAL_DATA.get().unwrap().state.lock().unwrap().tool_ray_radius = tool_ray_radius;

                //==tool_ray_distance
                let mut tool_ray_distance  = GLOBAL_DATA.get().unwrap().state.lock().unwrap().tool_ray_distance;
                ui.add(
                    egui::DragValue::new(&mut tool_ray_distance)
                        .speed(0.001)
                        .clamp_range(0.001..=0.9)
                        .prefix("tool_ray_distance: "),
                );
                GLOBAL_DATA.get().unwrap().state.lock().unwrap().tool_ray_distance = tool_ray_distance;


            });
        });

    });

    egui::Window::new("IndexViewer").resizable(true).show(ctx, |ui| {


        let tool_selected_points_index_len = GLOBAL_DATA.get().unwrap().tool_selected_points_index.lock().unwrap().len();
        ui.label(format!("Count: {}", tool_selected_points_index_len));



        for i in 0..tool_selected_points_index_len{
            ui. push_id(i, |ui| {
                ui. collapsing("Vec", |ui| {
                    ScrollArea::horizontal().show(ui, |ui| {
                        // Add a lot of widgets here.
                        ui.horizontal(|ui|{
                            for j in GLOBAL_DATA.get().unwrap().tool_selected_points_index.lock().unwrap()[i].iter(){
                                ui.label(format!("{}", j));
                            }
                        });
                    });

                }); // this is fine!

            });


        }


    });

    egui::Window::new("DDS").resizable(true).show(ctx, |ui| {


        ui.horizontal(|ui| {

            let  mut enable_dds_update =   GLOBAL_DATA.get().unwrap().state.lock().unwrap().enable_dds_update ;
            ui.checkbox( &mut enable_dds_update, "Enable", );
            GLOBAL_DATA.get().unwrap().state.lock().unwrap().enable_dds_update = enable_dds_update;


            let dds_msg_count = GLOBAL_DATA.get().unwrap().state.lock().unwrap().dds_msg_count ;
            ui.label(format!("Count: {}", dds_msg_count));
        });

    });

    egui::Window::new("Transform").resizable(true).show(ctx, |ui|{

    });

    egui::Window::new("TwoClusterDistance").resizable(true).show(ctx, |ui|{

        let tool_selected_points_index_len = GLOBAL_DATA.get().unwrap().tool_selected_points_index.lock().unwrap().len();

        if(tool_selected_points_index_len > 0){
            let  [mut from_index,mut to_index ] = GLOBAL_DATA.get().unwrap().state
                .lock().unwrap().two_cluster_distance_index;
            ComboBox::from_label("from_index")
                .selected_text(from_index.to_string())
                .show_ui(ui, |ui| {
                    for i in  0.. tool_selected_points_index_len {
                        ui.selectable_value(&mut from_index, i, i.to_string());
                    }
                });

            ComboBox::from_label("to_index")
                .selected_text(to_index.to_string())
                .show_ui(ui, |ui| {
                    for i in  0.. tool_selected_points_index_len {
                        ui.selectable_value(&mut to_index, i, i.to_string());
                    }
                });
            GLOBAL_DATA.get().unwrap().state
                .lock().unwrap().two_cluster_distance_index = [from_index,to_index];

            if(from_index != to_index){
                let mut from_center = [0.0,0.0,0.0];
                let mut to_center = [0.0,0.0,0.0];

                if ui.button("compute distance").clicked(){

                }
                // let mut from_sum =  GLOBAL_DATA.get().unwrap().tool_selected_points_index.lock().unwrap()[from_index].iter().reduce(|acc, e| acc + e).unwrap();

            }


        }





    });
}