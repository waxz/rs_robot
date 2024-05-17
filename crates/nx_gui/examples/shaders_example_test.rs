use bevy::app::Startup;
use bevy::prelude::{
    info, Assets, Color, Commands, Cuboid, IntoSystemConfigs, Mesh, Query, Res, ResMut, Resource,
    SpatialBundle, StandardMaterial, Time, Update, Vec3,
};
use bevy::render::view::NoFrustumCulling;
use bevy_egui::{egui, EguiContexts};
use itertools::Itertools;
use nx_common::common::signal_handler::SignalHandler;
use nx_common::common::thread::{get_current_cpu, Thread};
use nx_gui::app_builder::create_bevy_app;
use nx_gui::shaders::{
    create_render_resource, setup_shaders_render, InstanceData, InstanceMaterialData,
    ShaderResConfig,
};

fn main()
{
    let t1 = Thread::new(|| {
        let mut signal = SignalHandler::default();
        while signal.is_run() {
            println!("run {:?}", get_current_cpu());

            std::thread::sleep(std::time::Duration::from_millis(1000));
        }
        println!("exit {:?}", get_current_cpu());
    });
    let t2 = Thread::new(|| {
        let mut signal = SignalHandler::default();
        while signal.is_run() {
            println!("run {:?}", get_current_cpu());

            std::thread::sleep(std::time::Duration::from_millis(1000));
        }
        println!("exit {:?}", get_current_cpu());
    });

    let mut app = create_bevy_app([0.5, 0.5, 0.5], [500, 100]);

    setup_shaders_render(&mut app, set_shader_render_demo);

    app.add_systems(Update, update_shader_render_demo);

    app.run();
    println!("exit {:?}", get_current_cpu());
}

//----

fn set_shader_render_demo(mut config: ResMut<ShaderResConfig>)
{
    config.enable = true;
    config.static_point_num = 100 * 100 * 100;
    config.run_count = 0;
    info!("set_shader_render_demo: {:?}", config);
}

fn update_shader_render_demo(
    mut query: Query<&mut InstanceMaterialData>,
    mut config: ResMut<ShaderResConfig>,
)
{
    info!("update_shader_render_demo : {:?}", config);

    if config.enable && config.static_point_num == 100 * 100 * 100 {
        let extent = 10.0 / 2.0;

        let jump = extent / 100 as f32;

        {
            for (_, mut data) in query.iter_mut().enumerate() {
                let mut data = &mut data.0;

                if config.run_count == 1 {
                    for x in 0..100 {
                        for y in 0..100 {
                            for z in 0..100 {
                                let xf = x as f32 * jump - 0.5 * extent;
                                let yf = y as f32 * jump - 0.5 * extent;
                                let zf = z as f32 * jump - 0.5 * extent;

                                let index = x * 10000 + y * 100 + z;
                                let p = &mut data[index];
                                p.position = Vec3 {
                                    x: xf,
                                    y: yf,
                                    z: zf,
                                };
                                p.color = [
                                    if (x % 3 == 0) { 0.0 } else { 1.0 },
                                    if (y % 4 == 0) { 0.0 } else { 1.0 },
                                    if (z % 5 == 0) { 0.0 } else { 1.0 },
                                    0.0,
                                ];
                                p.scale = 0.01;
                                p.selected = 0;
                            }
                        }
                    }
                } else {
                    let rotate = 0.05;

                    for (i, v) in data.iter_mut().enumerate() {
                        let r = (v.position.x * v.position.x + v.position.y * v.position.y).sqrt();
                        let a = v.position.y.atan2(v.position.x);
                        v.position.x = r * (a + rotate).cos();
                        v.position.y = r * (a + rotate).sin();
                    }
                }
                config.run_count += 1;
            }
        }
    }
}
