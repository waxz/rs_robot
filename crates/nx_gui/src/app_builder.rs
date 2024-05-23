use nx_common::common::signal_handler::SignalHandler;
use std::f32::consts::PI;
//BEVY
use bevy_mod_raycast::prelude::*;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin, TouchControls};

use bevy::app::AppExit;
use bevy::pbr::CascadeShadowConfigBuilder;
use bevy::render::camera::RenderTarget;
use bevy::window::WindowRef;
use bevy::winit::WinitWindows;
use bevy::{
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
    utils::Duration,
    window::{PresentMode, RequestRedraw, WindowPlugin},
    winit::WinitSettings,
};
// use bevy::ui::AlignItems::Default;
use bevy_egui::egui::FontFamily::Proportional;
use bevy_egui::egui::TextStyle::{Body, Button, Heading, Monospace, Name, Small};
use bevy_egui::egui::{CollapsingHeader, ComboBox, RichText, ScrollArea, Ui};
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use bytemuck::{Pod, Zeroable};

pub fn create_bevy_app(bg_color: [f32; 3], winit_wait_ms: [u64; 2]) -> App
{
    let mut app = App::new();
    app.insert_resource(ClearColor(Color::rgb(
        bg_color[0],
        bg_color[1],
        bg_color[2],
    )))
    .insert_resource(WinitSettings {
        // focused_mode: bevy::winit::UpdateMode::Continuous,
        focused_mode: bevy::winit::UpdateMode::ReactiveLowPower {
            wait: Duration::from_millis(winit_wait_ms[0]),
        },

        unfocused_mode: bevy::winit::UpdateMode::ReactiveLowPower {
            wait: Duration::from_millis(winit_wait_ms[1]),
        },
    })
    .add_plugins((
        DefaultPlugins,
        PanOrbitCameraPlugin,
        EguiPlugin,
        DefaultRaycastingPlugin,
    ))
    .add_systems(Update, bevy::window::close_on_esc)
    .add_systems(Startup, setup_window_camera)
    .add_systems(Update, camera_keyboard_controls)
    .add_systems(Update, window_exit);

    app
}

// This is how you can change config at runtime.
// Press 'T' to toggle the camera controls.

fn setup_window_camera(mut commands: Commands)
{
    //
    commands.insert_resource(WindowState {
        exit: false,
        signal: Default::default(),
    });

    commands.insert_resource(CameraFocusRay {
        tool_ray_distance: 0.05,
        camera_focus: [0.0, 0.0, 0.0],
    });

    // Camera
    commands.spawn((
        Camera3dBundle {
            // transform: Transform::from_translation(Vec3::new(1., 1., 1.0)),
            transform: Transform::from_xyz(1.0, 1.0, 1.0).looking_at(Vec3::ZERO, Vec3::X),
            ..default()
        },
        PanOrbitCamera {
            target_focus: Vec3::ZERO,
            target_yaw: PI,
            target_pitch: PI,
            radius: Some(10.0),
            yaw_upper_limit: Some(10.0),
            yaw_lower_limit: Some(-10.0),
            pitch_lower_limit: Some(-10.0),
            pitch_upper_limit: Some(10.0),
            // modifier_pan: Some(KeyCode::ControlLeft),
            modifier_orbit: Some(KeyCode::ControlLeft),
            is_upside_down: true,
            allow_upside_down: true,
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

    #[cfg(second_window)]
    {
        // Spawn a second window
        let second_window = commands
            .spawn(Window {
                title: "Second window".to_owned(),
                ..default()
            })
            .id();

        // second window camera
        commands.spawn((
            Camera3dBundle {
                transform: Transform::from_translation(Vec3::new(5.0, 1.5, 7.0)),
                camera: Camera {
                    target: RenderTarget::Window(WindowRef::Entity(second_window)),
                    ..default()
                },
                ..default()
            },
            PanOrbitCamera::default(),
        ));
    }
}

//---------------------------

#[derive(Resource)]
pub struct CameraFocusRay
{
    pub tool_ray_distance: f32,
    pub camera_focus: [f32; 3],
}
#[derive(Resource)]
pub struct WindowState
{
    exit: bool,
    signal: SignalHandler,
}

fn camera_keyboard_controls(
    time: Res<Time>,
    key_input: Res<ButtonInput<KeyCode>>,
    mut pan_orbit_query: Query<(&mut PanOrbitCamera, &mut Transform)>,
    mut gizmos: Gizmos,
    mut camera_ray: ResMut<CameraFocusRay>,
)
{
    gizmos.ray(Vec3::new(0., 0., 0.), Vec3::new(1., 0., 0.), Color::RED);
    gizmos.ray(Vec3::new(0., 0., 0.), Vec3::new(0., 1., 0.), Color::GREEN);
    gizmos.ray(Vec3::new(0., 0., 0.), Vec3::new(0., 0., 1.), Color::BLUE);

    let tool_ray_distance = camera_ray.tool_ray_distance;

    for (mut pan_orbit, mut transform) in pan_orbit_query.iter_mut() {
        gizmos.ray(
            pan_orbit.target_focus,
            Vec3::from([tool_ray_distance, 0.0, 0.0]),
            Color::RED,
        );
        gizmos.ray(
            pan_orbit.target_focus,
            Vec3::from([0.0, tool_ray_distance, 0.0]),
            Color::GREEN,
        );
        gizmos.ray(
            pan_orbit.target_focus,
            Vec3::from([0.0, 0.0, tool_ray_distance]),
            Color::BLUE,
        );
        gizmos
            .sphere(
                pan_orbit.target_focus,
                Quat::IDENTITY,
                tool_ray_distance,
                Color::BLACK,
            )
            .circle_segments(16);

        camera_ray.camera_focus = [pan_orbit.focus.x, pan_orbit.focus.y, pan_orbit.focus.z];

        if key_input.pressed(KeyCode::ControlLeft) {
            if key_input.just_pressed(KeyCode::KeyT) {
                pan_orbit.enabled = !pan_orbit.enabled;
                return;
            }

            // // Jump focus point 1m using Ctrl+Shift + Arrows
            // if key_input.pressed(KeyCode::ShiftLeft) {
            //
            //     //    z+  y+
            //     //x-  []  x+
            //     //y-  z-
            //     //reset
            //
            //     let step = 3.2;
            //     let mut new_transform = *transform ;
            //
            //
            //     if key_input.just_pressed(KeyCode::Numpad1) {
            //         new_transform.rotation *= Quat::from_rotation_y(-step);
            //     }
            //     if key_input.just_pressed(KeyCode::Numpad9) {
            //         new_transform.rotation *= Quat::from_rotation_y(step);
            //     }
            //
            //     if key_input.just_pressed(KeyCode::Numpad2) {
            //         new_transform.rotation *= Quat::from_rotation_z(-step);
            //
            //     }
            //     if key_input.just_pressed(KeyCode::Numpad8) {
            //         new_transform.rotation *= Quat::from_rotation_z(step);
            //
            //     }
            //
            //     if key_input.just_pressed(KeyCode::Numpad4) {
            //         new_transform.rotation *= Quat::from_rotation_x(-step);
            //
            //     }
            //     if key_input.just_pressed(KeyCode::Numpad6) {
            //         new_transform.rotation *= Quat::from_rotation_x(step);
            //
            //     }
            //     if key_input.just_pressed(KeyCode::Numpad0) {
            //     }
            //     *transform = new_transform;
            //     // transform.translation += delta_translation;
            //     // pan_orbit.target_focus += delta_translation;
            //
            // } else
            //
            {
                //    z+  y+
                //x-  []  x+
                //y-  z-
                //reset
                let step = 0.2;
                if key_input.just_pressed(KeyCode::Numpad1) {
                    pan_orbit.target_focus -= step * Vec3::Y;
                }
                if key_input.just_pressed(KeyCode::Numpad9) {
                    pan_orbit.target_focus += step * Vec3::Y;
                }

                if key_input.just_pressed(KeyCode::Numpad2) {
                    pan_orbit.target_focus -= step * Vec3::Z;
                }
                if key_input.just_pressed(KeyCode::Numpad8) {
                    pan_orbit.target_focus += step * Vec3::Z;
                }

                if key_input.just_pressed(KeyCode::Numpad4) {
                    pan_orbit.target_focus -= step * Vec3::X;
                }
                if key_input.just_pressed(KeyCode::Numpad6) {
                    pan_orbit.target_focus += step * Vec3::X;
                }
                if key_input.just_pressed(KeyCode::Numpad0) {
                    pan_orbit.target_focus = Vec3::ZERO;
                }
            }
        }

        // Force camera to update its transform
        pan_orbit.force_update = true;
    }
}

fn window_exit(mut exit: EventWriter<AppExit>, mut window_state: ResMut<WindowState>)
{
    if window_state.exit || !window_state.signal.is_run() {
        info!("window_exit send exit signal");
        exit.send(AppExit);
    }
}
