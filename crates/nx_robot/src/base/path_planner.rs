use nx_common::common::interpolate::{
    build_bezier_from_pose, build_bezier_line_from_pose, build_bezier_line_from_pose_with_start,
    compute_path_direction, compute_path_flow,
};
use nx_common::common::math::linspace;
use nx_common::log;
use serde::Deserialize;
use std::cell::RefCell;
use std::collections::HashMap;
use std::ops::{Deref, DerefMut};
use std::rc::Rc;
use tracing::warn;

#[derive(Debug, Deserialize, Clone)]
pub struct RobotPathPlannerOption
{
    is_omnidirectional: bool,
    use_start_pose: bool,
    curve_param: [f64; 3],
    node_interval: f64,
}
#[derive(Debug, Deserialize, Clone)]
pub struct RobotPathPlannerConfig
{
    default_option: String,

    options: HashMap<String, RobotPathPlannerOption>,
}
pub struct RobotPathPlanner
{
    config: RobotPathPlannerConfig,
    select_option: Rc<RefCell<Option<RobotPathPlannerOption>>>,
    pub path: Rc<RefCell<Vec<[f64; 3]>>>,
}

impl RobotPathPlanner
{
    pub fn new(config: RobotPathPlannerConfig) -> Self
    {
        Self {
            config,
            select_option: Rc::new(RefCell::new(None)),
            path: Rc::new(RefCell::new(vec![])),
        }
    }

    pub fn set_option(&self, option_name: &str) -> bool
    {
        self.config.options.contains_key(option_name);

        let c = &self.config;
        let option_name = if c.options.contains_key(option_name) {
            option_name
        } else {
            log!(
                "option_name [{}] doesn't exist, use default [{}]",
                option_name,
                c.default_option
            );
            &c.default_option
        };
        let binding = c.options.get(option_name);
        let opt = match &binding {
            Some(t) => t,
            None => {
                log!("option_name [{}] doesn't exist", option_name);
                return false;
            }
        };
        *self.select_option.borrow_mut() = Some((*opt).clone());

        true
    }
    pub fn request_goal(&self, current: &[f64; 3], target: &[f64; 3]) -> bool
    {
        // let c = self.config.borrow();
        // let option_name = if c.options.contains_key(option_name) {
        //     option_name
        // }else{
        //     log!("option_name [{}] doesn't exist, use default [{}]", option_name, c.default_option);
        //      &c.default_option
        // };
        // let binding = c.options.get(option_name);
        // let opt = match &binding {
        //     Some(t) => t,
        //     None => {
        //         log!("option_name [{}] doesn't exist", option_name);
        //         return false;
        //     }
        // };

        let binding = self.select_option.borrow();

        let opt = match binding.as_ref() {
            Some(t) => t,
            None => {
                log!("select_option doesn't exist");
                return false;
            }
        };
        // if self.select_config_option.borrow().is_none(){
        //
        //     return false;
        // }
        //
        //
        // let select_option =  self.select_config_option.borrow() ;
        // let opt = select_option.as_ref().unwrap();

        if opt.is_omnidirectional {
            //
            let mut current = *current;
            current[2] = target[2];
            let path_dist = ((target[0] - current[0]) * (target[0] - current[0])
                + (target[1] - current[1]) * (target[1] - current[1]))
                .sqrt();
            let path_node_num = (path_dist / opt.node_interval) as usize;
            linspace(
                &current,
                target,
                self.path.borrow_mut().deref_mut(),
                path_node_num,
            );
        } else {
            let [curve_ratio_b, curve_ratio_c, follow_goal_len] = opt.curve_param;

            if (!opt.use_start_pose) {
                let ok = build_bezier_line_from_pose(
                    current,
                    target,
                    opt.node_interval,
                    &[curve_ratio_b, curve_ratio_c, follow_goal_len],
                    self.path.borrow_mut().deref_mut(),
                );
                warn!(
                    "build_bezier_line_from_pose:current:{:?}, target: {:?}, ok: {}",
                    current, target, ok
                );

                if ok {
                    compute_path_flow(self.path.borrow_mut().deref_mut());
                    compute_path_direction(self.path.borrow_mut().deref_mut(), target);
                }
            } else {
                let ok = build_bezier_line_from_pose_with_start(
                    current,
                    target,
                    opt.node_interval,
                    &[curve_ratio_b, curve_ratio_c, follow_goal_len],
                    self.path.borrow_mut().deref_mut(),
                );
                warn!(
                    "build_bezier_line_from_pose_with_start:current:{:?}, target: {:?}, ok: {}",
                    current, target, ok
                );
                if ok {
                    compute_path_flow(self.path.borrow_mut().deref_mut());
                    compute_path_direction(self.path.borrow_mut().deref_mut(), target);
                }
                return ok;
            }
        }

        true
    }
}
