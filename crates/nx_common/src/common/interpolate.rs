use crate::common::transform2d;
use crate::{angle_norm, vector_2d_dist2};
use itertools::izip;
use std::f64::consts::{FRAC_PI_2, PI};
use tracing::warn;

pub fn build_bezier<const N: usize>(
    pa: &[f64; 2],
    pb: &[f64; 2],
    pc: &[f64; 2],
    pd: &[f64; 2],
    step: f64,
    path: &mut Vec<[f64; N]>,
)
{
    let mut simple_len: f64 =
        ((pa[0] - pb[0]) * (pa[0] - pb[0]) + (pa[1] - pb[1]) * (pa[1] - pb[1])).sqrt()
            + ((pc[0] - pb[0]) * (pc[0] - pb[0]) + (pc[1] - pb[1]) * (pc[1] - pb[1])).sqrt()
            + ((pc[0] - pd[0]) * (pc[0] - pd[0]) + (pc[1] - pd[1]) * (pc[1] - pd[1])).sqrt();
    let t_num = (simple_len / step) as usize;
    let step2 = 1.0f64 / (t_num as f64);
    log!(
        "pa : {:?},pb : {:?},pc : {:?},pd : {:?}, step :{}, step2: {}, t_num: {},simple_len: {}",
        pa,
        pb,
        pc,
        pd,
        step,
        step2,
        t_num,
        simple_len
    );

    path.resize(t_num + 1, [0.0; N]);

    let mut t: f64 = 0.0;

    let mut pab: [f64; 2] = [0.0; 2];
    let mut pbc: [f64; 2] = [0.0; 2];
    let mut pcd: [f64; 2] = [0.0; 2];
    let mut pabc: [f64; 2] = [0.0; 2];
    let mut pbcd: [f64; 2] = [0.0; 2];

    for i in 0..t_num {
        let one_minus_t = 1.0 - t;

        pab[0] = one_minus_t * pa[0] + t * pb[0];
        pab[1] = one_minus_t * pa[1] + t * pb[1];

        pbc[0] = one_minus_t * pb[0] + t * pc[0];
        pbc[1] = one_minus_t * pb[1] + t * pc[1];

        pcd[0] = one_minus_t * pc[0] + t * pd[0];
        pcd[1] = one_minus_t * pc[1] + t * pd[1];

        pabc[0] = one_minus_t * pab[0] + t * pbc[0];
        pabc[1] = one_minus_t * pab[1] + t * pbc[1];

        pbcd[0] = one_minus_t * pbc[0] + t * pcd[0];
        pbcd[1] = one_minus_t * pbc[1] + t * pcd[1];

        path[i][0] = one_minus_t * pabc[0] + t * pbcd[0];
        path[i][1] = one_minus_t * pabc[1] + t * pbcd[1];
        t += step2;
    }
    path[t_num][0] = pd[0];
    path[t_num][1] = pd[1];
}
pub fn build_bezier_line<const N: usize>(
    pa: &[f64; 2],
    pb: &[f64; 2],
    pc: &[f64; 2],
    pd: &[f64; 2],
    pe: &[f64; 2],
    step: f64,
    path: &mut Vec<[f64; N]>,
)
{
    warn!(
        "step: {}, pa : {:?},pb : {:?},pc : {:?},pd : {:?}, pe : {:?}",
        step,
        pa,
        pb,
        pc,
        pd,
        pe
    );

    let mut simple_len: f64 = (vector_2d_dist2!(pa, pb)).sqrt()
        + (vector_2d_dist2!(pb, pc)).sqrt()
        + (vector_2d_dist2!(pc, pd)).sqrt();
    let t_num = (simple_len / step) as usize;
    let curve_step = 1.0f64 / (t_num as f64);

    let line_de_len = (vector_2d_dist2!(pd, pe)).sqrt();
    let line_ab_len = (vector_2d_dist2!(pa, pb)).sqrt();

    let long_ab = 1.0;
    let long_ab_as_line = 0.5 * line_ab_len;

    let mut line_ab_num = if line_ab_len > long_ab {
        (long_ab_as_line / step) as usize
    } else {
        0
    };

    let line_de_num = (line_de_len / step) as usize;
    let line_de_step = line_de_len/(line_de_num as f64);

    let line_de_step_x = (pe[0] - pd[0])*line_de_step/line_de_len ;/// (line_de_num as f64);
    let line_de_step_y = (pe[1] - pd[1])*line_de_step/line_de_len;// / (line_de_num as f64);
    let mut line_de_update_x: f64 = line_de_step_x;
    let mut line_de_update_y: f64 = line_de_step_y;

    // println!("line_len: {}, step : {}, curve_step : {}, line_step_x : {}, line_step_y:{}",line_len, step,curve_step,line_step_x,line_step_y);

    let total_num = t_num + line_de_num + line_ab_num;
    warn!(
        "t_num: {}, line_de_num: {}, line_ab_num: {}",
        t_num, line_de_num, line_ab_num
    );
    path.resize(total_num, [0.0; N]);

    if total_num < 2 {
        return;
    }

    let mut pa = *pa;
    if line_ab_num > 0 {
        let ratio = long_ab_as_line / line_ab_len;
        let line_ab_step_x = ratio * (pb[0] - pa[0]) / (line_ab_num as f64);
        let line_ab_step_y = ratio * (pb[1] - pa[1]) / (line_ab_num as f64);
        let mut line_ab_update_x: f64 = line_ab_step_x;
        let mut line_ab_update_y: f64 = line_ab_step_y;

        for i in 0..line_ab_num {
            // line_update_x = line_step_x * (i - t_num) as f64;
            // line_update_y = line_step_y *  (i - t_num) as f64;

            path[i][0] = pa[0] + line_ab_update_x;
            path[i][1] = pa[1] + line_ab_update_y;

            line_ab_update_x += line_ab_step_x;
            line_ab_update_y += line_ab_step_y;
        }
        pa[0] += line_ab_update_x;
        pa[1] += line_ab_update_y;
    }

    // bezier
    let mut t: f64 = 0.0;

    let mut pab: [f64; 2] = [0.0; 2];
    let mut pbc: [f64; 2] = [0.0; 2];
    let mut pcd: [f64; 2] = [0.0; 2];
    let mut pabc: [f64; 2] = [0.0; 2];
    let mut pbcd: [f64; 2] = [0.0; 2];

    for i in line_ab_num..t_num + line_ab_num {
        let one_minus_t = 1.0 - t;

        pab[0] = one_minus_t * pa[0] + t * pb[0];
        pab[1] = one_minus_t * pa[1] + t * pb[1];

        pbc[0] = one_minus_t * pb[0] + t * pc[0];
        pbc[1] = one_minus_t * pb[1] + t * pc[1];

        pcd[0] = one_minus_t * pc[0] + t * pd[0];
        pcd[1] = one_minus_t * pc[1] + t * pd[1];

        pabc[0] = one_minus_t * pab[0] + t * pbc[0];
        pabc[1] = one_minus_t * pab[1] + t * pbc[1];

        pbcd[0] = one_minus_t * pbc[0] + t * pcd[0];
        pbcd[1] = one_minus_t * pbc[1] + t * pcd[1];

        path[i][0] = one_minus_t * pabc[0] + t * pbcd[0];
        path[i][1] = one_minus_t * pabc[1] + t * pbcd[1];
        t += curve_step;
    }
    // path[t_num][0] = pd[0];//0.5*(pd[0] + path[t_num-1][0] );
    // path[t_num][1] = pd[1];//0.5*(pd[1] + path[t_num-1][1] );
    //
    for i in t_num + line_ab_num..t_num + line_de_num + line_ab_num - 1 {
        // line_update_x = line_step_x * (i - t_num) as f64;
        // line_update_y = line_step_y *  (i - t_num) as f64;

        path[i][0] = pd[0] + line_de_update_x;
        path[i][1] = pd[1] + line_de_update_y;

        line_de_update_x += line_de_step_x;
        line_de_update_y += line_de_step_y;
    }
    // path[t_num + line_num][0] = pe[0];
    // path[t_num + line_num][1] = pe[1];
    // path[t_num + line_num][2] = pe[2];
}
// differential path generate, robot must move directly toward to target pose, start yaw is not used, target yaw must be used
pub fn build_bezier_from_pose(
    start: &[f64; 3],
    target: &[f64; 3],
    step: f64,
    ratio: &[f64; 2],
    path: &mut Vec<[f64; 3]>,
) -> bool
{
    let a = [start[0], start[1]];

    let d = [target[0], target[1]];

    // let start_pose = transform2d::Transform2d::new(start[0], start[1], start[2]);
    let target_pose = transform2d::Transform2d::new(target);

    let target_pose_inv = target_pose.inverse();

    let [start_pose_in_target_pose_x, start_pose_in_target_pose_y] =
        target_pose_inv.multiply_point(&start[0..2]);

    let mut b = [ratio[0] * start_pose_in_target_pose_x, 0.0]; // 0.6
    let mut c = [ratio[1] * start_pose_in_target_pose_x, 0.0]; //0.2

    let b = target_pose.multiply_point(&b);
    let c = target_pose.multiply_point(&c);

    build_bezier(&a, &b, &c, &d, step, path);
    if let Some(e) = path.last_mut() {
        *e = *target;
    }

    return true;
}
pub fn build_bezier_line_from_pose(
    start: &[f64; 3],
    target: &[f64; 3],
    step: f64,
    ratio: &[f64; 3],
    path: &mut Vec<[f64; 3]>,
) -> bool
{
    let a = [start[0], start[1]];

    let e = [target[0], target[1]];

    // let start_pose = transform2d::Transform2d::new(start[0], start[1], start[2]);
    let target_pose = transform2d::Transform2d::new(target);

    let target_pose_inv = target_pose.inverse();

    let [start_pose_in_target_pose_x, start_pose_in_target_pose_y] =
        target_pose_inv.multiply_point(&start[0..2]);

    let [ratio_b, ratio_c, mut len_d] = *ratio;
    len_d = len_d
        .min(start_pose_in_target_pose_x.abs())
        .copysign(start_pose_in_target_pose_x);

    let curve_x = start_pose_in_target_pose_x - len_d;

    let mut b = [ratio[0] * curve_x + len_d, 0.0]; // 0.6
    let mut c = [ratio[1] * curve_x + len_d, 0.0]; //0.2
    let mut d = [len_d, 0.0];

    let b = target_pose.multiply_point(&b);
    let c = target_pose.multiply_point(&c);
    let d = target_pose.multiply_point(&d);

    build_bezier_line(&a, &b, &c, &d, &e, step, path);
    if let Some(e) = path.last_mut() {
        *e = *target;
    }

    return path.len() > 1;
}
pub fn build_bezier_line_from_pose_with_start(
    start: &[f64; 3],
    target: &[f64; 3],
    step: f64,
    ratio: &[f64; 3],
    path: &mut Vec<[f64; 3]>,
) -> bool
{
    let a = [start[0], start[1]];

    let e = [target[0], target[1]];

    let dist = vector_2d_dist2!(start, target);
    if start.iter().any(|x| !x.is_finite()) || target.iter().any(|x| !x.is_finite()) || dist < 0.1 {
        return false;
    }

    // let start_pose = transform2d::Transform2d::new(start[0], start[1], start[2]);
    let target_pose = transform2d::Transform2d::new(target);
    let start_pose = transform2d::Transform2d::new(start);

    let target_pose_inv = target_pose.inverse();
    let start_pose_inv = start_pose.inverse();

    let [start_pose_in_target_pose_x, start_pose_in_target_pose_y] =
        target_pose_inv.multiply_point(&start[0..2]);

    let [ratio_b, ratio_c, mut len_d] = *ratio;
    len_d = len_d
        .min(start_pose_in_target_pose_x.abs())
        .copysign(start_pose_in_target_pose_x);

    let curve_x = start_pose_in_target_pose_x - len_d;

    let mut c = [ratio[1] * curve_x + len_d, 0.0]; //0.2
    let mut d = [len_d, 0.0];

    let c = target_pose.multiply_point(&c);

    let [c_in_start_pose_x, c_in_start_pose_y] = start_pose_inv.multiply_point(&c[0..2]);
    let mut b = [ratio[0] * c_in_start_pose_x, 0.0]; // 0.6

    let b = start_pose.multiply_point(&b);

    let d = target_pose.multiply_point(&d);

    build_bezier_line(&a, &b, &c, &d, &e, step, path);
    if let Some(e) = path.last_mut() {
        *e = *target;
    }
    return path.len() > 1;
}

// path nodes flow direction, not relative to robot pose
pub fn compute_path_flow(path: &mut Vec<[f64; 3]>) -> bool
{
    let len = path.len();

    // let [target_x,target_y,mut target_yaw] = *target;
    // target_yaw = angle_norm!(target_yaw);

    if (len >= 2) {
        for i in 0..len - 1 {
            let [p1x, p1y, ..] = path[i];
            let [p2x, p2y, ..] = path[i + 1];
            let mut p = &mut path[i];
            p[2] = (p2y - p1y).atan2(p2x - p1x);
        }

        path[len - 1][2] = path[len - 2][2];

        true
    } else {
        false
    }
}

pub fn compute_path_direction(path: &mut Vec<[f64; 3]>, target: &[f64; 3])
{
    let mut is_backward = false;
    if path.len() > 1 {
        let target_yaw_diff = angle_norm!(target[2] - path.last().unwrap()[2]);
        is_backward = target_yaw_diff.abs() > FRAC_PI_2;
        if is_backward {
            for p in &mut *path {
                p[2] = angle_norm!(p[2] + PI);
            }
        }
        if let Some(e) = path.last_mut() {
            e[0] = target[0];
            e[1] = target[1];
            e[2] = angle_norm!(target[2]);
        }
    } else {
    }
}

struct If<const B: bool>;
trait True {}
impl True for If<{ true }> {}

struct AssertGe<const N: usize, const D: usize>;

impl<const N: usize, const D: usize> AssertGe<N, D>
{
    const OK: () = assert!(N >= D, "N must be greater");
}

fn test_template_func<const N: usize>(data: &mut Vec<[f64; N]>)
{
    #[cfg(not(debug_assertions))]
    let () = AssertGe::<N, 3>::OK;
    // let _ = If::<{N  > 2}>::T;
    // const OK: () = assert!(N  > 2, "must be divisible");

    println!("data: {:?}", data);
}

#[cfg(test)]
mod test
{
    use crate::common::interpolate::{
        build_bezier, build_bezier_from_pose, compute_path_flow, test_template_func,
    };
    use std::f64::consts::PI;

    #[test]
    fn test_build_bezier()
    {
        let a = [2.0, 0.2];
        let b = [0.6, 0.0];
        let c = [0.2, 0.0];
        let d = [0.0, 0.0];

        let mut path: Vec<[f64; 3]> = vec![];

        build_bezier(&a, &b, &c, &d, 0.02, &mut path);
        println!("{:?}", path);
    }

    #[test]
    fn test_build_bezier_from_pose()
    {
        let start = [2.0, 0.2, 0.0];
        let target = [0.0, 0.0, 0.0];

        let mut path: Vec<[f64; 3]> = vec![];
        let mut path_flow: Vec<[f64; 3]> = vec![];

        {
            let target = [0.0, 0.0, 0.0];
            build_bezier_from_pose(&start, &target, 0.02, &[0.6, 0.2], &mut path);
            println!(
                "target = {:?},path_flow = {:?}\n, path = {:?}",
                target, path_flow, path
            );
        }
        {
            let target = [0.0, 0.0, PI];
            build_bezier_from_pose(&start, &target, 0.02, &[0.6, 0.2], &mut path);
            println!(
                "target = {:?},path_flow = {:?}\n, path = {:?}",
                target, path_flow, path
            );
        }
    }
    #[test]

    fn test_template()
    {
        let a: [f64; 2] = [2.0, 0.2];
        let b: [f64; 3] = [2.0, 0.2, 0.0];
        let mut va: Vec<[f64; 2]> = vec![];
        let mut vb: Vec<[f64; 3]> = vec![];

        va.push(a);

        vb.push(b);

        test_template_func(&mut va);
        test_template_func(&mut vb);
    }
}
