// Calculate rotation about z axis
/*
    cos(yaw),   -sin(yaw),      0,
    sin(yaw),   cos(yaw),       0,
    0,          0,              1
*/

use num_traits::Float;
use std::f64::consts::FRAC_PI_2;

use crate::angle_norm;
use std::fmt::{write, Display, Formatter};

// yaw (Z), pitch (Y), roll (X)
/// return (qw, qx, qy, qz)
pub fn euler_to_quaternion(yaw: f64, pitch: f64, roll: f64) -> [f64; 4]
{
    let mut qw: f64 = 0.0;
    let mut qx: f64 = 0.0;
    let mut qy: f64 = 0.0;
    let mut qz: f64 = 0.0;

    // Abbreviations for the various angular functions
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();
    let cp = (pitch * 0.5).cos();
    let sp = (pitch * 0.5).sin();
    let cr = (roll * 0.5).cos();
    let sr = (roll * 0.5).sin();

    qw = cy * cp * cr + sy * sp * sr;
    qx = cy * cp * sr - sy * sp * cr;
    qy = sy * cp * sr + cy * sp * cr;
    qz = sy * cp * cr - cy * sp * sr;

    [qw, qx, qy, qz]
}

/// return (qw, qx, qy, qz)
pub fn yaw_to_quaternion(yaw: f64) -> [f64; 4]
{
    let mut qw: f64 = 0.0;
    let mut qx: f64 = 0.0;
    let mut qy: f64 = 0.0;
    let mut qz: f64 = 0.0;

    // Abbreviations for the various angular functions
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();

    qw = cy;
    qx = 0.0;
    qy = 0.0;
    qz = sy;

    /* 2d condition only yaw
    q.qw = cos(yaw * 0.5) ;
    q.qx = 0 ;
    q.qy = 0 ;
    q.qz = sin(yaw * 0.5) ;
     * */
    [qw, qx, qy, qz]
}

//https://stackoverflow.com/questions/11667783/quaternion-and-normalization

///
/// return (yaw, pitch, roll)
pub fn quaternion_to_euler(mut qw: f64, mut qx: f64, mut qy: f64, mut qz: f64) -> [f64; 3]
{
    let mut yaw: f64 = 0.0;
    let mut pitch: f64 = 0.0;
    let mut roll: f64 = 0.0;

    let qmagsq = qw * qw + qx * qx + qy * qy + qz * qz;

    let mut scale = 1.0;

    if ((1.0 - qmagsq).abs() < 2.107342e-08) {
        scale = 2.0 / (1.0 + qmagsq);
    } else {
        scale = 1.0 / (qmagsq).sqrt();
    }

    qw *= scale;
    qx *= scale;
    qy *= scale;
    qz *= scale;

    // We choose the quaternion with positive 'qw', i.e., the one with a smaller
    // angle that represents this orientation.
    if (qw < 0.0) {
        // Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
        qw = -1. * qw;
        qx = -1. * qx;
        qy = -1. * qy;
        qz = -1. * qz;
    }

    // roll (qx-axis rotation)
    let sinr_cosp = 2.0 * (qw * qx + qy * qz);
    let cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = sinr_cosp.atan2(cosr_cosp);

    // pitch (qy-axis rotation)
    let sinp = 2.0 * (qw * qy - qz * qx);
    pitch = if ((sinp).abs() >= 1.0) {
        std::f64::consts::FRAC_PI_2.copysign(sinp)
    } else {
        (sinp).asin()
    };

    // yaw (qz-axis rotation)
    let siny_cosp = 2.0 * (qw * qz + qx * qy);
    let cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = siny_cosp.atan2(cosy_cosp);

    [yaw, pitch, roll]
}

//https://juejin.cn/s/euler%20angle%20to%20rotation%20matrix%20c%2B%2B

pub fn euler_to_maxtrix<T: Float>(yaw: T, pitch: T, roll: T) -> [[T; 3]; 3]
{
    let mut cy = yaw.cos();
    let mut sy = yaw.sin();
    let mut cp = pitch.cos();
    let mut sp = pitch.sin();
    let mut cr = roll.cos();
    let mut sr = roll.sin();

    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

#[derive(Debug)]
pub struct Transform2d
{
    pub matrix: [[f64; 3]; 3],
}

impl Transform2d
{
    pub fn new(pose: &[f64; 3]) -> Self
    {
        let [x, y, yaw] = pose;
        Self {
            matrix: [
                [yaw.cos(), -yaw.sin(), *x],
                [yaw.sin(), yaw.cos(), *y],
                [0.0, 0.0, 1.0],
            ],
        }
    }

    pub fn as_array(&self) -> [f64; 3]
    {
        [self.x(), self.y(), self.yaw()]
    }
    pub fn set(&mut self, x: f64, y: f64, yaw: f64)
    {
        self.matrix = [
            [yaw.cos(), -yaw.sin(), x],
            [yaw.sin(), yaw.cos(), y],
            [0.0, 0.0, 1.0],
        ];
    }

    pub fn x(&self) -> f64
    {
        self.matrix[0][2]
    }
    pub fn y(&self) -> f64
    {
        self.matrix[1][2]
    }

    pub fn yaw(&self) -> f64
    {
        self.matrix[1][0].atan2(self.matrix[0][0])
    }

    pub fn inverse(&self) -> Self
    {
        let mut result = Self::default();
        let mut determinant: f64 = 0.0;
        //finding determinant
        for i in 0..3 {
            determinant += (self.matrix[0][i]
                * (self.matrix[1][(i + 1) % 3] * self.matrix[2][(i + 2) % 3]
                    - self.matrix[1][(i + 2) % 3] * self.matrix[2][(i + 1) % 3]));
        }

        let determinant_inv = 1.0 / determinant;

        for i in 0..3 {
            for j in 0..3 {
                result.matrix[i][j] = ((self.matrix[(j + 1) % 3][(i + 1) % 3]
                    * self.matrix[(j + 2) % 3][(i + 2) % 3])
                    - (self.matrix[(j + 1) % 3][(i + 2) % 3]
                        * self.matrix[(j + 2) % 3][(i + 1) % 3]))
                    * determinant_inv;
            }
        }

        result
    }

    pub fn multiply_point(&self, point: &[f64]) -> [f64; 2]
    {
        /*
         r00 r01 r02 tx     x0        x1
         r10 r11 r12 ty  X  y0   =>   y1
         r20 r21 r22 tz     z0        z1
         0   0   0   1      1         1
        */

        /*
        r00 r01 r02 tx     x0        x1
        r10 r11 r12 ty  X  y0   =>   y1
        r20 r21 r22 tz     z0        z1
        0   0   0   1      1         1
        */

        [
            self.matrix[0][0] * point[0] + self.matrix[0][1] * point[1] + self.matrix[0][2],
            self.matrix[1][0] * point[0] + self.matrix[1][1] * point[1] + self.matrix[1][2],
        ]
    }
    pub fn multiply_pose(&self, point: &[f64; 3]) -> [f64; 3]
    {
        /*
         r00 r01 r02 tx     x0        x1
         r10 r11 r12 ty  X  y0   =>   y1
         r20 r21 r22 tz     z0        z1
         0   0   0   1      1         1
        */

        [
            self.matrix[0][0] * point[0] + self.matrix[0][1] * point[1] + self.matrix[0][2],
            self.matrix[1][0] * point[0] + self.matrix[1][1] * point[1] + self.matrix[1][2],
            angle_norm!(point[2] + self.yaw()),
        ]
    }
    pub fn multiply(&self, rhs: &Self) -> Self
    {
        let mut result = Self::default();

        let a = &self.matrix;
        let b = &rhs.matrix;
        let c = &mut result.matrix;

        // Calculate the j-th column of the result in-place (in B) using the helper array

        for i in 0..3 {
            for j in 0..3 {
                c[i][j] = 0.0;
                for k in 0..3 {
                    c[i][j] += a[i][k] * b[k][j];
                }
            }
        }
        result
    }
    pub fn multiply_inplace(&mut self, rhs: &Self)
    {
        let a = &self.matrix;
        let b = &rhs.matrix;
        let mut c = [[0.0f64; 3]; 3];

        // Calculate the j-th column of the result in-place (in B) using the helper array

        for i in 0..3 {
            for j in 0..3 {
                c[i][j] = 0.0;
                for k in 0..3 {
                    c[i][j] += a[i][k] * b[k][j];
                }
            }
        }
        self.matrix = c;
    }
}

impl Default for Transform2d
{
    fn default() -> Self
    {
        Self {
            matrix: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }
}

impl Clone for Transform2d
{
    fn clone(&self) -> Self
    {
        Self {
            matrix: self.matrix,
        }
    }
}

impl Copy for Transform2d {}

impl Display for Transform2d
{
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result
    {
        write!(
            f,
            "\n[x, y, yaw]:\n[{}, {}, {}], \n[matrix]:\n{:?}",
            self.x(),
            self.y(),
            self.yaw(),
            self.matrix
        )
    }
}

#[test]
fn test()
{
    let mut t1 = Transform2d::default();
    let mut t2 = Transform2d::new(&[0.1, 0.2, 0.4]);

    let mut t1_inv = t1.inverse();
    let mut t2_inv = t2.inverse();

    let t3 = t2.multiply(&t2_inv);
    println!("t2 = {}", t2);

    println!("t3 = {}", t3);
    let point = [1.0, 0.5];
    let point_2 = t2.multiply_point(&point);
    println!("point: {:?}, point_2: {:?}", point, point_2);

    {
        let p1 = [0.1, 0.2, 0.3];
        let tp1 = Transform2d::new(&p1);
        let p2 = [1.1, 2.2, 1.3];
        let mut tp2 = Transform2d::new(&p2);

        let tp3 = tp1.multiply(&tp2);

        let p3 = tp1.multiply_pose(&p2);
        let p3_2 = tp1.multiply_point(&p2);
        println!("p3_2:{:?}", p3_2);

        println!("p3:{:?}", p3);
        println!("tp3:{ }", tp3);
    }
}
