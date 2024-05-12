use num_traits::Float;
use std::f64::consts::{PI, TAU};
use std::ops::{AddAssign, Mul};


//https://stackoverflow.com/questions/74193288/calculating-distance-from-a-point-to-a-line-given-the-line-direction

#[derive(Debug, Clone, Copy)]
pub struct Point3f{
    pub x:f32,
    pub y:f32,
    pub z:f32
}
impl Point3f{

    pub fn normalise(&mut self){
        let length_div = 1.0/self.length();
        self.x  *= length_div;
        self.y  *= length_div;
        self.z  *= length_div;
    }

    pub fn dot(&self, rhs: Point3f ) -> f32{
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
    pub fn length(&self) ->f32{
        (self.x*self.x + self.y*self.y + self.z*self.z).sqrt()
    }

    pub fn distance_to_line(line0 :Point3f, mut line_dir: Point3f, mut point:Point3f) ->f32{
        // Translate point making it relative to the start of the line
        point -= line0;
        line_dir.normalise();

        // Coefficient to get closest point on the line,
        // equal to length(point) * cos(angle) / length(lineDir)
        let pl:f32 = line_dir.dot( point );
        // Position of the closest point on the line

        let closest = line_dir * pl;

        // Compute distance between two points
        return ( point - closest ).length();
    }
}


impl std::ops::SubAssign for Point3f {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;

    }
}
impl std::ops::DivAssign<f32> for Point3f{
    fn div_assign(&mut self, rhs: f32) {

        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
    }
}

impl std::ops::Mul<f32> for Point3f{
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self{
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl std::ops::Sub for Point3f {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self{
            x:self.x - rhs.x,
            y:self.y - rhs.y,
            z:self.z - rhs.z,
        }
    }
}
impl std::ops::Add for Point3f{
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {

        Self{
            x:self.x + rhs.x,
            y:self.y + rhs.y,
            z:self.z + rhs.z,
        }
    }

}

impl Mul<Point3f> for f32 {
    type Output = Point3f;

    fn mul(self, rhs: Point3f) -> Self::Output {
        Self::Output{
            x : self * rhs.x,
            y : self * rhs.y,
            z : self * rhs.z,
        }
    }
}

//https://stackoverflow.com/questions/45630057/rust-generic-addassign-with-references

//https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees
// float normalizedAngle = angle - (ceil((angle + M_PI)/(2*M_PI))-1)*2*M_PI;  // (-Pi;Pi]:
// pub fn angle_norm (angle:f64) ->  f64
// {
//     angle - ( ((angle + PI ).ceil()/(TAU))-1.0)*TAU
// }
// angle - ( ((angle + M_PI)/(2.0*M_PI)).ceil()-1.0)*2.0*M_PI
#[macro_export]
macro_rules! angle_norm {
    ($fmt:expr) => {
        $fmt - ((($fmt + 3.14159265358979323846264338327950288)
            / (6.28318530717958647692528676655900577))
            .ceil()
            - 1.0)
            * 6.28318530717958647692528676655900577
    };
}

#[macro_export]
macro_rules! vector_2d_dist2 {
    ($v1:expr,$v2:expr) => {
        ($v1[0] - $v2[0]) * ($v1[0] - $v2[0]) + ($v1[1] - $v2[1]) * ($v1[1] - $v2[1])
    };
}

#[macro_export]
macro_rules! vector_2d_norm2 {
    ($v1:expr ) => {
        ($v1[0]) * ($v1[0]) + ($v1[1]) * ($v1[1])
    };
}
#[macro_export]
macro_rules! cross_product_2d {
    ($v1:expr,$v2:expr) => {
        $v1[0] * $v2[1] - $v1[1] * $v2[0]
    };
}

#[macro_export]
macro_rules! dot_product_2d {
    ($v1:expr,$v2:expr) => {
        $v1[0] * $v2[0] + $v1[1] * $v2[1]
    };
}
pub fn cross_product_2d<T>(u: &[T; 2], v: &[T; 2]) -> T
where
    T: Float,
{
    u[0] * v[1] - u[1] * v[0]
}

pub fn dot_product_2d<T>(u: &[T; 2], v: &[T; 2]) -> T
where
    T: Float,
{
    u[0] * v[0] + u[1] * v[1]
}

fn det<T>(a: T, b: T, c: T, d: T) -> T
where
    T: Float,
{
    a * d - b * c
}

//https://www.programiz.com/cpp-programming/examples/quadratic-roots
//For a quadratic equation ax2+bx+c = 0 (where a, b and c are coefficients), it's roots is given by following the formula.
//
pub fn find_roots_of_quadratic_equation(a: f64, b: f64, c: f64) -> (u32, f64, f64)
{
    let mut discriminant: f64 = 0.0;
    let mut x1: f64 = 0.0;
    let mut x2: f64 = 0.0;

    let mut realPart: f64 = 0.0;
    let mut imaginaryPart: f64 = 0.0;

    discriminant = b * b - 4.0 * a * c;

    if (discriminant > 0.0) {
        let discriminant_sqrt = discriminant.sqrt();
        x1 = (-b + discriminant_sqrt) / (2.0 * a);
        x2 = (-b - discriminant_sqrt) / (2.0 * a);
        //        std::cout << "Roots are real and different." << std::endl;
        //        std::cout << "x1 = " << x1 << std::endl;
        //        std::cout << "x2 = " << x2 << std::endl;
        return (2, x1, x2);
    } else if (discriminant == 0.0) {
        //        std::cout << "Roots are real and same." << std::endl;
        x1 = -b / (2.0 * a);
        //        std::cout << "x1 = x2 =" << x1 << std::endl;
        return (1, x1, x1);
    } else {
        realPart = -b / (2.0 * a);
        imaginaryPart = (-discriminant).sqrt() / (2.0 * a);
        //        std::cout << "Roots are complex and different."  << std::endl;
        //        std::cout << "x1 = " << realPart << "+" << imaginaryPart << "i" << std::endl;
        //        std::cout << "x2 = " << realPart << "-" << imaginaryPart << "i" << std::endl;
        return (0, x1, x1);
    }
    return (0, x1, x1);
}

pub fn line_line_intersect<T>(
    x1: T,
    y1: T, //Line 1 start
    x2: T,
    y2: T, //Line 1 end
    x3: T,
    y3: T, //Line 2 start
    x4: T,
    y4: T, //Line 2 end
) -> (bool, [T; 2])
where
    T: Float
        // + Copy
        // + std::ops::Mul<Output = T>
        // + std::ops::Div<Output = T>
        // + std::ops::Add<Output = T>
        // + std::ops::Sub<Output = T>
        // + std::cmp::PartialEq
        + From<f64>,
{
    let mut out: [T; 2] = [0.0.into(); 2];

    let [ix_out, iy_out] = &mut out;

    //http://mathworld.wolfram.com/Line-LineIntersection.html

    let det_l1 = det(x1, y1, x2, y2);
    let det_l2 = det(x3, y3, x4, y4);
    let x1mx2 = x1 - x2;
    let x3mx4 = x3 - x4;
    let y1my2 = y1 - y2;
    let y3my4 = y3 - y4;

    let xnom = det(det_l1, x1mx2, det_l2, x3mx4);
    let ynom = det(det_l1, y1my2, det_l2, y3my4);
    let denom = det(x1mx2, y1my2, x3mx4, y3my4);
    if denom == 0.0.into()
    //Lines don't seem to cross
    {
        return (false, out);
    }

    *ix_out = xnom / denom;
    *iy_out = ynom / denom;

    let valid = ix_out.is_finite() && iy_out.is_finite();

    return (valid, out);
}

pub fn linspace<T, const N: usize>(
    start: &[T; N],
    end: &[T; N],
    target: &mut Vec<[T; N]>,
    len: usize,
) where
    T: Float + AddAssign<T> + From<f64>,
{
    target.resize(len + 1, *start);

    for i in 0..N {
        let dist = (end[i] - start[i]) / ((len as f64).into());
        let mut update_dist: T = 0.0.into();
        for j in 0..target.len() - 1 {
            target[j][i] += update_dist.into();

            update_dist += dist;
        }
        *target.last_mut().unwrap() = *end;
    }
}

#[cfg(test)]
mod test
{
    use crate::common::math::{linspace, Point3f};
    use std::f64::consts::{PI, TAU};
    use std::ops::Mul;

    #[test]
    fn test_linspace_bench()
    {
        bench!(
            {
                let v1 = [0.0_f64, 1.0];
                let v2 = [1.0_f64, 3.0];
                let mut vv1: Vec<[f64; 2]> = vec![];
                linspace(&v1, &v2, &mut vv1, 10);
            },
            5
        );
    }
    #[test]
    fn test_linspace()
    {
        let v1 = [0.0_f64, 1.0, 0.1];
        let v2 = [1.0_f64, 3.0, 0.2];
        let mut vv1: Vec<[f64; 3]> = vec![];
        linspace(&v1, &v2, &mut vv1, 10);

        println!("vv1 : {:?}", vv1);
    }
    #[test]
    fn test_atan()
    {
        let v1: f64 = 0.0;
        let v2: f64 = 0.0;
        let v3 = v1.atan2(v2);
        println!("v3: {}", v3);
    }
    #[test]
    fn test_angle_norm()
    {
        let v1: f64 = -3.1415765;
        let v2: f64 = 0.0;

        let v3 = angle_norm!(v1 - v2);
        println!("v3 = {}", v3);
        println!("v1 + pi = {}", v1 + 2.0 * PI);

        let v4 = v1 - (((v1 + PI).ceil() / (TAU)) - 1.0) * TAU;

        println!("v4 = {}", v4);

        let angle = v1;
        let M_PI = PI;

        let v5 = angle - (((angle + M_PI) / (2.0 * M_PI)).ceil() - 1.0) * 2.0 * M_PI;
        println!("v5 = {}", v5);

        return;
        for i in -70..70 {
            let i = i as f64 * 0.1;
            println!("i = {}, angle_norm(i) = {} ", i, angle_norm!(i));
        }
        for i in -70..70 {
            let i = i as f64 * 0.1;
            println!("i = {}, angle_norm(i) = {} ", i, angle_norm!(i));
        }
    }

    #[test]
    fn test_product()
    {
        let v1 = [1, 2];
        let v2 = [1, 2];

        println!("v1 dot v2 = {}", dot_product_2d!(v1, v2));
        println!("v1 cross v2 = {}", cross_product_2d!(v1, v2));
    }

    #[test]
    fn point_to_line_3d(){

        let mut v1 = Point3f{
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let mut v2 = Point3f{
            x: 3.0,
            y: 4.0,
            z: 5.0,
        };
        let mut v3 = v1 + v2;
        let mut v4 = v1 - v2;
        let mut v5 = 2.0_f32 * v4 * -1.0_f32;


        v3 = Point3f{
            x: 4.0,
            y: 1.0,
            z: 1.0,
        };
        v4 = Point3f{
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        v5 = Point3f{
            x: 4.0,
            y: 1.0,
            z: 2.0,
        };
        let dist = Point3f::distance_to_line(v3,v4,v5);

        println!("v3: {:?}, v4 : {:?}, v5: {:?}, dist: {}", v3, v4,v5,dist);


    }
}
