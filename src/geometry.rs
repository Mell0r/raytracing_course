use nalgebra::Vector3;

pub enum Shape {
    Plane { normal: Vector3<f32> },
    Ellipsoid { r: Vector3<f32> },
    Box { s: Vector3<f32> },
}

pub struct Ray {
    pub point: Vector3<f32>,
    pub direction: Vector3<f32>,
}

fn solve_quadratic_equation(a: f32, b: f32, c: f32) -> Option<(f32, f32)> {
    let discr = b * b - 4.0 * a * c;
    if discr < 0.0 {
        None
    } else {
        let resolve1 = (-b - discr.sqrt()) / (2.0 * a);
        let resolve2 = (-b + discr.sqrt()) / (2.0 * a);
        Some((f32::min(resolve1, resolve2), f32::max(resolve1, resolve2)))
    }
}

pub fn intersect(ray: &Ray, shape: &Shape) -> Option<f32> {
    match shape {
        Shape::Plane { normal } => {
            let div = ray.direction.dot(normal);
            if div == 0.0   {
                return None
            };
            let t = -ray.point.dot(normal) / div;
            if t < 0.0 {
                None
            } else {
                Some(t)
            }
        }
        Shape::Ellipsoid { r } => {
            let point_div_r = ray.point.component_div(r);
            let dir_div_r = ray.direction.component_div(r);
            solve_quadratic_equation(
                dir_div_r.dot(&dir_div_r),
                2.0 * point_div_r.dot(&dir_div_r),
                point_div_r.dot(&point_div_r) - 1.0,
            ).and_then(|p| if p.0 >= 0.0 {
                Some(p.0)
            } else if p.1 >= 0.0 {
                Some(p.1)
            } else {
                None
            })
        }
        Shape::Box { s } => {
            let calc_in_and_out = |s_proj: f32, point_proj, dir_proj| {
                let t0 = (s_proj - point_proj) / dir_proj;
                let t1 = (-s_proj - point_proj) / dir_proj;
                (f32::min(t0, t1), f32::max(t0, t1))
            };
            let tx = calc_in_and_out(s.x,ray.point.x, ray.direction.x);
            let ty = calc_in_and_out(s.y,ray.point.y, ray.direction.y);
            let tz = calc_in_and_out(s.z,ray.point.z, ray.direction.z);
            let t0 = f32::max(tx.0, f32::max(ty.0, tz.0));
            let t1 = f32::min(tx.1, f32::min(ty.1, tz.1));
            if t0 > t1 || t1 < 0.0 {
                None
            } else if t0 >= 0.0 {
                Some(t0)
            } else {
                Some(t1)
            }
        }
    }
}
