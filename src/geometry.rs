use nalgebra::Vector3;

use crate::scene::{Primitive, Scene};

pub enum Shape {
    Plane { normal: Vector3<f64> },
    Ellipsoid { r: Vector3<f64> },
    Box { s: Vector3<f64> },
}

pub struct Ray {
    pub point: Vector3<f64>,
    pub direction: Vector3<f64>,
}

const EPS: f64 = 0.00001;

pub fn build_shifted_ray(point: Vector3<f64>, direction: Vector3<f64>) -> Ray {
    Ray {
        point: point + direction * EPS,
        direction,
    }
}

fn solve_quadratic_equation(a: f64, b: f64, c: f64) -> Option<(f64, f64)> {
    let discr = b * b - 4.0 * a * c;
    if discr < 0.0 {
        None
    } else {
        let resolve1 = (-b - discr.sqrt()) / (2.0 * a);
        let resolve2 = (-b + discr.sqrt()) / (2.0 * a);
        Some((f64::min(resolve1, resolve2), f64::max(resolve1, resolve2)))
    }
}

pub struct Intersection {
    pub t: f64,
    pub normal: Vector3<f64>,
    pub outside: bool,
}

fn normalize(v: Vector3<f64>) -> Vector3<f64> {
    if v.x.abs() >= v.y.abs() && v.x.abs() >= v.z.abs() {
        Vector3::<f64>::new(v.x.signum(), 0.0, 0.0)
    } else if v.y.abs() >= v.x.abs() && v.y.abs() >= v.z.abs() {
        Vector3::<f64>::new(0.0, v.y.signum(), 0.0)
    } else {
        Vector3::<f64>::new(0.0, 0.0, v.z.signum())
    }
}

pub fn intersect_shape(ray: &Ray, shape: &Shape) -> Option<Intersection> {
    match shape {
        Shape::Plane { normal } => {
            let div = ray.direction.dot(normal);
            if div == 0.0 {
                return None;
            };
            let t = -ray.point.dot(normal) / div;
            if t < 0.0 {
                None
            } else {
                let outside = ray.direction.dot(normal) < 0.0;
                let normal_conjugated = if outside { *normal } else { -normal };
                Some(Intersection {
                    t,
                    normal: normal_conjugated,
                    outside: outside,
                })
            }
        }
        Shape::Ellipsoid { r } => {
            let point_div_r = ray.point.component_div(r);
            let dir_div_r = ray.direction.component_div(r);
            solve_quadratic_equation(
                dir_div_r.dot(&dir_div_r),
                2.0 * point_div_r.dot(&dir_div_r),
                point_div_r.dot(&point_div_r) - 1.0,
            )
            .and_then(|p| {
                if p.0 >= 0.0 {
                    Some(p.0)
                } else if p.1 >= 0.0 {
                    Some(p.1)
                } else {
                    None
                }
            })
            .map(|t| {
                let p = ray.point + ray.direction * t;
                let mut normal = p.component_div(r).normalize();
                let outside = normal.dot(&ray.direction.normalize()) < 0.0;
                if !outside {
                    normal = -normal;
                }
                assert!(
                    normal.dot(&ray.direction.normalize()) < 0.1,
                    "{}",
                    normal.dot(&ray.direction.normalize())
                );
                Intersection { t, normal, outside }
            })
        }
        Shape::Box { s } => {
            let calc_in_and_out = |s_proj: f64, point_proj, dir_proj| {
                let t0 = (s_proj - point_proj) / dir_proj;
                let t1 = (-s_proj - point_proj) / dir_proj;
                (f64::min(t0, t1), f64::max(t0, t1))
            };
            let tx = calc_in_and_out(s.x, ray.point.x, ray.direction.x);
            let ty = calc_in_and_out(s.y, ray.point.y, ray.direction.y);
            let tz = calc_in_and_out(s.z, ray.point.z, ray.direction.z);
            let t0 = f64::max(tx.0, f64::max(ty.0, tz.0));
            let t1 = f64::min(tx.1, f64::min(ty.1, tz.1));
            if t0 > t1 || t1 < 0.0 {
                None
            } else if t0 >= 0.0 {
                Some((t0, true))
            } else {
                Some((t1, false))
            }
            .map(|(t, outside)| {
                let p = ray.point + ray.direction * t;
                let normal = if outside {
                    normalize(p.component_div(s))
                } else {
                    -normalize(p.component_div(s))
                };
                Intersection { t, normal, outside }
            })
        }
    }
}

pub fn intersect_scene<'a>(
    ray: &Ray,
    scene: &'a Scene,
    distance_cap: Option<f64>,
) -> Option<(Intersection, &'a Primitive)> {
    scene
        .primitives
        .iter()
        .filter_map(|primitive| {
            let moved_ray_point = ray.point - primitive.position;
            let ray_to_intersect = Ray {
                point: primitive
                    .rotation
                    .conjugate()
                    .transform_vector(&moved_ray_point),
                direction: primitive
                    .rotation
                    .conjugate()
                    .transform_vector(&ray.direction),
            };
            intersect_shape(&ray_to_intersect, &primitive.shape).map(|intersection| {
                (
                    Intersection {
                        t: intersection.t,
                        normal: primitive.rotation.transform_vector(&intersection.normal),
                        outside: intersection.outside,
                    },
                    primitive,
                )
            })
        })
        .min_by(|x, y| x.0.t.partial_cmp(&y.0.t).expect("Nan on intersection."))
        .and_then(|(intersection, primitive)| {
            if let Some(val) = distance_cap {
                if intersection.t * ray.direction.norm() > val {
                    None
                } else {
                    Some((intersection, primitive))
                }
            } else {
                Some((intersection, primitive))
            }
        })
}
