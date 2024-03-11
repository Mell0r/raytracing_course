use nalgebra::Vector3;

use crate::scene::{Primitive, Scene};

#[derive(Clone)]
pub enum Shape {
    Plane { normal: Vector3<f64> },
    Ellipsoid { r: Vector3<f64> },
    Box { s: Vector3<f64> },
}

pub struct Ray {
    pub point: Vector3<f64>,
    pub direction: Vector3<f64>,
}

const EPS: f64 = 0.0001;

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
    pub ts: Vec<f64>,
    pub normals: Vec<Vector3<f64>>,
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
            if div.abs() <= 0.00001 {
                return None;
            };
            let t = -ray.point.dot(normal) / div;
            if t < 0.0 {
                None
            } else {
                let outside = ray.direction.dot(normal) < 0.0;
                let normal_conjugated = if outside { *normal } else { -normal };
                Some(Intersection {
                    ts: vec![t],
                    normals: vec![normal_conjugated.normalize()],
                    outside,
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
                    Some((vec![p.0, p.1], true))
                } else if p.1 >= 0.0 {
                    Some((vec![p.1], false))
                } else {
                    None
                }
            })
            .map(|(ts, outside)| Intersection {
                ts: ts.clone(),
                normals: ts
                    .iter()
                    .map(|&t| {
                        let p = ray.point + ray.direction * t;
                        let normal = p.component_div(r).component_div(r).normalize();
                        if outside {
                            normal
                        } else {
                            -normal
                        }
                    })
                    .collect(),
                outside,
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
                Some((vec![t0, t1], true))
            } else {
                Some((vec![t1], false))
            }
            .map(|(ts, outside)| Intersection {
                ts: ts.clone(),
                normals: ts
                    .iter()
                    .map(|&t| {
                        let p = ray.point + ray.direction * t;
                        if outside {
                            normalize(p.component_div(s))
                        } else {
                            -normalize(p.component_div(s))
                        }
                    })
                    .collect(),
                outside,
            })
        }
    }
}

pub fn intersect_primitive(ray: &Ray, primitive: &Primitive) -> Option<Intersection> {
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

    intersect_shape(&ray_to_intersect, &primitive.shape)
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
                        ts: intersection.ts,
                        normals: intersection
                            .normals
                            .iter()
                            .map(|&normal| primitive.rotation.transform_vector(&normal))
                            .collect(),
                        outside: intersection.outside,
                    },
                    primitive,
                )
            })
        })
        .min_by(|x, y| {
            x.0.ts[0]
                .partial_cmp(&y.0.ts[0])
                .expect("Nan on intersection.")
        })
        .and_then(|(intersection, primitive)| {
            if let Some(val) = distance_cap {
                if intersection.ts[0] * ray.direction.norm() > val {
                    None
                } else {
                    Some((intersection, primitive))
                }
            } else {
                Some((intersection, primitive))
            }
        })
}