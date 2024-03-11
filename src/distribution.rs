use std::{f64::consts::PI, iter::zip};

use nalgebra::Vector3;
use rand::{rngs::ThreadRng, seq::SliceRandom, Rng};

use crate::{
    geometry::{intersect_shape, Ray, Shape},
    scene::Primitive,
};

pub trait DistributionTooling {
    fn sample(
        &self,
        rng: &mut ThreadRng,
        point_from: &Vector3<f64>,
        normal_from: &Vector3<f64>,
    ) -> Vector3<f64>;
    fn pdf(
        &self,
        point_from: &Vector3<f64>,
        normal_from: &Vector3<f64>,
        direction: &Vector3<f64>,
    ) -> f64;
}

pub fn generate_unit_on_sphere(rng: &mut ThreadRng) -> Vector3<f64> {
    let direction = Vector3::<f64>::new(
        rng.gen_range(-1.0..1.0),
        rng.gen_range(-1.0..1.0),
        rng.gen_range(-1.0..1.0),
    );
    if direction.norm() > 1.0 {
        generate_unit_on_sphere(rng)
    } else {
        direction.normalize()
    }
}

pub struct CosineWeightedDistr {}

impl DistributionTooling for CosineWeightedDistr {
    fn sample(
        &self,
        rng: &mut ThreadRng,
        _point_from: &Vector3<f64>,
        normal_from: &Vector3<f64>,
    ) -> Vector3<f64> {
        (generate_unit_on_sphere(rng) + normal_from).normalize()
    }

    fn pdf(
        &self,
        _point_from: &Vector3<f64>,
        normal_from: &Vector3<f64>,
        direction: &Vector3<f64>,
    ) -> f64 {
        f64::max(0.0, direction.normalize().dot(normal_from) / PI)
    }
}

pub struct LightSourceDistr {
    pub primitive: Primitive,
}

impl DistributionTooling for LightSourceDistr {
    fn sample(
        &self,
        rng: &mut ThreadRng,
        point_from: &Vector3<f64>,
        _normal_from: &Vector3<f64>,
    ) -> Vector3<f64> {
        let mut generate_rand_local_point = || -> Vector3<f64> {
            match self.primitive.shape {
                Shape::Plane { normal: _ } => Default::default(),

                Shape::Box { s } => {
                    let w_x = 4.0 * s.y * s.z;
                    let w_y = 4.0 * s.x * s.z;
                    let w_z = 4.0 * s.x * s.y;
                    let rnd_face = rng.gen_range(0.0..(w_x + w_y + w_z));
                    let rnd_sign = if rng.gen_bool(0.5) { 1.0 } else { -1.0 };
                    let rnd_val1 = rng.gen_range(-1.0..1.0);
                    let rnd_val2 = rng.gen_range(-1.0..1.0);
                    if rnd_face < w_x {
                        Vector3::<f64>::new(s.x * rnd_sign, s.y * rnd_val1, s.z * rnd_val2)
                    } else if rnd_face < w_x + w_y {
                        Vector3::<f64>::new(s.x * rnd_val1, s.y * rnd_sign, s.z * rnd_val2)
                    } else {
                        Vector3::<f64>::new(s.x * rnd_val1, s.y * rnd_val2, s.z * rnd_sign)
                    }
                }

                Shape::Ellipsoid { r } => generate_unit_on_sphere(rng).component_mul(&r),
            }
        };

        (self
            .primitive
            .rotation
            .transform_vector(&generate_rand_local_point())
            + self.primitive.position
            - point_from)
            .normalize()
    }

    fn pdf(
        &self,
        point_from: &Vector3<f64>,
        _normal_from: &Vector3<f64>,
        direction: &Vector3<f64>,
    ) -> f64 {
        let Some(intersection) = intersect_shape(
            &Ray {
                point: *point_from,
                direction: *direction, // self
                                       //     .primitive
                                       //     .rotation
                                       //     .conjugate()
                                       //     .transform_vector(&direction)
                                       //     - self.primitive.position,
            },
            &self.primitive.shape,
        ) else {
            return 0.0;
        };

        zip(intersection.ts, intersection.normals)
            .map(|(t, normal)| {
                let intersection_point = point_from + t * direction;

                let local_point = self
                    .primitive
                    .rotation
                    .conjugate()
                    .transform_vector(&(intersection_point - self.primitive.position));

                let local_pdf = match self.primitive.shape {
                    Shape::Plane { normal: _ } => Default::default(),
                    Shape::Box { s } => 1.0 / 8.0 / (s.x * s.y + s.x * s.z + s.y * s.z),
                    Shape::Ellipsoid { r } => {
                        let n = local_point.component_div(&r);

                        1.0 / 4.0
                            / PI
                            / ((n.x * r.y * r.z).powi(2)
                                + (r.x * n.y * r.z).powi(2)
                                + (r.x * r.y * n.z).powi(2))
                            .sqrt()
                    }
                };

                let vector_on_sample = intersection_point - point_from;
                let omega = vector_on_sample.normalize();
                local_pdf * (vector_on_sample.norm_squared() / (normal.dot(&omega)).abs())
            })
            .sum()
    }
}

pub struct MixDistr {
    pub distribs: Vec<Box<dyn DistributionTooling>>,
}

impl DistributionTooling for MixDistr {
    fn sample(
        &self,
        rng: &mut ThreadRng,
        point_from: &Vector3<f64>,
        normal: &Vector3<f64>,
    ) -> Vector3<f64> {
        self.distribs
            .choose(rng)
            .expect("Empty distribution vector in mixed distribution.")
            .sample(rng, point_from, normal)
    }

    fn pdf(&self, point_from: &Vector3<f64>, normal: &Vector3<f64>, dir: &Vector3<f64>) -> f64 {
        self.distribs
            .iter()
            .map(|distr| distr.pdf(point_from, normal, dir))
            .sum::<f64>()
            / self.distribs.len() as f64
    }
}
