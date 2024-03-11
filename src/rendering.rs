use std::f64::consts::PI;

use nalgebra::Vector3;
use rand::rngs::ThreadRng;
use rand::Rng;

use crate::distribution::CosineWeightedDistr;
use crate::distribution::DistributionTooling;
use crate::distribution::LightSourceDistr;
use crate::distribution::MixDistr;
use crate::geometry::Intersection;
use crate::geometry::Shape::Plane;
use crate::geometry::{build_shifted_ray, intersect_scene, Ray};
use crate::scene::{self, Scene};

const BLACK: Vector3<f64> = Vector3::<f64>::new(0.0, 0.0, 0.0);

fn aces_tonemap(x: f64) -> f64 {
    const A: f64 = 2.51;
    const B: f64 = 0.03;
    const C: f64 = 2.43;
    const D: f64 = 0.59;
    const E: f64 = 0.14;

    f64::clamp(x * (A * x + B) / (x * (C * x + D) + E), 0.0, 1.1)
}

fn proportion_to_value(color: Vector3<f64>) -> [u8; 3] {
    [
        (aces_tonemap(color.x).powf(1.0 / 2.2) * 255.0).round() as u8,
        (aces_tonemap(color.y).powf(1.0 / 2.2) * 255.0).round() as u8,
        (aces_tonemap(color.z).powf(1.0 / 2.2) * 255.0).round() as u8,
    ]
}

// fn gen_w_and_pdf(
//     global_distr: &dyn DistributionTooling,
//     rng: &mut ThreadRng,
//     intersection_point: &Vector3<f64>,
//     intersection: &Intersection,
// ) -> (Vector3<f64>, f64) {
//     let w = global_distr.sample(rng, intersection_point, &intersection.normals[0]);

//     let pdf = global_distr.pdf(&intersection_point, &intersection.normals[0], &w);

//     if pdf < 0.0 {
//         gen_w_and_pdf(global_distr, rng, intersection_point, intersection)
//     } else {
//         (w, pdf)
//     }
// }

fn get_ray_color(
    scene: &Scene,
    rng: &mut ThreadRng,
    global_distr: &dyn DistributionTooling,
    ray: &Ray,
    depth: u32,
) -> Vector3<f64> {
    if depth >= scene.ray_depth {
        return BLACK;
    }

    intersect_scene(&ray, scene, None)
        .map(|(intersection, primitive)| {
            let intersection_point = ray.point + ray.direction * intersection.ts[0];
            match &primitive.material {
                scene::Material::DIFFUSE => {
                    let w = global_distr.sample(rng, &intersection_point, &intersection.normals[0]);

                    let pdf = global_distr.pdf(&intersection_point, &intersection.normals[0], &w);

                    if pdf <= 0.0 || w.dot(&intersection.normals[0]) <= 0.0 {
                        primitive.emission
                    } else {
                        primitive.emission
                            + (primitive.color / PI).component_mul(&get_ray_color(
                                scene,
                                rng,
                                global_distr,
                                &build_shifted_ray(intersection_point, w),
                                depth + 1,
                            )) * (w.dot(&intersection.normals[0]))
                                / pdf
                    }
                }
                scene::Material::METALLIC => {
                    let reflected_direction = ray.direction
                        - 2.0
                            * intersection.normals[0].dot(&ray.direction)
                            * intersection.normals[0];
                    primitive.color.component_mul(&get_ray_color(
                        scene,
                        rng,
                        global_distr,
                        &build_shifted_ray(intersection_point, reflected_direction),
                        depth + 1,
                    ))
                }
                scene::Material::DIELECTRIC { ior } => {
                    let (nu_1, nu_2): (f64, f64) = if intersection.outside {
                        (1.0, *ior)
                    } else {
                        (*ior, 1.0)
                    };
                    let normalized_ray_direction = ray.direction.normalize();
                    // let cos_tetta_1 = -intersection.normal.dot(&normalized_ray_direction);
                    let cos_tetta_1 = -intersection.normals[0].dot(&normalized_ray_direction);
                    let sin_tetta_2 = nu_1 / nu_2 * (1.0 - cos_tetta_1.powi(2)).sqrt();
                    let reflected_dir =
                        normalized_ray_direction + 2.0 * cos_tetta_1 * intersection.normals[0];
                    let r_0 = ((nu_1 - nu_2) / (nu_1 + nu_2)).powi(2);
                    let reflected_coef = r_0 + (1.0 - r_0) * (1.0 - cos_tetta_1).powi(5);
                    let reflected_color = get_ray_color(
                        scene,
                        rng,
                        global_distr,
                        &build_shifted_ray(intersection_point, reflected_dir),
                        depth + 1,
                    );
                    if sin_tetta_2 <= 1.0 && rand::thread_rng().gen::<f64>() > reflected_coef {
                        let cos_tetta_2 = (1.0 - sin_tetta_2.powi(2)).sqrt();
                        let refracted_dir = nu_1 / nu_2 * normalized_ray_direction
                            + (nu_1 / nu_2 * cos_tetta_1 - cos_tetta_2) * intersection.normals[0];
                        let refracted_color = get_ray_color(
                            scene,
                            rng,
                            global_distr,
                            &build_shifted_ray(intersection_point, refracted_dir),
                            depth + 1,
                        );
                        if intersection.outside {
                            refracted_color.component_mul(&primitive.color)
                        } else {
                            refracted_color
                        }
                    } else {
                        reflected_color
                    }
                }
            }
        })
        .unwrap_or(scene.background_color)
}

pub fn render_scene(scene: &Scene) -> Vec<u8> {
    let global_distr = &MixDistr {
        distribs: vec![
            Box::new(CosineWeightedDistr {}),
            Box::new(MixDistr {
                distribs: scene
                    .primitives
                    .iter()
                    .filter(|primitive| match primitive.shape {
                        Plane { normal: _ } => false,
                        _ => true,
                    })
                    .map(|primitive| {
                        Box::new(LightSourceDistr {
                            primitive: primitive.clone(),
                        }) as Box<dyn DistributionTooling>
                    })
                    .collect(),
            }),
        ],
    };

    let mut rng = rand::thread_rng();
    let mut result = Vec::<u8>::new();
    for row in 0..scene.height {
        for column in 0..scene.width {
            let x_local = column as f64 + 0.5;
            let y_local = row as f64 + 0.5;
            let x_global =
                (2.0 * x_local / scene.width as f64 - 1.0) * (scene.camera.fov_x / 2.0).tan();
            let y_global = (2.0 * y_local / scene.height as f64 - 1.0)
                * (scene.camera.fov_y / 2.0).tan()
                * (-1.0); // to reverse y asix
            let ray = Ray {
                point: scene.camera.position,
                direction: x_global * scene.camera.right_axis
                    + y_global * scene.camera.up_axis
                    + scene.camera.forward_axis,
            };

            let sum_pixel_color = (0..scene.samples)
                .map(|_| get_ray_color(scene, &mut rng, global_distr, &ray, 0))
                .sum::<Vector3<f64>>()
                / scene.samples as f64;

            result.extend(proportion_to_value(sum_pixel_color))
        }
    }
    result
}
