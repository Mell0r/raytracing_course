use nalgebra::Vector3;
use rand::Rng;

use crate::geometry::{build_shifted_ray, generate_random_unit_direction, intersect_scene, Ray};
use crate::scene::{self, get_light_characteristic_to_point, Scene};

const BLACK: Vector3<f64> = Vector3::<f64>::new(0.0, 0.0, 0.0);

// fn saturate(color: Vector3<f64>) -> Vector3<f64> {
//     Vector3::<f64>::new(
//         f64::clamp(color.x, 0.0, 1.1),
//         f64::clamp(color.y, 0.0, 1.1),
//         f64::clamp(color.z, 0.0, 1.1),
//     )
// }

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
        // (color.x * 255.0).round() as u8,
        // (color.y * 255.0).round() as u8,
        // (color.z * 255.0).round() as u8,
    ]
}

fn get_ray_color(scene: &Scene, ray: &Ray, depth: u32) -> Vector3<f64> {
    if depth >= scene.ray_depth {
        return BLACK;
    }

    intersect_scene(&ray, scene, None)
        .map(|(intersection, primitive)| {
            let intersection_point = ray.point + ray.direction * intersection.t;
            match &primitive.material {
                scene::Material::DIFFUSE => {
                    // primitive.color.component_mul(&scene.lights.iter().fold(
                    //     scene.ambient_light,
                    //     |sum: Vector3<f64>, light| {
                    //         let (light_direction, intensity, distance) =
                    //             get_light_characteristic_to_point(&light, &intersection_point);

                    //         if let Some(_) = intersect_scene(
                    //             &build_shifted_ray(intersection_point, light_direction),
                    //             scene,
                    //             distance,
                    //         ) {
                    //             return sum;
                    //         };

                    //         let mult: f64 = light_direction.normalize().dot(&intersection.normal);
                    //         if mult < 0.0 {
                    //             sum
                    //         } else {
                    //             sum + mult * intensity
                    //         }
                    //     },
                    // ))

                    let rand_dir = generate_random_unit_direction(&intersection.normal);
                    primitive.emission
                        + primitive.color.component_mul(&get_ray_color(
                            &scene,
                            &build_shifted_ray(intersection_point, rand_dir),
                            depth + 1,
                        )) * rand_dir.dot(&intersection.normal)
                            * 2.0
                }
                scene::Material::METALLIC => {
                    let reflected_direction = ray.direction
                        - 2.0 * intersection.normal.dot(&ray.direction) * intersection.normal;
                    primitive.color.component_mul(&get_ray_color(
                        scene,
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
                    let cos_tetta_1 = -intersection.normal.dot(&normalized_ray_direction);
                    let sin_tetta_2 = nu_1 / nu_2 * (1.0 - cos_tetta_1.powi(2)).sqrt();
                    let reflected_dir =
                        normalized_ray_direction + 2.0 * cos_tetta_1 * intersection.normal;
                    let r_0 = ((nu_1 - nu_2) / (nu_1 + nu_2)).powi(2);
                    let reflected_coef = r_0 + (1.0 - r_0) * (1.0 - cos_tetta_1).powi(5);
                    let reflected_color = get_ray_color(
                        scene,
                        &build_shifted_ray(intersection_point, reflected_dir),
                        depth + 1,
                    );
                    if sin_tetta_2 <= 1.0 && rand::thread_rng().gen::<f64>() > reflected_coef {
                        let cos_tetta_2 = (1.0 - sin_tetta_2.powi(2)).sqrt();
                        let refracted_dir = nu_1 / nu_2 * normalized_ray_direction
                            + (nu_1 / nu_2 * cos_tetta_1 - cos_tetta_2) * intersection.normal;
                        let refracted_color = get_ray_color(
                            scene,
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
    (0..scene.height)
        .flat_map(move |row| {
            (0..scene.width).flat_map(move |column| {
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

                let mut sum_pixel_color: Vector3<f64> = Default::default();
                for _ in 0..scene.samples {
                    sum_pixel_color += get_ray_color(scene, &ray, 0);
                }
                sum_pixel_color /= scene.samples as f64;

                proportion_to_value(sum_pixel_color)
            })
        })
        .collect()
}
