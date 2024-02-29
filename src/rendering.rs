use nalgebra::Vector3;

use crate::geometry::{intersect, Ray};
use crate::scene::Scene;

fn color_to_pixel(color: Vector3<f32>) -> [u8; 3] {
    [
        (color.x * 255.0).round() as u8,
        (color.y * 255.0).round() as u8,
        (color.z * 255.0).round() as u8,
    ]
}

pub fn render_scene(scene: &Scene) -> Vec<u8> {
    (0..scene.height)
        .flat_map(move |row| {
            (0..scene.width).flat_map(move |column| {
                let x_local = column as f32 + 0.5;
                let y_local = row as f32 + 0.5;
                let x_global =
                    (2.0 * x_local / scene.width as f32 - 1.0) * (scene.camera.fov_x / 2.0).tan();
                let y_global = (2.0 * y_local / scene.height as f32 - 1.0)
                    * (scene.camera.fov_y / 2.0).tan()
                    * (-1.0); // to reverse y asix
                let ray = Ray {
                    point: scene.camera.position,
                    direction: x_global * scene.camera.right_axis
                        + y_global * scene.camera.up_axis
                        + scene.camera.forward_axis,
                };

                let color = scene
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
                        intersect(&ray_to_intersect, &primitive.shape)
                            .map(|intersection| (intersection, primitive.color.clone()))
                    })
                    .min_by(|x, y| x.0.partial_cmp(&y.0).expect("Nan on intersection."))
                    .map(|p| p.1)
                    .unwrap_or(scene.background_color);

                color_to_pixel(color)
            })
        })
        .collect()
}
