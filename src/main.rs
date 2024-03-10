mod geometry;
mod rendering;
mod scene;
mod distribution;

extern crate nalgebra as na;
use std::env;
use std::fs;
use std::io::Write;

use image::ImageFormat;
use image::RgbImage;

use rendering::render_scene;
use scene::parse_scene;

fn main() {
    let args: Vec<String> = env::args().collect();

    let scene_path = &args[1];
    let output_path = &args[2];

    let scene = parse_scene(fs::read_to_string(scene_path).expect("No scene scene file provided."));

    let rendered_scene = render_scene(&scene);
    dump_to_ppm(scene.height, scene.width, &rendered_scene, output_path);
}

fn dump_to_png(height: u32, width: u32, rendered_scene: &Vec<u8>, output_path: &String) {
    let mut image = RgbImage::new(width, height);
    for x in 0..width {
        for y in 0..height {
            for i in 0..3 {
                image.get_pixel_mut(x as u32, y as u32).0[i] =
                    rendered_scene[(y * width * 3 + x * 3) as usize + i];
            }
        }
    }
    image
        .save_with_format(output_path, ImageFormat::Png)
        .unwrap();
}

fn dump_to_ppm(height: u32, width: u32, rendered_scene: &Vec<u8>, output_path: &String) {
    let mut output_file = fs::OpenOptions::new()
        .write(true)
        .append(true)
        .create(true)
        .open(output_path)
        .unwrap();
    output_file.write(b"P6\n").unwrap();
    output_file
        .write(format!("{} {}\n", width, height).as_bytes())
        .unwrap();
    output_file.write(b"255\n").unwrap();
    output_file.write(rendered_scene.as_slice()).unwrap();
}
