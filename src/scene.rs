use na::UnitQuaternion;
use na::Vector3;
use nalgebra::Quaternion;

use crate::geometry::Shape;

pub struct Camera {
    pub position: Vector3<f32>,
    pub right_axis: Vector3<f32>,
    pub up_axis: Vector3<f32>,
    pub forward_axis: Vector3<f32>,
    pub fov_x: f32,
    pub fov_y: f32,
}

pub struct Primitive {
    pub shape: Shape,
    pub color: Vector3<f32>,
    pub position: Vector3<f32>,
    pub rotation: UnitQuaternion<f32>,
}

pub struct Scene {
    pub width: u32,
    pub height: u32,
    pub background_color: Vector3<f32>,
    pub camera: Camera,
    pub primitives: Vec<Primitive>,
}

pub fn parse_scene(file_content: String) -> Scene {
    let mut width: Option<u32> = None;
    let mut height: Option<u32> = None;
    let mut background_color: Option<Vector3<f32>> = None;
    let mut position: Option<Vector3<f32>> = None;
    let mut right_axis: Option<Vector3<f32>> = None;
    let mut up_axis: Option<Vector3<f32>> = None;
    let mut forward_axis: Option<Vector3<f32>> = None;
    let mut fov_x: Option<f32> = None;
    let mut primitives: Vec<Primitive> = vec![];

    for line in file_content.lines() {
        let tokens: Vec<String> = line.split_whitespace().map(|s| s.to_string()).collect();

        if tokens.len() == 0 {
            continue;
        }

        let parse_vector3 = || {
            Vector3::new(
                tokens[1].parse().expect("Input file format error."),
                tokens[2].parse().expect("Input file format error."),
                tokens[3].parse().expect("Input file format error."),
            )
        };

        match tokens[0].as_str() {
            "DIMENSIONS" => {
                width = Some(tokens[1].parse().expect("Input file format error."));
                height = Some(tokens[2].parse().expect("Input file format error."));
            }
            "BG_COLOR" => {
                background_color = Some(parse_vector3());
            }
            "CAMERA_POSITION" => {
                position = Some(parse_vector3());
            }
            "CAMERA_RIGHT" => {
                right_axis = Some(parse_vector3());
            }
            "CAMERA_UP" => {
                up_axis = Some(parse_vector3());
            }
            "CAMERA_FORWARD" => {
                forward_axis = Some(parse_vector3());
            }
            "CAMERA_FOV_X" => {
                fov_x = Some(tokens[1].parse().expect("Input file format error."));
            }
            "NEW_PRIMITIVE" => primitives.push(Primitive {
                shape: Shape::Plane {
                    normal: Default::default(),
                },
                color: Default::default(),
                position: Default::default(),
                rotation: Default::default(),
            }),
            "PLANE" => {
                primitives
                    .last_mut()
                    .expect("Input file format error.")
                    .shape = Shape::Plane {
                    normal: parse_vector3(),
                }
            }
            "ELLIPSOID" => {
                primitives
                    .last_mut()
                    .expect("Input file format error.")
                    .shape = Shape::Ellipsoid { r: parse_vector3() }
            }
            "BOX" => {
                primitives
                    .last_mut()
                    .expect("Input file format error.")
                    .shape = Shape::Box { s: parse_vector3() }
            }
            "POSITION" => {
                primitives
                    .last_mut()
                    .expect("Input file format error.")
                    .position = parse_vector3()
            }
            "ROTATION" => {
                primitives
                    .last_mut()
                    .expect("Input file format error.")
                    .rotation = UnitQuaternion::new_normalize(Quaternion::new(
                    tokens[4].parse().expect("Input file format error."),
                    tokens[1].parse().expect("Input file format error."),
                    tokens[2].parse().expect("Input file format error."),
                    tokens[3].parse().expect("Input file format error."),
                ))
            }
            "COLOR" => {
                primitives
                    .last_mut()
                    .expect("Input file format error.")
                    .color = parse_vector3()
            }
            _ => {}
        }
    }

    let width = width.expect("Width is not specified in input file.");
    let height = height.expect("Height is not specified in input file.");
    let fov_x = fov_x.expect("FOVx is not specified in input file.");

    Scene {
        width,
        height,
        background_color: background_color
            .expect("Background color is not specified in input file."),
        camera: Camera {
            position: position.expect("Position is not specified in input file."),
            right_axis: right_axis.expect("Right axis is not specified in input file."),
            up_axis: up_axis.expect("Up axis is not specified in input file."),
            forward_axis: forward_axis.expect("Forward axis is not specified in input file."),
            fov_x,
            fov_y: 2.0 * ((fov_x / 2.0).tan() * height as f32 / width as f32).atan(),
        },
        primitives,
    }
}
