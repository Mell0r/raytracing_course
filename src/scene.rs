use na::UnitQuaternion;
use na::Vector3;
use nalgebra::Quaternion;

use crate::geometry::Shape;

pub struct Camera {
    pub position: Vector3<f64>,
    pub right_axis: Vector3<f64>,
    pub up_axis: Vector3<f64>,
    pub forward_axis: Vector3<f64>,
    pub fov_x: f64,
    pub fov_y: f64,
}

pub enum Material {
    METALLIC,
    DIELECTRIC { ior: f64 },
    DIFFUSE,
}

pub struct Primitive {
    pub shape: Shape,
    pub color: Vector3<f64>,
    pub position: Vector3<f64>,
    pub rotation: UnitQuaternion<f64>,
    pub material: Material,
    pub emission: Vector3<f64>,
}

pub enum LightType {
    Point {
        position: Vector3<f64>,
        attenuation: Vector3<f64>,
    },
    Directed {
        direction: Vector3<f64>,
    },
}

pub struct Light {
    pub intensity: Vector3<f64>,
    pub ltype: LightType,
}

pub fn get_light_characteristic_to_point(
    light: &Light,
    point: &Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>, Option<f64>) {
    match light.ltype {
        LightType::Point {
            position,
            attenuation,
        } => {
            let direction_to_light = position - point;
            let r = direction_to_light.norm();
            (
                direction_to_light,
                light.intensity / (attenuation.x + attenuation.y * r + attenuation.z * r * r),
                Some(r),
            )
        }
        LightType::Directed { direction } => (direction, light.intensity, None),
    }
}

pub struct Scene {
    pub width: u32,
    pub height: u32,
    pub background_color: Vector3<f64>,
    pub camera: Camera,
    pub primitives: Vec<Primitive>,
    pub ray_depth: u32,
    pub ambient_light: Vector3<f64>,
    pub lights: Vec<Light>,
    pub samples: u32,
}

pub fn parse_scene(file_content: String) -> Scene {
    let mut width: Option<u32> = None;
    let mut height: Option<u32> = None;
    let mut background_color: Option<Vector3<f64>> = None;
    let mut position: Option<Vector3<f64>> = None;
    let mut right_axis: Option<Vector3<f64>> = None;
    let mut up_axis: Option<Vector3<f64>> = None;
    let mut forward_axis: Option<Vector3<f64>> = None;
    let mut fov_x: Option<f64> = None;
    let mut primitives: Vec<Primitive> = vec![];
    let mut ray_depth: Option<u32> = None;
    let mut ambient_light: Option<Vector3<f64>> = Some(Default::default());
    let mut lights: Vec<Light> = vec![];
    let mut samples: Option<u32> = None;

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
            "BG_COLOR" => background_color = Some(parse_vector3()),
            "CAMERA_POSITION" => position = Some(parse_vector3()),
            "CAMERA_RIGHT" => right_axis = Some(parse_vector3()),
            "CAMERA_UP" => up_axis = Some(parse_vector3()),
            "CAMERA_FORWARD" => forward_axis = Some(parse_vector3()),
            "CAMERA_FOV_X" => fov_x = Some(tokens[1].parse().expect("Input file format error.")),
            "NEW_PRIMITIVE" => primitives.push(Primitive {
                shape: Shape::Plane {
                    normal: Default::default(),
                },
                color: Default::default(),
                position: Default::default(),
                rotation: Default::default(),
                material: Material::DIFFUSE,
                emission: Default::default(),
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
            "METALLIC" => {
                primitives
                    .last_mut()
                    .expect("Input file format error.")
                    .material = Material::METALLIC
            }
            "DIELECTRIC" => {
                primitives
                    .last_mut()
                    .expect("Input file format error.")
                    .material = Material::DIELECTRIC {
                    ior: Default::default(),
                }
            }
            "IOR" => {
                primitives
                    .last_mut()
                    .expect("Input file format error.")
                    .material = Material::DIELECTRIC {
                    ior: tokens[1].parse().expect("Input file format error."),
                }
            }
            "RAY_DEPTH" => ray_depth = Some(tokens[1].parse().expect("Input file format error.")),
            "AMBIENT_LIGHT" => ambient_light = Some(parse_vector3()),
            "NEW_LIGHT" => lights.push(Light {
                intensity: Default::default(),
                ltype: LightType::Directed {
                    direction: Default::default(),
                },
            }),
            "LIGHT_INTENSITY" => {
                lights
                    .last_mut()
                    .expect("Input file format error.")
                    .intensity = parse_vector3()
            }
            "LIGHT_DIRECTION" => {
                lights.last_mut().expect("Input file format error.").ltype = LightType::Directed {
                    direction: parse_vector3(),
                }
            }
            "LIGHT_POSITION" => {
                lights.last_mut().expect("Input file format error.").ltype =
                    match lights.last_mut().expect("Input file format error.").ltype {
                        LightType::Directed { direction: _ } => LightType::Point {
                            position: parse_vector3(),
                            attenuation: Default::default(),
                        },
                        LightType::Point {
                            position: _,
                            attenuation,
                        } => LightType::Point {
                            position: parse_vector3(),
                            attenuation,
                        },
                    }
            }
            "LIGHT_ATTENUATION" => {
                lights.last_mut().expect("Input file format error.").ltype =
                    match lights.last_mut().expect("Input file format error.").ltype {
                        LightType::Directed { direction: _ } => LightType::Point {
                            position: Default::default(),
                            attenuation: parse_vector3(),
                        },
                        LightType::Point {
                            position,
                            attenuation: _,
                        } => LightType::Point {
                            position,
                            attenuation: parse_vector3(),
                        },
                    }
            }
            "SAMPLES" => samples = Some(tokens[1].parse().expect("Input file format error.")),
            "EMISSION" => {
                primitives
                    .last_mut()
                    .expect("Input file format error.")
                    .emission = parse_vector3()
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
            fov_y: 2.0 * ((fov_x / 2.0).tan() * height as f64 / width as f64).atan(),
        },
        primitives,
        ray_depth: ray_depth.expect("Ray depth is not specified in input file."),
        ambient_light: ambient_light.expect("Ambient light is not specified in input file."),
        lights,
        samples: samples.expect("Samples number is not specified in input file.")
    }
}
