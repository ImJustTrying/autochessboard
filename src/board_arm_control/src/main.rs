use rosrust::{publish, param};
use rosrust_msg::std_msgs::Float64;
use std::fmt;
use std::io;
use std::io::Write;
use std::ops::Add;

#[derive(Debug,Copy,Clone)]
struct Vector2 {
    x1: f64,
    x2: f64
}
impl Vector2 {
    pub fn new(x1: f64, x2: f64) -> Vector2 {
        Vector2 {
            x1,
            x2
        }
    }
}
impl fmt::Display for Vector2 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({}, {})", self.x1, self.x2)
    }
}
impl Add for Vector2 {
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self {
            x1: self.x1 + other.x1,
            x2: self.x2 + other.x2
        }
    }
}


// converts from a lattice coordinate system (with the origin at the bottom
// left of the 5x5 board, extending to (20,20)) to real space coordinates.
fn lattice_to_real_coord(x1: i8, x2: i8) -> Vector2 {
    // TODO: Rename parameter to avoid name collision, e.g.
    // "/simulation/constants/tile_size"
    let real_tile_size: f64 = param("tile_size").unwrap()
        .get::<f64>().unwrap();
    let lattice_tile_size: f64 = real_tile_size / 2.0;
    let origin = Vector2::new(-5.0 * real_tile_size, -5.0 * real_tile_size);
    let offset = Vector2::new(
        lattice_tile_size * f64::from(x1),
        lattice_tile_size * f64::from(x2)
    );
    origin + offset
}


fn forward_kinematics(config: Vector2) -> Vector2 {
    let l : f64 = param("link_length").unwrap().get::<f64>().unwrap();
    let x1 = l * (config.x1).cos() + l * (config.x1 + config.x2).cos();
    let x2 = l * (config.x1).sin() + l * (config.x1 + config.x2).sin();
    Vector2::new(
        // Truncate values very close to 0
        if x1.abs() < 1e-10 { 0.0 } else { x1 },
        if x2.abs() < 1e-10 { 0.0 } else { x2 }
    )
}


fn inverse_kinematics(config: Vector2) -> Vector2 {
    let l : f64 = param("/link_length").unwrap().get::<f64>().unwrap();
    println!("l = {}", l);
    let dist_squared = config.x1 * config.x1 + config.x2 * config.x2;
    // derived using the law of cosines, trigonometry
    let x2 = (dist_squared / (2.0 * l * l) - 1.0).acos();
    let x1 = config.x2.atan2(config.x1) - x2 / 2.0;

    Vector2::new(
        if x1.abs() < 1e-10 { 0.0 } else { x1 },
        if x2.abs() < 1e-10 { 0.0 } else { x2 }
    )
}


// start & end are specified in lattice space coordinates
fn find_path(start: (i8, i8), end: (i8, i8)) -> Vec<Vector2> {
    let mut lattice_path = vec![start];
    let mut lattice_dist = (end.0 - start.0, end.1 - start.1);
    let mut current_pos = start;
    let direction = (
        if lattice_dist.0 > 0 { 1 } else { -1 },
        if lattice_dist.1 > 0 { 1 } else { -1 }
    );

    while current_pos.0 != end.0 || current_pos.1 != end.1 {
        if lattice_dist.0.abs() > lattice_dist.1.abs() {
            current_pos.0 += direction.0;
            lattice_dist.0 -= direction.0;
        } else {
            current_pos.1 += direction.1;
            lattice_dist.1 -= direction.1;
        }
        lattice_path.push(current_pos);
    }

    lattice_path.iter().map(|pos| {
        println!("({}, {})", pos.0, pos.1);
        lattice_to_real_coord(pos.0, pos.1)
    }).collect()
}


fn main() -> io::Result<()> {
    rosrust::init("autochessboard");

    /*
    let _j1_sub = subscribe("/board_arm/joint1_position_controller/state", 1,
        |s: control_msgs::JointControllerState| {
            rosrust::ros_debug!("from joint 1 controller: {:#?}", s);
        }
    ).unwrap();

    let _j2_sub = subscribe("/board_arm/joint2_position_controller/state", 1,
        |s: control_msgs::JointControllerState| {
            rosrust::ros_debug!("from joint 2 controller: {:#?}", s);
        }
    ).unwrap();
    */

    let j1_pub = publish::<Float64>
        ("/board_arm/joint1_position_controller/command", 100)
        .expect("Failed to create joint1 command publisher");
    let j2_pub = publish::<Float64>
        ("/board_arm/joint2_position_controller/command", 100)
        .expect("Failed to create joint2 command publisher");

    let rate = rosrust::rate(1.0);
    let stdin = io::stdin();
    let mut stdout = io::stdout();
    let mut input = String::new();

    while rosrust::is_ok() {
        // Read in desired position
        print!("start x1: ");
        stdout.flush()?;
        stdin.read_line(&mut input)?;
        let start1 = input[..input.len()-1].parse::<i8>();
        if let Err(_) = start1 {
            println!("Invalid float; terminating");
            break;
        }
        input.clear();

        print!("start x2: ");
        stdout.flush()?;
        stdin.read_line(&mut input)?;
        let start2 = input[..input.len()-1].parse::<i8>();
        if let Err(_) = start2 {
            println!("Invalid float; terminating");
            break;
        }
        input.clear();

        print!("end x1: ");
        stdout.flush()?;
        stdin.read_line(&mut input)?;
        let end1 = input[..input.len()-1].parse::<i8>();
        if let Err(_) = end1 {
            println!("Invalid float; terminating");
            break;
        }
        input.clear();

        print!("end x2: ");
        stdout.flush()?;
        stdin.read_line(&mut input)?;
        let end2 = input[..input.len()-1].parse::<i8>();
        if let Err(_) = end2 {
            println!("Invalid float; terminating");
            break;
        }

        let path = find_path(
            (start1.unwrap(), start2.unwrap()),
            (end1.unwrap(), end2.unwrap())
        );

        for position in path {
            println!("Position: {}", position);
            let config = inverse_kinematics(position);
            let mut j1_msg = Float64::default();
            let mut j2_msg = Float64::default();
            j1_msg.data = config.x1;
            j2_msg.data = config.x2;
            j1_pub.send(j1_msg).unwrap();
            j2_pub.send(j2_msg).unwrap();
            rate.sleep();
        }
        break;
    }

    Ok(())
}
