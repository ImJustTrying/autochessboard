use rosrust::{sleep, Duration, publish, param, subscribe};
use rosrust_msg::{board_arm_control, std_msgs};
use rosrust_msg::control_msgs::JointControllerState;

use std::f64::consts::PI;
use std::fmt;
use std::io;
use std::num::ParseIntError;
use std::ops::Add;
use std::sync::{Arc, Mutex, Condvar};


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
    let dist_squared = config.x1 * config.x1 + config.x2 * config.x2;
    // derived using the law of cosines, trigonometry
    let x2 = (dist_squared / (2.0 * l * l) - 1.0).acos();
    let x1 = config.x2.atan2(config.x1) - x2 / 2.0;

    Vector2::new(
        if x1.abs() < 1e-10 { 0.0 } else { x1 },
        if x2.abs() < 1e-10 { 0.0 } else { x2 }
    )
}


// start and end are specified in lattice space coordinates
fn find_path(start: (i8, i8), end: (i8, i8)) -> Vec<Vector2> {
    if start.0 == end.0 && start.1 == end.1 { return vec![]; }
    let mut lattice_path = vec![start];
    let mut lattice_dist = (end.0 - start.0, end.1 - start.1);
    let mut current_pos = start;
    let direction = (
        if lattice_dist.0 > 0 { 1 } else { -1 },
        if lattice_dist.1 > 0 { 1 } else { -1 }
    );

    // Move the arm in between tiles before path finding
    if current_pos.0 % 2 == 1 {
        current_pos.0 += direction.0;
        lattice_dist.0 -= direction.0;
    }
    if current_pos.1 % 2 == 1 {
        current_pos.1 += direction.1;
        lattice_dist.1 -= direction.1;
    }
    lattice_path.push(current_pos);

    while lattice_dist.0.abs() > 1 || lattice_dist.1.abs() > 1 {
        if lattice_dist.0.abs() > lattice_dist.1.abs() {
            // double the distance moved to remain inbetween tiles
            current_pos.0 += 2 * direction.0;
            lattice_dist.0 -= 2 * direction.0;
        } else {
            current_pos.1 += 2 * direction.1;
            lattice_dist.1 -= 2 * direction.1;
        }
        lattice_path.push(current_pos);
    }

    println!("distance: ({}, {})", lattice_dist.0, lattice_dist.1);
    if lattice_dist.0.abs() == 1 {
        current_pos.0 += direction.0;
        lattice_dist.0 -= direction.0;
    }
    if lattice_dist.1.abs() == 1 {
        current_pos.1 += direction.1;
        lattice_dist.1 -= direction.1;
    }
    lattice_path.push(current_pos);

    lattice_path.iter().map(|pos| {
        println!("({}, {})", pos.0, pos.1);
        lattice_to_real_coord(pos.0, pos.1)
    }).collect()
}


fn main() -> io::Result<()> {
    rosrust::init("board_arm_control_node");

    let _service = rosrust::service::<board_arm_control::MakeMove, _>(
        "board_arm_control/make_move",
        move |req| {
            let mut chars = req.move_.chars();
            let letter_to_num = |c: char| -> i8 {
                match c {
                    'a' => 3,
                    'b' => 5,
                    'c' => 7,
                    'd' => 9,
                    'e' => 11,
                    'f' => 13,
                    'g' => 15,
                    'h' => 17,
                     _  => 0
                }
            };

            let start = (
                letter_to_num(chars.next().unwrap()),
                chars.next().unwrap().to_digit(10).unwrap() as i8
            );
            let end = (
                letter_to_num(chars.next().unwrap()),
                chars.next().unwrap().to_digit(10).unwrap() as i8
            );

            println!("({}, {}), ({}, {})", start.0, start.1, end.0, end.1);
            Ok(board_arm_control::MakeMoveRes {
                success: true,
                error: String::from("")
            })
        }
    ).unwrap();

    let dist_criteria = Arc::new((Mutex::new((false, false)), Condvar::new()));
    let j1_criteria = Arc::clone(&dist_criteria);
    let j2_criteria = Arc::clone(&dist_criteria);

    let _j1_sub = subscribe("/board_arm/joint1_position_controller/state", 1,
        move |s: JointControllerState| {
            let commanded =
                if s.set_point < 0.0 { s.set_point + 2.0 * PI }
                else { s.set_point };
            let position =
                if s.process_value < 0.0 { s.process_value + 2.0 * PI }
                else { s.process_value };

            if position - commanded < 0.1 {
                let (lock, cvar) = &*j1_criteria;
                let mut pending = lock.lock().unwrap();
                (*pending).0 = true;
                cvar.notify_one();
            }
        }
    ).expect("Failed to subscribe to joint1 position controller");

    let _j2_sub = subscribe("/board_arm/joint2_position_controller/state", 1,
        move |s: JointControllerState| {
            let commanded =
                if s.set_point < 0.0 { s.set_point + 2.0 * PI }
                else { s.set_point };
            let position =
                if s.process_value < 0.0 { s.process_value + 2.0 * PI }
                else { s.process_value };

            if position - commanded < 0.1 {
                let (lock, cvar) = &*j2_criteria;
                let mut pending = lock.lock().unwrap();
                (*pending).1 = true;
                cvar.notify_one();
            }
        }
    ).expect("Failed to subscribe to joint2 position controller");

    let _j1_pub = publish::<std_msgs::Float64>
        ("/board_arm/joint1_position_controller/command", 100)
        .expect("Failed to create joint1 command publisher");
    let _j2_pub = publish::<std_msgs::Float64>
        ("/board_arm/joint2_position_controller/command", 100)
        .expect("Failed to create joint2 command publisher");

    rosrust::spin();
    /*
    let stdin = io::stdin();
    let mut stdout = io::stdout();

    let read_input = || -> Result<i8, ParseIntError> {
        let mut buffer = String::new();
        stdin.read_line(&mut buffer).unwrap();
        buffer[..buffer.len()-1].parse::<i8>()
    };

    while rosrust::is_ok() {
        // Read in desired position
        print!("start x1: ");
        stdout.flush()?;
        let start1 = match read_input() {
            Ok(i) => i,
            Err(_) => { break; }
        };

        print!("start x2: ");
        stdout.flush()?;
        let start2 = match read_input() {
            Ok(i) => i,
            Err(_) => { eprintln!("Error"); break; }
        };

        print!("end x1: ");
        stdout.flush()?;
        let end1 = match read_input() {
            Ok(i) => i,
            Err(_) => { break; }
        };

        print!("end x2: ");
        stdout.flush()?;
        let end2 = match read_input() {
            Ok(i) => i,
            Err(_) => { break; }
        };

        let path = find_path(
            (start1, start2),
            (end1, end2)
        );

        let (lock, cvar) = &*dist_criteria;
        for position in path {
            println!("Position: {}", position);
            let config = inverse_kinematics(position);
            let mut j1_msg = std_msgs::Float64::default();
            let mut j2_msg = std_msgs::Float64::default();
            j1_msg.data = config.x1;
            j2_msg.data = config.x2;
            j1_pub.send(j1_msg).unwrap();
            j2_pub.send(j2_msg).unwrap();

            // Wait for arm to be sufficiently close to commanded position
            println!("Blocking!");
            let _guard = cvar.wait_while(lock.lock().unwrap(), |pending| {
                if (*pending).0 && (*pending).1 {
                    (*pending).0 = false;
                    (*pending).1 = false;
                    false
                } else {
                    true
                }
            }).unwrap();
            println!("Unblocking!");
            sleep(Duration::from_seconds(1));
        }
        break;
    }
    */

    Ok(())
}
