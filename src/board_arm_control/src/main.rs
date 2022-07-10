use rosrust::{subscribe, publish, param};
use rosrust_msg::std_msgs::Float64;
use std::f64::consts::FRAC_PI_2;
use std::fmt;
use std::io;
use std::io::Write;

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


fn forward_kinematics(config: Vector2) -> Vector2 {
    let l : f64 = rosrust::param("/link_length").unwrap().get::<f64>().unwrap();
    let x1 = l * (config.x1).cos() + l * (config.x1 + config.x2).cos();
    let x2 = l * (config.x1).sin() + l * (config.x1 + config.x2).sin();
    Vector2 {
        // Truncate values very close to 0
        x1 : if x1.abs() < 1e-10 { 0.0 } else { x1 },
        x2 : if x2.abs() < 1e-10 { 0.0 } else { x2 }
    }
}

fn inverse_kinematics(config: Vector2) -> Vector2 {
    let l : f64 = rosrust::param("/link_length").unwrap().get::<f64>().unwrap();
    println!("l = {}", l);
    let dist_squared = config.x1 * config.x1 + config.x2 * config.x2;
    // derived using the law of cosines, trigonometry
    let x2 = (dist_squared / (2.0 * l * l) - 1.0).acos();
    let inverse_tangent : f64 =
        if config.x1 == 0.0 {
            if config.x2 > 0.0 {
                FRAC_PI_2
            } else if config.x2 < 0.0 {
                -FRAC_PI_2
            } else {
                0.0
            }
        } else {
            config.x2.atan2(config.x1)
        };
    let x1 = inverse_tangent - x2 / 2.0;

    Vector2 {
        x1 : if x1.abs() < 1e-10 { 0.0 } else { x1 },
        x2 : if x2.abs() < 1e-10 { 0.0 } else { x2 }
    }
}

fn main() -> io::Result<()> {
    rosrust::init("autochessboard");
    while !rosrust::is_initialized() {
        println!("not initialized");
        rosrust::rate(1.0).sleep();
    }

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

    let j1_pub = publish("/board_arm/joint1_position_controller/command", 100)
        .expect("Failed to create joint1 command publisher");
    let j2_pub = publish("/board_arm/joint2_position_controller/command", 100)
        .expect("Failed to create joint2 command publisher");

    let rate = rosrust::rate(1.0/5.0);
    let stdin = io::stdin();
    let mut stdout = io::stdout();

    while rosrust::is_ok() {
        // Read in desired position
        let mut input = String::new();
        print!("x1: ");
        stdout.flush()?;
        stdin.read_line(&mut input)?;
        println!("{}", input);
        let x1 = input[..input.len()-1].parse::<f64>().unwrap();
        input.clear();
        print!("x2: ");
        stdout.flush()?;
        stdin.read_line(&mut input)?;
        println!("{}", input);
        let x2 = input[..input.len()-1].parse::<f64>().unwrap();

        // compute configuration for 
        let position = Vector2::new(x1, x2);
        let config = inverse_kinematics(position);
        println!("Config: {:#?}", config);

        let mut j1_msg = Float64::default();
        let mut j2_msg = Float64::default();
        j1_msg.data = config.x1;
        j2_msg.data = config.x2;
        j1_pub.send(j1_msg).unwrap();
        j2_pub.send(j2_msg).unwrap();
        rate.sleep();
    }

    rosrust::spin();
    Ok(())
}
