use rosrust;
use std::f64::consts::PI;

fn main() {
    rosrust::init("trajectory_generator");
    while !rosrust::is_initialized() {
        println!("not initialized");
        rosrust::rate(1.0).sleep();
    }

    let publisher = rosrust::publish::<rosrust_msg::sensor_msgs::JointState>("joint_states_interpolated", 100).unwrap();
    let rate = rosrust::rate(144.0);
    let one_rev_per_sec: f64 = 2.0 * PI / 144.0;
    let mut joint_pos_0: f64 = 0.0;
    let mut joint_pos_1: f64 = 0.0;

    while rosrust::is_ok() {
        let mut msg = rosrust_msg::sensor_msgs::JointState::default();
        joint_pos_0 += one_rev_per_sec * 1.0/4.0;
        joint_pos_1 += one_rev_per_sec * 1.0/8.0;
        if joint_pos_0 >= 2.0 * PI {
            joint_pos_0 = 0.0;
        }
        if joint_pos_1 >= 2.0 * PI {
            joint_pos_1 = 0.0;
        }

        msg.name.push(String::from("joint0"));
        msg.name.push(String::from("joint1"));
        msg.position.push(joint_pos_0);
        msg.position.push(joint_pos_1);
        publisher.send(msg).unwrap();
        rate.sleep();
    }
}
