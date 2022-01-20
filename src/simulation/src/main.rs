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
    let joint_torque_0: f64 = 30.0;
    let joint_torque_1: f64 = 30.0;

    while rosrust::is_ok() {
        let mut msg = rosrust_msg::sensor_msgs::JointState::default();
        msg.name.push(String::from("joint0"));
        msg.name.push(String::from("joint1"));
        msg.velocity.push(joint_torque_0);
        msg.velocity.push(joint_torque_1);
        publisher.send(msg).unwrap();
        rate.sleep();
    }
}
