# autochessboard
This is a robotics passion project.
This repository holds a catkin workspace which contains numerous ROS packages.
The project itself is an automated chessboard which serves as a physical interface to an online chess game.
That is, when you make a move on the physical board, that move is forwarded to your online opponent.
When they make a move, an arm under the board moves the pieces that the opponent did, also considering captures and promotions.

# packages
The following is a breakdown of the various ROS packages in this workspace
- `lichess_api`
  * This package contains a nodejs program that will advertise ROS services for various actions that the user makes using the online chess service
  * Usage: `rosrun lichess_api lichess.js`
- `autochessboard`
  * This is a rust package that provides a REPL interface for performing actions on the chess service
  * `chess.rs` contains type definitions and procedures for chess pieces, the chessboard, etc.
  * `main.rs` contains the REPL code, and uses rust's ROS client library to call on the services from `lichess_api`
  * Usage: `rosrun autochessboard autochessboard`
- `simulation`
  * As the name implies, this package will contain code for seperately launch `rviz` visulizations and `gazebo` physical simulations of the arm
  * URDF files and `gazebo` world files are stored here
  * Usage: `roslaunch simulation rviz.launch` or `roslaunch simulation board_arm.launch`
- `board_arm_control`
  * Configuration files for the `ros-control` package are contained in this package
  * Rust code for interfacing with GPIO pins (e.g. the ones on a RaspberryPI) for sending signals to physical actuators
  * Usage: `roslaunch board_arm_control board_arm_control.launch` or (not yet implemented) `rosrun board_arm_control gpio`

# dependencies
- [ROS 1](https://ros.org/)
  * Specifically the `ros-*-desktop-full` variant
  * Tested on melodic & noetic
- [Gazebo](http://gazebosim.org/)
  * Tested on versions 9 & 11
- [`ros-control` &  `ros-controllers`](http://wiki.ros.org/ros_control)
  * Use `sudo apt install ros-control ros-controllers`
- [NodeJS](https://nodejs.org/) & [`npm`](https://www.npmjs.com/)
  * Tested on node LTS (16.13.2) & npm version 8.1.2
- [`rosnodejs`](https://www.npmjs.com/package/rosnodejs) & [`node-fetch`](https://www.npmjs.com/package/node-fetch)
  * Use `npm install rosnodejs node-fetch`
- [Rust](https://rust-lang.org)
  * Tested on rust 2018 & 2021 edition
