mod chess;
use chess::Board;

use std::io;

fn main() {
    let board: Board = Board::init();
    println!("{}", board);

    let mut input = String::new();
    match io::stdin().read_line(&mut input) {
        Ok(n) => { println!("String read ({} bytes long): {}", n, input) },
        Err(e) => { println!("Error: {}", e) }
    }
}
