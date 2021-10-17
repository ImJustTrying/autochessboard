mod chess;
use chess::*;

use std::io::{self, Write};

fn parse_move(s: String, b: &Board) -> Option<Move> {
    let split = s.split(' ');
    let mut m: Move = Move {
        moving_piece: Piece::Empty,
        taken_piece: None,
        old_pos: (0,0),
        new_pos: (0,0),
        taken_pos: (0,0)
    };
    let mut num_parameters: u8 = 0;

    for pos in split {
        num_parameters += 1;

        // Map the chessboard notation to row and column indices
        let col = match pos.chars().nth(0) {
            Some('a') | Some('A') => Some(0),
            Some('b') | Some('B') => Some(1),
            Some('c') | Some('C') => Some(2),
            Some('d') | Some('D') => Some(3),
            Some('e') | Some('E') => Some(4),
            Some('f') | Some('F') => Some(5),
            Some('g') | Some('G') => Some(6),
            Some('h') | Some('H') => Some(7),
            _ => None
        };
        let row = match pos.chars().nth(1) {
            Some(c) => c.to_digit(10),
            None => None
        };

        // If all is valid, populate the Move structure with the appropriate data.
        if row == None || col == None || num_parameters > 2 {
            return None;
        } else {
            // This is so we can index into b.board[][]
            // Also notice we use "8 - row" since the board indices used by the user (and displayed
            // on the UI) are not the same used by the internal representation. They are flipped.
            // For example, "a1" on the board (the lower-left corner) has indices b[7][0].
            // Both the column index "0" and the left-most column "a" correspond, but the row index
            // "7" and the board index "1" do not.
            let row_index: usize = (8 - row.unwrap()) as usize;
            let col_index: usize = col.unwrap() as usize;

            if num_parameters == 1 {
                m.old_pos.0 = row_index as i8;
                m.old_pos.1 = col_index as i8;
                m.moving_piece = b.board[row_index][col_index];
            } else {
                m.new_pos.0 = row_index as i8;
                m.new_pos.1 = col_index as i8;
                m.taken_pos.0 = m.new_pos.0;
                m.taken_pos.1 = m.new_pos.1;
                m.taken_piece = match b.board[row_index][col_index] {
                    Piece::Empty => None,
                    _ => Some(b.board[row_index][col_index]) 
                };
            }
        }
    }

    Some(m)
}

fn main() {
    let mut board: Board = Default::default();

    while !board.game_over {
        println!("{}", board);
        print!("({}'s turn) Enter move > ", board.get_player_turn_str());
        io::stdout().flush().expect("IO error during flush");
        let mut input = String::new();

        match io::stdin().read_line(&mut input) {
            Ok(n) => {
                if n < 6 {
                    eprintln!("Invalid move: too few characters");
                } else {
                    let parsed: Option<Move> = parse_move(input, &board);
                    match parsed {
                        None => eprintln!("Failed to parse input"),
                        Some(mut m) => {
                            let successful_move: bool = board.make_move(&mut m);
                            println!("move: {}, move success: {}", m, successful_move);
                        }
                    }
                }
            },
            Err(e) => { println!("Error: {}", e) }
        }
    }
    let winner: &str = match board.get_player_turn_str() {
        "White" => "Black",
        "Black" => "White",
        _ => ""
    };
    println!("{} wins!", winner);
}
