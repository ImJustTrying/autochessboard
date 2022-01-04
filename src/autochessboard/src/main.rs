mod chess;
use chess::*;
use rosrust::{client, subscribe};
use rosrust_msg::{lichess_api};
use std::collections::HashMap;
use repl_rs::{Command, Parameter, Result, Value, Repl, Convert};

struct Context {
    board: Board,
    game_ongoing: bool
}

/*
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

fn run_game() {
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
*/

fn challenge(args: HashMap<String, Value>, context: &mut Context) -> Result<Option<String>> {
    let mut result = None;
    match args.get("player") {
        Some(p) => {
            let s: String = p.convert().unwrap();
            if s == "AI" || s == "ai" {
                let challenge_client = client::<lichess_api::ChallengeAI>("/challenge_ai_srv").unwrap();
                let challenge_response = challenge_client.req(
                    &rosrust_msg::lichess_api::ChallengeAIReq { level: 1 }
                ).unwrap().unwrap();
                rosrust::ros_debug!("challenge response: {:#?}", challenge_response);

                let move_client = client::<lichess_api::StreamGameState>("/game_stream_srv").unwrap();
                let move_response = move_client.req(
                    &rosrust_msg::lichess_api::StreamGameStateReq { id: challenge_response.id }
                ).unwrap().unwrap();
                rosrust::ros_debug!("move stream response: {:#?}", move_response);

                result = Some(format!("{:#?}", move_response));
            }
        },
        None => {}
    };
    Ok(result)
}

fn main() -> Result<()> {
    rosrust::init("rust_node");
    while !rosrust::is_initialized() {
        println!("not initialized");
        rosrust::rate(1.0).sleep();
    }
    let board: Board = Default::default();
    let context: Context = Context { board: board, game_ongoing: false };

    let event_sub = subscribe("/game_event_stream", 100, |v: lichess_api::GameEvent| {
        rosrust::ros_debug!("from game event stream: {:#?}", v);
    }).unwrap();
    let move_sub = subscribe("/move_stream", 100, |v: lichess_api::GameState| {
        rosrust::ros_debug!("from move stream: {:#?}", v);
    }).unwrap();

    let mut repl = Repl::new(context)
        .with_name("Autochessboard")
        .with_version("v0.1.0")
        .with_description("A physical interface for online chess play")
        .add_command(
            Command::new("challenge", challenge)
                .with_parameter(Parameter::new("player").set_required(true)?)?
                .with_parameter(Parameter::new("level").set_required(false)?)?
                .with_help("challenge a particular player or AI"),
        );

    let _ = repl.run();
    Ok(())
}
