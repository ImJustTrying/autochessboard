mod chess;
use chess::*;
use rosrust::{client, subscribe};
use rosrust_msg::{lichess_api};
use repl_rs::{Command, Parameter, Value, Repl, Convert};
use std::collections::HashMap;
use std::fmt;
use std::sync::{Arc, Mutex};
use std::result::Result;

struct Context {
    board: Board,
    game_ongoing: bool,
    game_id: String,
    pending_flag: bool
}

// Define error type for REPL
#[derive(Debug)]
enum Error {
    ReplError(repl_rs::Error),
    InvalidUsage(String),
    APIError(String),
    SyntaxError,
    InternalError
}

// For converting from my error type to the REPL's
impl From<repl_rs::Error> for Error {
    fn from(error: repl_rs::Error) -> Self {
        Error::ReplError(error)
    }
}

// Implement Display so the REPL can print my custom error types
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> Result<(), fmt::Error> {
        match self {
            Error::ReplError(e) => write!(f, "{}", e),
            Error::InvalidUsage(s) => write!(f, "InvalidUsage: {}", s),
            Error::APIError(s) => write!(f, "APIError: {}", s),
            Error::SyntaxError => write!(f, "Syntax error"),
            Error::InternalError => write!(f, "Internal error")
        }
    }
}


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

        // Map the chess notation to row and column indices
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

/*
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


// Usage: challenge player [color="random"] [ai_level=1]
// If color is omitted, a random color is chosen. Color is required if specifying ai level
fn challenge(args: HashMap<String, Value>, thread_safe_context: &mut Arc<Mutex<Context>>) -> Result<Option<String>, Error> {
    let mut context = thread_safe_context.lock().unwrap();
    if context.game_ongoing {
        Err(Error::InvalidUsage(String::from("Game is ongoing")))
    } else {
        let p = args.get("player").unwrap();
        let mut result = None;
        let s: String = p.convert().unwrap();
        let color: String = match args.get("color") {
            // Safe -- converting to string never fails
            Some(c) => c.convert().unwrap(),
            None => String::from("random")
        };
        let ai_level: u8 = match args.get("ai_level") {
            Some(l) => l.convert().unwrap(),
            None => 1
        };

        if s == "AI" || s == "ai" {
            // Request for the challenge itself
            let challenge_client = client::<lichess_api::ChallengeAI>("/challenge_ai").unwrap();
            let challenge_response = challenge_client.req(
                &rosrust_msg::lichess_api::ChallengeAIReq {
                    level: ai_level,
                    color: color
                }
            ).unwrap().unwrap();
            rosrust::ros_debug!("challenge response: {:#?}", challenge_response);
            context.game_id = challenge_response.id;
            context.game_ongoing = true;

            // Request a stream of game events given the game id we got from the challenge
            let move_client = client::<lichess_api::StreamGameState>("/stream_game").unwrap();
            let move_response = move_client.req(
                &rosrust_msg::lichess_api::StreamGameStateReq {
                    id: String::from(context.game_id.as_str())
                }
            ).unwrap().unwrap();
            rosrust::ros_debug!("move stream response: {:#?}", move_response);

            // If the AI has already made a move (being white), reflect them on our board
            if move_response.initial_state.moves != "" {
                println!("non-empty moves");
                let split = move_response.initial_state.moves.split(' ');
                for move_str in split {
                    let mut s = move_str.to_owned();
                    s.insert(2, ' ');
                    let mut m = match parse_move(s, &context.board) {
                        Some(m) => m,
                        None => { return Err(Error::SyntaxError); }
                    };
                    if !context.board.make_move(&mut m) {
                        return Err(Error::InternalError);
                    }
                }
            }

            result = Some(format!("{}", context.board));
        }
        Ok(result)
    }
}


fn abort(_args: HashMap<String, Value>, thread_safe_context: &mut Arc<Mutex<Context>>) -> Result<Option<String>, Error> {
    let mut context = thread_safe_context.lock().unwrap();
    if context.game_ongoing {
        let client = client::<lichess_api::Abort>("/abort").unwrap();
        let response = client.req(
            &rosrust_msg::lichess_api::AbortReq {
                id: String::from(context.game_id.as_str())
            }
        ).unwrap().unwrap();
        if response.success {
            context.game_id = String::from("");
            context.game_ongoing = false;
            Ok(Some("Game successfully aborted".to_string()))
        } else {
            rosrust::ros_debug!("abort response: {:#?}", response);
            Err(Error::APIError(response.error))
        }
    } else {
        Err(Error::InvalidUsage("Cannot abort while no game is ongoing".to_string()))
    }
}


fn resign(_args: HashMap<String, Value>, thread_safe_context: &mut Arc<Mutex<Context>>) -> Result<Option<String>, Error> {
    let mut context = thread_safe_context.lock().unwrap();
    if context.game_ongoing {
        let client = client::<lichess_api::Resign>("/resign").unwrap();
        let response = client.req(
            &rosrust_msg::lichess_api::ResignReq {
                id: String::from(context.game_id.as_str())
            }
        ).unwrap().unwrap();
        if response.success {
            context.game_id = String::from("");
            context.game_ongoing = false;
            Ok(Some("Game successfully forfeited".to_string()))
        } else {
            rosrust::ros_debug!("resign response: {:#?}", response);
            Err(Error::APIError(response.error))
        }
    } else {
        Err(Error::InvalidUsage("Cannot resign while no game is ongoing".to_string()))
    }
}


fn make_move(args: HashMap<String, Value>, thread_safe_context: &mut Arc<Mutex<Context>>) -> Result<Option<String>, Error> {
    let mut context = thread_safe_context.lock().unwrap();
    if context.game_ongoing {
        let piece_to_move: String = args.get("piece_to_move").unwrap().convert().unwrap();
        let space_to_move: String = args.get("space_to_move").unwrap().convert().unwrap();
        let mut m = match parse_move(format!("{} {}", piece_to_move, space_to_move), &context.board) {
            Some(m) => m,
            None => {
                return Err(Error::SyntaxError);
            }
        };
        let client = client::<lichess_api::MakeMove>("/make_move").unwrap();
        context.pending_flag = false;
        let response = client.req(
            &rosrust_msg::lichess_api::MakeMoveReq {
                id: String::from(context.game_id.as_str()),
                move_: format!("{}{}", piece_to_move, space_to_move)
            }
        ).unwrap().unwrap();
        if response.success {
            if context.board.make_move(&mut m) {
                Ok(Some(format!("{}", context.board)))
            } else {
                rosrust::ros_debug!("make move response: {:#?}", response);
                Err(Error::InternalError)
            }
        } else {
            rosrust::ros_debug!("make move response: {:#?}", response);
            Err(Error::APIError(response.error))
        }
    } else {
        Err(Error::InvalidUsage("Cannot make move while no game is ongoing".to_string()))
    }
}


fn main() -> Result<(), Error> {
    rosrust::init("autochessboard");
    while !rosrust::is_initialized() {
        println!("not initialized");
        rosrust::rate(1.0).sleep();
    }

    let thread_safe_context = Arc::new(Mutex::new(Context {
        board: Default::default(),
        game_ongoing: false,
        game_id: String::new(),
        pending_flag: false
    }));
    let callback_context_reference = Arc::clone(&thread_safe_context);

    let _event_sub = subscribe("/game_event_stream", 100, |v: lichess_api::GameEvent| {
        rosrust::ros_debug!("from game event stream: {:#?}", v);
    }).unwrap();

    let _move_sub = subscribe("/move_stream", 100, move |game_state: lichess_api::GameState| {
        //rosrust::ros_debug!("from move stream: {:#?}", game_state);
        let mut context = callback_context_reference.lock().unwrap();
        // if the flag is false, then this move is one made by the user
        // otherwise it is one by the opponent, which we want to use to update our board
        if game_state.status != "started" {
            rosrust::ros_debug!("game is not started");
        } else if !context.pending_flag {
            rosrust::ros_debug!("got users move");
            context.pending_flag = true;
        } else {
            rosrust::ros_debug!("got opponents move");
            context.pending_flag = false;
            let mut last_move = game_state.moves.split(' ').last().unwrap().to_owned();
            last_move.insert(2, ' ');
            let mut m = parse_move(last_move, &context.board).unwrap();
            if !context.board.make_move(&mut m) {
                rosrust::ros_err!("Error making move");
            } else {
                println!("{}", context.board);
            }
        }
    }).unwrap();

    let mut repl = Repl::new(thread_safe_context)
        .with_name("Autochessboard")
        .with_version("v0.1.0")
        .with_description("A physical interface for online chess play")
        .add_command(
            Command::new("challenge", challenge)
                .with_parameter(Parameter::new("player").set_required(true)?)?
                .with_parameter(Parameter::new("color").set_required(false)?)?
                .with_parameter(Parameter::new("level").set_required(false)?)?
                .with_help("Usage: challenge player [color] [ai_level]\nchallenge a particular player or an AI\n")
        ).add_command(
            Command::new("abort", abort)
                .with_help("Usage: abort\nabort the current game\n")
        ).add_command(
            Command::new("resign", resign)
                .with_help("Usage: resign\nforfeit the current game\n")
        ).add_command(
            Command::new("move", make_move)
                .with_parameter(Parameter::new("piece_to_move").set_required(true)?)?
                .with_parameter(Parameter::new("space_to_move").set_required(true)?)?
                .with_help("Usage: move piece_to_move space_to_move\nmake the specified move\n")
        );

    let _ = repl.run();
    println!("after repl run");
    Ok(())
}
