use chess::*;
use rosrust::{client, subscribe};
use rosrust_msg::{lichess_api};
use repl_rs::{Command, Parameter, Value, Repl, Convert};
use std::collections::HashMap;
use std::fmt;
use std::sync::{Arc, Mutex};
use std::result::Result;

struct Context {
    game: Game,
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


// Parse a move in the form of a string to a move struct.
fn parse_move(s: String, g: &Game) -> Option<Move> {
    let split = s.split(' ');
    let mut m: Move = Move {
        moving_piece: Piece::Empty,
        taken_piece: None,
        promotion: None,
        old_pos: (0,0),
        new_pos: (0,0),
        taken_pos: None,
        is_castle: false
    };
    let mut num_parameters: u8 = 0;
    let mut moving_is_white: bool = false;
    let b: &Board = &g.board;

    for pos in split {
        num_parameters += 1;
        if num_parameters <= 2 {
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
            if row == None || col == None {
                return None;
            }

            // This is so we can index into b.board[][]
            // Also notice we use "8 - row" since the board indices used by the user (and displayed
            // on the UI) are not the same used by the internal representation. They are flipped.
            // For example, "a1" on the board (the lower-left corner) has indices b[7][0].
            // Both the column index "0" and the left-most column "a" correspond, but the row index
            // "7" and the board index "1" do not.
            let row_index: usize = (8 - row.unwrap()) as usize;
            let col_index: usize = col.unwrap() as usize;
            let piece: Piece = b.get_piece(row_index, col_index).unwrap();

            if num_parameters == 1 { // first parameter
                m.old_pos.0 = row_index as i8;
                m.old_pos.1 = col_index as i8;
                m.moving_piece = piece;
                match m.moving_piece {
                    Piece::Queen{is_white} | Piece::King{is_white} | Piece::Rook{is_white} |
                    Piece::Bishop{is_white} | Piece::Knight{is_white} | Piece::Pawn{is_white} => {
                        moving_is_white = is_white;
                    }
                    Piece::Empty => { return None; },
                };
            }
            
            else if num_parameters == 2 { // second parameter
                let mut taken_pos = (0,0);
                m.new_pos.0 = row_index as i8;
                m.new_pos.1 = col_index as i8;
                if piece != Piece::Empty && same_color(&piece, &m.moving_piece) {
                    return None;
                }
                else if piece != Piece::Empty {
                    taken_pos.0 = m.new_pos.0;
                    taken_pos.1 = m.new_pos.1;
                    m.taken_piece = b.get_piece(row_index, col_index);
                }
                else if m.moving_piece == (Piece::King{is_white: true}) ||
                    m.moving_piece == (Piece::King{is_white: false}) {
                    // Check if this move is a castle
                    let on_same_row = m.new_pos.0 == m.old_pos.0;
                    let moved_two_spaces = (m.new_pos.1 - m.old_pos.0).abs() == 2;
                    m.is_castle = on_same_row == moved_two_spaces;
                }

                else if m.moving_piece == (Piece::Pawn{is_white: true}) ||
                    m.moving_piece == (Piece::Pawn{is_white: false}){
                    // Check if this move was an en passant move
                    let piece_is_pawn = piece == (Piece::Pawn{is_white: true}) || piece == (Piece::Pawn{is_white: false});
                    let correct_colors = !same_color(&piece, &m.moving_piece);
                    let taking_on_correct_row = if moving_is_white { m.new_pos.0 == 5 } else { m.new_pos.0 == 2 };
                    let last_move = match g.last_move() {
                        Some(l) => l,
                        None => { continue; }
                    };
                    let last_move_was_double =
                        (last_move.new_pos.1 - last_move.old_pos.1).abs() == 0 &&
                        (last_move.new_pos.0 - last_move.old_pos.1).abs() == 2;
                    let last_move_was_pawn = match last_move.moving_piece {
                        Piece::Pawn{..} => true,
                        _ => false
                    };
                    let correct_columns = (last_move.new_pos.1 - m.new_pos.1).abs() == 1;
                    let taking_is_behind_taken = (last_move.new_pos.0 - m.new_pos.0).abs() == 1;
                    if piece_is_pawn && correct_colors &&
                        taking_on_correct_row && last_move_was_double && last_move_was_pawn &&
                        correct_columns && taking_is_behind_taken {
                        m.taken_pos = Some(last_move.new_pos);
                        m.taken_piece = Some(last_move.moving_piece);
                    }
                }
            }
        }

        // This is the indicator for the piece we should promote a pawn to
        else if num_parameters == 3 {
            if m.moving_piece != (Piece::Pawn{is_white: true}) && m.moving_piece != (Piece::Pawn{is_white: false}) ||
                (m.new_pos.0 != 0 && m.new_pos.0 != 7) {
                return None;
            }

            let piece: Piece = match pos.chars().nth(0) {
                Some('q') | Some('Q') => Piece::Queen  {is_white: moving_is_white},
                Some('r') | Some('R') => Piece::Rook   {is_white: moving_is_white},
                Some('b') | Some('B') => Piece::Bishop {is_white: moving_is_white},
                Some('n') | Some('N') => Piece::Knight {is_white: moving_is_white},
                _ => Piece::Empty
            };
            if piece != Piece::Empty {
                m.promotion = Some(piece);
            }
        }
    }

    println!("{:#?}", m);
    if num_parameters != 2 && num_parameters != 3 {
        None
    } else {
        Some(m)
    }
}


// Usage: challenge player [color="random"] [ai_level=1]
// If color is omitted, a random color is chosen. Color is required if specifying ai level
fn challenge(args: HashMap<String, Value>, thread_safe_context: &mut Arc<Mutex<Context>>) -> Result<Option<String>, Error> {
    let mut context = thread_safe_context.lock().unwrap();
    if context.game_ongoing {
        Err(Error::InvalidUsage(String::from("Game is ongoing")))
    } else {
        let p = args.get("player").unwrap();
        let mut result = None;
        let player: String = p.convert().unwrap();
        let color: String = match args.get("color") {
            // Safe -- converting to string never fails
            Some(c) => c.convert().unwrap(),
            None => String::from("random")
        };
        let ai_level: u8 = match args.get("ai_level") {
            Some(l) => l.convert().unwrap(),
            None => 1
        };

        if player == "AI" || player == "ai" {
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
                    let mut m = match parse_move(s, &context.game) {
                        Some(m) => m,
                        None => { return Err(Error::SyntaxError); }
                    };
                    if !context.game.board.make_move(&mut m) {
                        return Err(Error::InternalError);
                    }
                }
            }

            result = Some(format!("{}", context.game.board));
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
        let client = client::<lichess_api::MakeMove>("/make_move").unwrap();
        context.pending_flag = false;
        let response = client.req(
            &rosrust_msg::lichess_api::MakeMoveReq {
                id: String::from(context.game_id.as_str()),
                move_: format!("{}{}", piece_to_move, space_to_move)
            }
        ).unwrap().unwrap();
        if response.success {
            let mut m = match parse_move(format!("{} {}", piece_to_move, space_to_move), &context.game) {
                Some(m) => m,
                None => {
                    return Err(Error::SyntaxError);
                }
            };
            if context.game.board.make_move(&mut m) {
                
                Ok(Some(format!("{}", context.game.board)))
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

    // This context will be shared by all calls to the functions provided by the REPL
    let thread_safe_context = Arc::new(Mutex::new(Context {
        game: Default::default(),
        game_ongoing: false,
        game_id: String::new(),
        pending_flag: false
    }));
    let callback_context_reference = Arc::clone(&thread_safe_context);

    let _event_sub = subscribe("/game_event_stream", 100, |v: lichess_api::GameEvent| {
        rosrust::ros_debug!("from game event stream: {:#?}", v);
    }).unwrap();

    let _move_sub = subscribe("/move_stream", 100, move |game_state: lichess_api::GameState| {
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
            let mut m = parse_move(last_move, &context.game).unwrap();
            if !context.game.board.make_move(&mut m) {
                rosrust::ros_err!("Error making move");
            } else {
                println!("{}", context.game.board);
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
