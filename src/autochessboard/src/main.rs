mod msg;
use chess::*;
use rosrust::{client, subscribe};
use msg::{lichess_api, board_arm_control, autochessboard};
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
            Error::InternalError => write!(f, "Internal error")
        }
    }
}


fn fen_to_board_display(fen: String) -> String {
    const WHITE_KING   : char = '\u{2654}';
    const WHITE_QUEEN  : char = '\u{2655}';
    const WHITE_ROOK   : char = '\u{2656}';
    const WHITE_BISHOP : char = '\u{2657}';
    const WHITE_KNIGHT : char = '\u{2658}';
    const WHITE_PAWN   : char = '\u{2659}';
    const BLACK_KING   : char = '\u{265A}';
    const BLACK_QUEEN  : char = '\u{265B}';
    const BLACK_ROOK   : char = '\u{265C}';
    const BLACK_BISHOP : char = '\u{265D}';
    const BLACK_KNIGHT : char = '\u{265E}';
    const BLACK_PAWN   : char = '\u{265F}';

    let mut display_string = String::new();
    let fen_parts: Vec<&str> = fen.split(' ').collect();
    let mut rank = b'8';

    display_string.push(char::from(rank));
    rank -= 1;

    for c in fen_parts[0].chars() {
        if c == '/' {
            display_string.push('\n');
            display_string.push(char::from(rank));
            rank -= 1;
        } else if c.is_ascii_digit() {
            let n = c.to_digit(10).unwrap();
            for _ in 0..n {
                display_string.push_str(" .");
            }
        } else if c.is_ascii_alphabetic() {
            display_string.push(' ');
            display_string.push(match c {
                'K' => WHITE_KING,
                'Q' => WHITE_QUEEN,
                'R' => WHITE_ROOK,
                'B' => WHITE_BISHOP,
                'N' => WHITE_KNIGHT,
                'P' => WHITE_PAWN,
                'k' => BLACK_KING,
                'q' => BLACK_QUEEN,
                'r' => BLACK_ROOK,
                'b' => BLACK_BISHOP,
                'n' => BLACK_KNIGHT,
                'p' => BLACK_PAWN,
                _   => '?'
            });
        }
    }

    display_string.push_str("\n ");
    for file in b'a'..b'i' {
        display_string.push(' ');
        display_string.push(char::from(file));
    }
    display_string
}


// Usage: challenge player [color="random"] [ai_level=1]
// If color is omitted, a random color is chosen. Color is required if
// specifying ai level
fn challenge(
    args: HashMap<String, Value>,
    thread_safe_context: &mut Arc<Mutex<Context>>)
    -> Result<Option<String>, Error> {

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
            let challenge_client = client::<lichess_api::ChallengeAI>(
                "/challenge_ai"
            ).expect("Could not contact /challenge_ai service");
            let challenge_response = challenge_client.req(
                &lichess_api::ChallengeAIReq {
                    level: ai_level,
                    color
                }
            ).unwrap().unwrap();
            rosrust::ros_debug!(
                "challenge response: {:#?}",
                challenge_response
            );
            context.game_id = challenge_response.id;
            context.game_ongoing = true;

            // Request a stream of game events given the game id we got from the
            // challenge
            let move_client = client::<lichess_api::StreamGameState>(
                "/stream_game"
            ).expect("Could not contact /stream_game service");
            let move_response = move_client.req(
                &lichess_api::StreamGameStateReq {
                    id: String::from(context.game_id.as_str())
                }
            ).unwrap().unwrap();
            rosrust::ros_debug!("move stream response: {:#?}", move_response);

            // If the AI has already made a move (being white), reflect them on
            // our board
            if move_response.initial_state.moves != "" {
                println!("non-empty moves");
                let split = move_response.initial_state.moves.split(' ');
                for move_str in split {
                    let m = ChessMove::from_san(
                        &context.game.current_position(),
                        move_str
                    ).expect("Invalid move");
                    if !context.game.make_move(m) {
                        return Err(Error::InternalError);
                    }
                }
            }

            result = Some(fen_to_board_display(
                context.game.current_position().to_string()
            ));
        }
        Ok(result)
    }
}


fn abort(
    _args: HashMap<String, Value>,
    thread_safe_context: &mut Arc<Mutex<Context>>)
    -> Result<Option<String>, Error> {

    let mut context = thread_safe_context.lock().unwrap();
    if context.game_ongoing {
        let client = client::<lichess_api::Abort>("/abort").unwrap();
        let response = client.req(
            &lichess_api::AbortReq {
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
        Err(Error::InvalidUsage(
            "Cannot abort while no game is ongoing".to_string()
        ))
    }
}


fn resign(
    _args: HashMap<String, Value>,
    thread_safe_context: &mut Arc<Mutex<Context>>)
    -> Result<Option<String>, Error> {

    let mut context = thread_safe_context.lock().unwrap();
    if context.game_ongoing {
        let client = client::<lichess_api::Resign>("/resign").unwrap();
        let response = client.req(
            &lichess_api::ResignReq {
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
        Err(Error::InvalidUsage(
            "Cannot resign while no game is ongoing".to_string()
        ))
    }
}


fn make_move(
    args: HashMap<String, Value>,
    thread_safe_context: &mut Arc<Mutex<Context>>)
    -> Result<Option<String>, Error> {

    let mut context = thread_safe_context.lock().unwrap();
    if context.game_ongoing {
        let piece_to_move: String = args
            .get("piece_to_move").unwrap()
            .convert().unwrap();
        let space_to_move: String = args
            .get("space_to_move").unwrap()
            .convert().unwrap();
        let chess_client = client::<lichess_api::MakeMove>("/make_move").unwrap();
        let board_client = client::<board_arm_control::MakeMove>("/board_arm_control/make_move").unwrap();
        context.pending_flag = false;

        // This will check that the move is valid for us
        let chess_response = chess_client.req(&lichess_api::MakeMoveReq {
            id: String::from(context.game_id.as_str()),
            move_: format!("{}{}", piece_to_move, space_to_move)
        }).unwrap().unwrap();

        if chess_response.success {
            let m = ChessMove::from_san(
                &context.game.current_position(),
                format!("{}{}", piece_to_move, space_to_move).as_str()
            ).expect("Invalid move");

            if context.game.make_move(m) {
                // This will cause the robot arm to move the pieces accordingly
                let board_response = board_client.req(
                    &board_arm_control::MakeMoveReq {
                        move_: format!("{}{}", piece_to_move, space_to_move)
                    }
                ).unwrap().unwrap();

                if !board_response.success {
                    rosrust::ros_debug!("board move response: {:#?}", board_response);
                    Err(Error::APIError(board_response.error))
                } else {
                    Ok(Some(fen_to_board_display(
                        context.game.current_position().to_string()
                    )))
                }
            } else {
                Err(Error::InternalError)
            }
        } else {
            rosrust::ros_debug!("chess move response: {:#?}", chess_response);
            Err(Error::APIError(chess_response.error))
        }
    } else {
        Err(Error::InvalidUsage(
            "Cannot make move while no game is ongoing".to_string()
        ))
    }
}


fn main() -> Result<(), Error> {
    rosrust::init("autochessboard");
    while !rosrust::is_initialized() {
        println!("not initialized");
        rosrust::rate(1.0).sleep();
    }

    // This context will be shared by all calls to the functions provided by
    // the REPL
    let thread_safe_context = Arc::new(Mutex::new(Context {
        game: Game::new(),
        game_ongoing: false,
        game_id: String::new(),
        pending_flag: false
    }));
    let callback_context_reference = Arc::clone(&thread_safe_context);

    let _event_sub = subscribe(
        "/game_event_stream",
        100,
        |v: lichess_api::GameEvent| {
            rosrust::ros_debug!("from game event stream: {:#?}", v);
        }
    ).unwrap();

    let _move_sub = subscribe(
        "/move_stream",
        100,
        move |game_state: lichess_api::GameState| {
            let mut context = callback_context_reference.lock().unwrap();
            // if the flag is false, then this move is one made by the user
            // otherwise it is one by the opponent, which we want to use to
            // update our board
            if game_state.status != "started" {
                rosrust::ros_debug!("game is not started");
            } else if !context.pending_flag {
                rosrust::ros_debug!("got users move");
                context.pending_flag = true;
            } else {
                rosrust::ros_debug!("got opponents move");
                context.pending_flag = false;
                let last_move = game_state.moves
                    .split(' ').last().unwrap();
                let m = ChessMove::from_san(
                    &context.game.current_position(),
                    last_move
                ).expect("Invalid move");

                if !context.game.make_move(m) {
                    rosrust::ros_err!("Error making move");
                } else {
                    println!("{}", fen_to_board_display(
                        context.game.current_position().to_string()
                    ));
                }
            }
        }
    ).unwrap();

    let _sensor_sub = subscribe(
        "/sensor_gradient",
        100,
        move |gradient: autochessboard::BoardGradient| {
            let mut board_gradient: [[i8; 8]; 8] = [[0; 8]; 8];
            for i in 0..8 {
               for j in 0..8 {
                   board_gradient[j][i] = gradient.gradient[i * 8 + j];
                   print!("{} ", board_gradient[i][j]);
               }
               println!("");
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
                .with_help(
                    "Usage: challenge player [color] [ai_level]\n\
                    challenge a particular player or an AI\n"
                )
        ).add_command(
            Command::new("abort", abort)
                .with_help("Usage: abort\nabort the current game\n")
        ).add_command(
            Command::new("resign", resign)
                .with_help("Usage: resign\nforfeit the current game\n")
        ).add_command(
            Command::new("move", make_move)
                .with_parameter(
                    Parameter::new("piece_to_move").set_required(true)?
                )?
                .with_parameter(
                    Parameter::new("space_to_move").set_required(true)?
                )?
                .with_help(
                    "Usage: move piece_to_move space_to_move\n\
                    make the specified move\n"
                )
        );

    let _ = repl.run();
    println!("REPL terminated; terminating program");

    rosrust::spin();
    Ok(())
}

/*
fn main() {
    let mut game = Game::new();
    let m1 = ChessMove::from_san(&game.current_position(), "Nc3").expect("error!");
    let success = game.make_move(m1);
    let m2 = ChessMove::from_san(&game.current_position(), "Nc6").expect("error 2!");
    println!("{}", m1);
    println!("{}", success);
    println!("{}", m2);
}
*/
