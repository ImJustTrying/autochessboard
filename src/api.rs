use serde_json::{Value, from_str, from_reader, Deserializer};
use std::io::{stdin, BufReader};

// Reference: https://lichess.org/api
#[derive(Deserialize, Debug)]
struct Variant {
    key: String,
    name: String,
    short: String
}

#[derive(Deserialize, Debug)]
struct Status {
    id: u16,
    name: String
}

#[derive(Deserialize, Debug)]
struct StreamStart {
    id: String,
    rated: bool,
    variant: Variant,
    speed: String,
    perf: String,
    createdAt: u64,
    lastMoveAt: u64,
    status: String,
    clock: Clock,
    initialFen: String,
    turns: u16,
    startedAtTurn: u16,
    source: String,
    //type: String,
    state: State
}

#[derive(Deserialize, Debug)]
struct GameState {
    //type: String,
    moves: String,
    wtime: u64,
    btime: u64,
    winc: u8,
    binc: u8,
    status: String
}

#[derive(Deserialize, Debug)]
struct ChallengeResponse {
    id: String,
    speed: String,
    perf: String,
    rated: bool,
    fen: String,
    player: String,
    turns: u16,
    startedAtTurn: u16,
    source: String,
    createdAt: u64,
    variant: Variant,
    status: Status
}

fn main() {
    loop {
        let mut buf = String::new();
        let input_str = stdin().read_line(&mut buf).expect("Error reading line");
        let json_stream: Value = from_str(&buf).expect("JSON syntax error");
        println!("{}", json_stream);
    }
}
