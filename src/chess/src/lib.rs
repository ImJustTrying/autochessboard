use std::fmt;
use std::io::{self, Write};
use std::vec::Vec;

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Piece {
    Pawn   { is_white: bool },
    Bishop { is_white: bool },
    Knight { is_white: bool },
    Rook   { is_white: bool },
    Queen  { is_white: bool },
    King   { is_white: bool },
    Empty
}

#[derive(Copy, Clone, Debug)]
pub struct Move {
    pub moving_piece: Piece,
    pub taken_piece: Option<Piece>,
    pub promotion: Option<Piece>,
    pub old_pos: (i8, i8),
    pub new_pos: (i8, i8),
    // the only time new_pos and taken_pos are different is when en passant captures occur
    pub taken_pos: Option<(i8, i8)>,
    pub is_castle: bool
}

pub struct Board {
    // Maybe make these private and give access through getters?
    board: [[Piece; 8]; 8],
    // These are the pieces taken by that color, not the ones that that color has on the board.
    // That is, taken_by_white contains the black pieces taken by white, and vice versa for
    // taken_by_black.
    taken_by_white: Vec<Piece>,
    taken_by_black: Vec<Piece>,
}

pub struct Game {
    pub board: Board,
    moves: Vec<Move>,
    game_over: bool,
    // turn = true -> whites turn, otherwise blacks turn
    turn: bool,
}


impl Game {
    // TODO: Create the FEN representation of the board in it's current state
    pub fn get_fen(&self) -> String {
        let fen = String::new();
        for rank in self.board.board {
            for file in rank {
                println!("{:#?}", file);
            }
        }
	fen
    }

    pub fn get_player_turn_str(&self) -> &str {
        if self.turn {
            "White"
        } else {
            "Black"
        }
    }

    pub fn last_move(&self) -> Option<&Move> {
        self.moves.last()
    }
}

impl Default for Game {
    fn default() -> Self {
        Game {
            board: Default::default(),
            moves: Vec::new(),
            turn: true,
            game_over: false
        }
    }
}


// --------------------- This is for the Board struct --------------------- 
impl Default for Board {
    fn default() -> Self {
        let mut default_board = [[Piece::Empty; 8]; 8];
        let mut i = 0;
        // Setup the initial state of the board
        while i < 8 {
            let initial_black_piece: Piece = match i {
                0 | 7 => Piece::Rook   { is_white: false },
                1 | 6 => Piece::Knight { is_white: false },
                2 | 5 => Piece::Bishop { is_white: false },
                3     => Piece::Queen  { is_white: false },
                4     => Piece::King   { is_white: false },
                _ => Piece::Empty
            };
            let initial_white_piece: Piece = match i {
                0 | 7 => Piece::Rook   { is_white: true },
                1 | 6 => Piece::Knight { is_white: true },
                2 | 5 => Piece::Bishop { is_white: true },
                3     => Piece::Queen  { is_white: true },
                4     => Piece::King   { is_white: true },
                _ => Piece::Empty
            };

            default_board[1][i] = Piece::Pawn { is_white: false };
            default_board[6][i] = Piece::Pawn { is_white: true };
            default_board[0][i] = initial_black_piece;
            default_board[7][i] = initial_white_piece;
            i += 1;
        }

        Board {
            board: default_board,
            taken_by_white: Vec::new(),
            taken_by_black: Vec::new(),
        }
    }
}

impl Board {
    pub fn make_move(&mut self, m: &mut Move) -> bool {
        self.board[m.old_pos.0 as usize][m.old_pos.1 as usize] = Piece::Empty;
        match m.taken_piece {
            Some(p) => {
                match m.taken_pos {
                    Some(t) => {
                        self.board[t.0 as usize][t.1 as usize] = Piece::Empty;
                    },
                    None => {}
                };

                match p {
                    Piece::Pawn   {is_white} | Piece::Rook   {is_white} |
                    Piece::Queen  {is_white} | Piece::Bishop {is_white} |
                    Piece::Knight {is_white} => {
                        if is_white {
                            self.taken_by_black.push(p);
                        } else {
                            self.taken_by_white.push(p);
                        }
                    },
                    _ => {}
                };
            },
            None => {}
        };
        self.board[m.new_pos.0 as usize][m.new_pos.1 as usize] = m.moving_piece;

        // Determine if pawn promotion should occur
        match m.moving_piece {
            Piece::Pawn{is_white} => {
                // TODO: if the pawn gets to the opposite side of the board, promote
            },
            _ => {}
        };
        true
    }

    pub fn make_moves(&mut self, moves: &mut Vec<Move>) -> u8 {
        let mut num_moves_made: u8 = 0;
        for m in moves.iter_mut() {
            if self.make_move(m) {
                num_moves_made += 1;
            } else {
                return num_moves_made;
            }
        }
        num_moves_made
    }

    // Returns (white score, black score)
    pub fn get_score(&self) -> (u8, u8) {
        let f = |acc: u8, piece: &Piece| {
            acc + piece_value(piece)
        };
        (self.taken_by_white.iter().fold(0, f), self.taken_by_black.iter().fold(0, f))
    }

    pub fn get_piece(&self, rank: usize, file: usize) -> Option<Piece> {
        if rank < 8 && file < 8 {
            Some(self.board[rank][file])
        } else {
            None
        }
    }
}


impl fmt::Display for Board {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut string = String::new();
        let mut row = 0;
        let score = self.get_score();

        while row < 8 {
            let mut col = 0;
            while col <= 9 {
                string.push(' ');
                if col == 0 { // Board coordinates
                    string.push_str(&(8 - row).to_string()[..]);
                } else if col < 9 { // Board
                    string.push(piece_to_unicode(&self.board[row][col - 1]));
                } else { // Score
                    if row == 0 {
                        string.push_str(" White: ");
                        string.push_str(&score.0.to_string()[..]);
                    } else if row == 1 {
                        string.push_str(" Black: ");
                        string.push_str(&score.1.to_string()[..]);
                    }
                }
                col += 1;
            }
            string.push('\n');
            row += 1;
        }
        string.push_str("   a b c d e f g h\n");
        write!(f, "{}", string)
    }
}

impl fmt::Display for Move {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = String::new();
        let index_to_letter = |index: i8| -> char {
            match index {
                0 => 'a',
                1 => 'b',
                2 => 'c',
                3 => 'd',
                4 => 'e',
                5 => 'f',
                6 => 'g',
                7 => 'h',
                _ => '_'
            }
        };

        s.push(piece_to_unicode(&self.moving_piece));
        s.push(index_to_letter(self.old_pos.1));
        // The (8 - ...) is to transform the internal board's indices to those used on the board UI.
        s.push_str(&(8 - self.old_pos.0).to_string()[..]);
        match self.taken_piece {
            Some(p) => {
                s.push('x');
                s.push(piece_to_unicode(&p));
            },
            None => {
                s.push('-');
            }
        }
        s.push(index_to_letter(self.new_pos.1));
        s.push_str(&(8 - self.new_pos.0).to_string()[..]);
        write!(f, "{}", s)
    }
}

// ============== PUBLIC FUNCTIONS ==============
pub fn move_strings_to_moves(move_strings: Vec<String>) -> Vec<Move> {
    /*
    for (move_string in move_strings) {
        
    }
    */
    vec![]
}

pub fn same_color(p1: &Piece, p2: &Piece) -> bool {
    let p1_is_white: bool = call_on_empty_status(p1, || {true}, |is_white| {is_white}); 
    let p2_is_white: bool = call_on_empty_status(p2, || {true}, |is_white| {is_white});
    p1_is_white == p2_is_white
}


// ============== PRIVATE FUNCTIONS ==============
// Takes two argument functions and calls either the first or second depending on whether the
// piece is empty or nonempty.
fn call_on_empty_status<T: Fn() -> bool, U: Fn(bool) -> bool>
(piece: &Piece, on_empty: T, on_nonempty: U) -> bool {
    match piece {
        Piece::Empty => { on_empty() },
        Piece::Pawn   {is_white, ..} | Piece::Rook   {is_white, ..} |
        Piece::King   {is_white, ..} | Piece::Queen  {is_white, ..} |
        Piece::Bishop {is_white, ..} | Piece::Knight {is_white, ..} =>
        { on_nonempty(*is_white) }
    }
}

fn piece_value(piece: &Piece) -> u8 {
    match piece {
        Piece::Empty => 0,
        Piece::Pawn{..}   => 1,
        Piece::Knight{..} => 3,
        Piece::Bishop{..} => 3,
        Piece::Rook{..}   => 5,
        Piece::Queen{..}  => 10,
        Piece::King{..}   => 0
    }
}

fn piece_to_unicode(piece: &Piece) -> char {
    const EMPTY_PIECE:  char = '.';
    const WHITE_KING:   char = '\u{2654}';
    const WHITE_QUEEN:  char = '\u{2655}';
    const WHITE_ROOK:   char = '\u{2656}';
    const WHITE_BISHOP: char = '\u{2657}';
    const WHITE_KNIGHT: char = '\u{2658}';
    const WHITE_PAWN:   char = '\u{2659}';
    const BLACK_KING:   char = '\u{265A}';
    const BLACK_QUEEN:  char = '\u{265B}';
    const BLACK_ROOK:   char = '\u{265C}';
    const BLACK_BISHOP: char = '\u{265D}';
    const BLACK_KNIGHT: char = '\u{265E}';
    const BLACK_PAWN:   char = '\u{265F}';

    let get_piece_char = |is_white: &bool, white: char, black: char| -> char {
        if *is_white {
            white
        } else {
            black
        }
    };

    match piece {
        Piece::Empty => EMPTY_PIECE,
        Piece::Pawn   {is_white, ..} => get_piece_char(is_white, WHITE_PAWN,   BLACK_PAWN),
        Piece::Bishop {is_white, ..} => get_piece_char(is_white, WHITE_BISHOP, BLACK_BISHOP),
        Piece::Knight {is_white, ..} => get_piece_char(is_white, WHITE_KNIGHT, BLACK_KNIGHT),
        Piece::Rook   {is_white, ..} => get_piece_char(is_white, WHITE_ROOK,   BLACK_ROOK),
        Piece::Queen  {is_white, ..} => get_piece_char(is_white, WHITE_QUEEN,  BLACK_QUEEN),
        Piece::King   {is_white, ..} => get_piece_char(is_white, WHITE_KING,   BLACK_KING)
    }
}


// ================== TESTS ================== 
#[cfg(test)]
mod unit_tests {
    use crate::{Move, Board, Piece};
    #[test]
    fn first_move() {
        let mut board: Board = Default::default();
        let mut m: Move = Move {
            moving_piece: Piece::Pawn { is_white: true },
            taken_piece: None,
            promotion: None,
            old_pos: (0, 1),
            new_pos: (0, 3),
            taken_pos: None
        };
        assert!(board.make_move(&mut m));
    }
}
