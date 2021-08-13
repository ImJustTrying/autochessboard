use std::fmt;

const EMPTY_PIECE: char = '.';
const WHITE_KING: char = '\u{2654}';
const WHITE_QUEEN: char = '\u{2655}';
const WHITE_ROOK: char = '\u{2656}';
const WHITE_BISHOP: char = '\u{2657}';
const WHITE_KNIGHT: char = '\u{2658}';
const WHITE_PAWN: char = '\u{2659}';
const BLACK_KING: char = '\u{265A}';
const BLACK_QUEEN: char = '\u{265B}';
const BLACK_ROOK: char = '\u{265C}';
const BLACK_BISHOP: char = '\u{265D}';
const BLACK_KNIGHT: char = '\u{265E}';
const BLACK_PAWN: char = '\u{265F}';


#[derive(Copy, Clone)]
pub enum Piece {
    // If the associated boolean is true, then the piece is white, otherwise it is black.
    Pawn(bool),
    Bishop(bool),
    Knight(bool),
    Rook(bool),
    Queen(bool),
    King(bool),
    Empty
}

pub struct Board {
    board: [[Piece; 8]; 8]
}

impl Board {
    pub fn init() -> Board {
        let mut default_board = [[Piece::Empty; 8]; 8];
        let mut i = 0;
        // Setup the initial state of the board
        while i < 8 {
            let initial_black_piece: Piece = match i {
                0 | 7 => Piece::Rook(false),
                1 | 6 => Piece::Knight(false),
                2 | 5 => Piece::Bishop(false),
                3     => Piece::Queen(false),
                4     => Piece::King(false),
                _ => Piece::Empty
            };
            let initial_white_piece: Piece = match i {
                0 | 7 => Piece::Rook(true),
                1 | 6 => Piece::Knight(true),
                2 | 5 => Piece::Bishop(true),
                3     => Piece::Queen(true),
                4     => Piece::King(true),
                _ => Piece::Empty
            };

            default_board[1][i] = Piece::Pawn(false);
            default_board[6][i] = Piece::Pawn(true);
            default_board[0][i] = initial_black_piece;
            default_board[7][i] = initial_white_piece;
            i += 1;
        }

        Board {
            board: default_board
        }
    }
}

impl fmt::Display for Board {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut string = String::new();
        let get_piece_char = |is_white: &bool, white: char, black: char| -> char {
            if *is_white {
                white
            } else {
                black
            }
        };

        for row in self.board.iter() {
            for piece in row.iter() {
                // map each piece to its unicode representation
                let piece_character: char = match piece {
                    Piece::Empty => EMPTY_PIECE,
                    Piece::Pawn(is_white) => get_piece_char(is_white, WHITE_PAWN, BLACK_PAWN),
                    Piece::Bishop(is_white) => get_piece_char(is_white, WHITE_BISHOP, BLACK_BISHOP),
                    Piece::Knight(is_white) => get_piece_char(is_white, WHITE_KNIGHT, BLACK_KNIGHT),
                    Piece::Rook(is_white) => get_piece_char(is_white, WHITE_ROOK, BLACK_ROOK),
                    Piece::Queen(is_white) => get_piece_char(is_white, WHITE_QUEEN, BLACK_QUEEN),
                    Piece::King(is_white) => get_piece_char(is_white, WHITE_KING, BLACK_KING)
                };
                string.push(' ');
                string.push(piece_character);
            }
            string.push('\n');
        }
        write!(f, "{}", string)
    }
}

pub fn is_valid_move((piece, old_pos, new_pos): (Piece, (u8, u8), (u8, u8))) -> bool {
    false
}
