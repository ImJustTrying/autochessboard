use std::fmt;

#[derive(Copy, Clone, PartialEq)]
pub enum Piece {
    Pawn   { is_white: bool, value: u8 },
    Bishop { is_white: bool, value: u8 },
    Knight { is_white: bool, value: u8 },
    Rook   { is_white: bool, value: u8 },
    Queen  { is_white: bool, value: u8 },
    King   { is_white: bool, value: u8 },
    Empty
}

#[derive(Copy, Clone)]
pub struct Move {
    pub moving_piece: Piece,
    pub taken_piece: Option<Piece>,
    pub old_pos: (i8, i8),
    pub new_pos: (i8, i8)
}

pub struct Board {
    pub board: [[Piece; 8]; 8],
    // turn = true -> whites turn, otherwise blacks turn
    turn: bool,
    last_move: Move,
    // These are the pieces taken by that color, not the ones that that color has on the board.
    // That is, white_pieces contains the black pieces taken by white, and vice versa for
    // black_pieces.
    white_pieces: Vec<Piece>,
    black_pieces: Vec<Piece>
}

impl Board {
    // We assume that the move is valid in the sense that both m.old_pos and m.new_pos are actual
    // positions on the board, and the moving piece is not an empty one (i.e. not Piece::Empty).
    pub fn is_valid_move(&self, m: &Move) -> bool {
        if m.old_pos.0 == m.new_pos.0 && m.old_pos.1 == m.new_pos.1 {
            return false;
        }

        match m.moving_piece {
            Piece::Empty => false,
            Piece::Pawn{is_white, ..} => {
                let is_taking_piece: bool = match m.taken_piece {
                    Some(_) => true,
                    None => false
                };
                let moving_forward: bool;
                let first_move: bool;

                if is_white {
                    first_move = m.old_pos.0 == 6;
                    moving_forward = m.old_pos.0 == m.new_pos.0 + 1;
                } else {
                    first_move = m.old_pos.0 == 1;
                    moving_forward = m.old_pos.0 == m.new_pos.0 - 1;
                }
                let only_moving_forward = moving_forward && m.old_pos.1 == m.new_pos.1;
                let moving_left = moving_forward && m.old_pos.1 == m.new_pos.1 + 1;
                let moving_right = moving_forward && m.old_pos.1 == m.new_pos.1 - 1;

                if first_move && !is_taking_piece {
                    let old_row: usize = m.old_pos.0 as usize;
                    let old_col: usize = m.old_pos.1 as usize;
                    let moving_two_spaces; 
                    let spaces_are_empty;

                    if is_white {
                        moving_two_spaces = m.old_pos.0 == m.new_pos.0 + 2 && m.old_pos.1 == m.new_pos.1;
                        spaces_are_empty = self.board[old_row - 1][old_col] == Piece::Empty
                          && self.board[old_row - 2][old_col] == Piece::Empty;
                    } else {
                        moving_two_spaces = m.old_pos.0 == m.new_pos.0 - 2 && m.old_pos.1 == m.new_pos.1;
                        spaces_are_empty = self.board[old_row + 1][old_col] == Piece::Empty
                          && self.board[old_row + 2][old_col] == Piece::Empty;
                    }
                    only_moving_forward || moving_two_spaces && spaces_are_empty
                } else if !is_taking_piece {
                    only_moving_forward
                } else {
                    moving_left || moving_right
                }
            },

            _ => false
        }
    }

    pub fn make_move(&mut self, m: &Move) -> bool {
        if self.is_valid_move(m) {
            self.board[m.new_pos.0 as usize][m.new_pos.1 as usize] = m.moving_piece;
            self.board[m.old_pos.0 as usize][m.old_pos.1 as usize] = Piece::Empty;
            self.last_move = *m;
            match m.taken_piece {
                Some(p) => match p {
                    Piece::Pawn   {is_white, ..} | Piece::Rook   {is_white, ..} |
                    Piece::King   {is_white, ..} | Piece::Queen  {is_white, ..} |
                    Piece::Bishop {is_white, ..} | Piece::Knight {is_white, ..} => {
                        if is_white {
                            self.black_pieces.push(p);
                        } else {
                            self.white_pieces.push(p);
                        }
                    },
                    Piece::Empty => {}
                },
                None => {}
            };
            true
        } else {
            false
        }
    }

    // Returns (white score, black score)
    pub fn get_score(&self) -> (u8, u8) {
        let f = |acc: u8, piece: &Piece| {
            match piece {
                Piece::Pawn   {value, ..} | Piece::Rook   {value, ..} |
                Piece::King   {value, ..} | Piece::Queen  {value, ..} |
                Piece::Bishop {value, ..} | Piece::Knight {value, ..} => {
                    acc + value
                },
                _ => acc
            }
        };
        (self.white_pieces.iter().fold(0, f), self.black_pieces.iter().fold(0, f))
    }
}

impl Default for Board {
    fn default() -> Self {
        let mut default_board = [[Piece::Empty; 8]; 8];
        let mut i = 0;
        // Setup the initial state of the board
        while i < 8 {
            let initial_black_piece: Piece = match i {
                0 | 7 => Piece::Rook   { is_white: false, value: 5  },
                1 | 6 => Piece::Knight { is_white: false, value: 3  },
                2 | 5 => Piece::Bishop { is_white: false, value: 3  },
                3     => Piece::Queen  { is_white: false, value: 10 },
                4     => Piece::King   { is_white: false, value: 0  },
                _ => Piece::Empty
            };
            let initial_white_piece: Piece = match i {
                0 | 7 => Piece::Rook   { is_white: true, value: 5  },
                1 | 6 => Piece::Knight { is_white: true, value: 3  },
                2 | 5 => Piece::Bishop { is_white: true, value: 3  },
                3     => Piece::Queen  { is_white: true, value: 10 },
                4     => Piece::King   { is_white: true, value: 0  },
                _ => Piece::Empty
            };

            default_board[1][i] = Piece::Pawn { is_white: false, value: 1 };
            default_board[6][i] = Piece::Pawn { is_white: true, value: 1 };
            default_board[0][i] = initial_black_piece;
            default_board[7][i] = initial_white_piece;
            i += 1;
        }

        Board {
            board: default_board,
            last_move: Move {
                moving_piece: Piece::Empty,
                taken_piece: None,
                old_pos: (0,0),
                new_pos: (0,0)
            },
            white_pieces: Vec::new(),
            black_pieces: Vec::new()
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


fn piece_to_unicode(piece: &Piece) -> char {
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
