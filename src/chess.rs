use std::fmt;
use std::io::{self, Write};

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
    pub old_pos: (i8, i8),
    pub new_pos: (i8, i8),
    pub taken_pos: (i8, i8)
}

pub struct Board {
    pub board: [[Piece; 8]; 8],
    // turn = true -> whites turn, otherwise blacks turn
    turn: bool,
    last_move: Move,
    // These are the pieces taken by that color, not the ones that that color has on the board.
    // That is, taken_by_white contains the black pieces taken by white, and vice versa for
    // taken_by_black.
    taken_by_white: Vec<Piece>,
    taken_by_black: Vec<Piece>,
    white_king: (u8, u8),
    black_king: (u8, u8)
}

impl Board {
    pub fn get_player_turn_str(&self) -> &str {
      if self.turn {
        "White"
      } else {
        "Black"
      }
    }

    // First bool is whether we are in check or not, the other signifies the color in check.
    fn is_in_check(&self) -> (bool, bool) {

        false
    }

    // We assume that the move is valid in the sense that both m.old_pos and m.new_pos are actual
    // positions on the board, and the moving piece is not an empty one (i.e. not Piece::Empty).
    fn is_valid_move(&self, m: &mut Move) -> bool {
        if m.old_pos.0 == m.new_pos.0 && m.old_pos.1 == m.new_pos.1 {
            return false;
        }
        let check_bishop = |bishop_white: bool| {
            // Let (x_i, y_i) be the initial position of the bishop, and (x_f, y_f) be the
            // final position. Then the move is valid if and only if |x_f - x_i| = |y_f - y_i|.
            // This fact embodies the idea of only moving along the diagonals on the board.
            // We assume the board is empty in the above theorem.
            let delta_row = (m.new_pos.0 - m.old_pos.0).abs();
            let delta_col = (m.new_pos.1 - m.old_pos.1).abs();
            if delta_row != delta_col { return false; }

            let dr: i8 = if m.new_pos.0 > m.old_pos.0 { 1 } else { -1 };
            let dc: i8 = if m.new_pos.1 > m.old_pos.1 { 1 } else { -1 };
            let mut pos = m.old_pos;
            loop {
                pos.0 += dr;
                pos.1 += dc;
                if pos == m.new_pos {
                    // Check that the final position is not occupied by a piece of the same
                    // color
                    return call_on_empty_status(
                      &self.board[pos.0 as usize][pos.1 as usize],
                      || -> bool { true },
                      |is_white: bool| -> bool { bishop_white != is_white }
                    );
                } else if self.board[pos.0 as usize][pos.1 as usize] != Piece::Empty {
                    return false;
                }
                if pos.0 > 7 || pos.0 < 0 || pos.1 > 7 || pos.1 < 0 {
                    return false;
                }
            }
        };

        let check_rook = |rook_white: bool| -> bool {
            // Let (x_i, y_i) be the initial position of the rook, and (x_f, y_f) be the
            // final position. Then the move is valid if and only if either |x_f - x_i|, or
            // |y_f - y_i| is zero, and the other is nonzero. This fact embodies the idea of
            // only moving along the rows and columns on the board. We assume the board is
            // empty in the above theorem.
            let delta_row = (m.new_pos.0 - m.old_pos.0).abs();
            let delta_col: i8 = (m.new_pos.1 - m.old_pos.1).abs();
            if !(delta_row == 0 && delta_col != 0 || delta_row != 0 && delta_col == 0) {
                return false;
            }

            let mut var_pos: i8;
            let const_pos: i8;
            let d: i8;
            if delta_row == 0 {
                const_pos = m.old_pos.0;
                var_pos = m.old_pos.1;
                d = if m.new_pos.1 > m.old_pos.1 { 1 } else { -1 };
            } else {
                const_pos = m.old_pos.1;
                var_pos = m.old_pos.0;
                d = if m.new_pos.0 > m.old_pos.0 { 1 } else { -1 };
            }

            loop {
                var_pos += d;
                if delta_row == 0 {
                    if var_pos == m.new_pos.1 {
                        return call_on_empty_status(
                          &self.board[const_pos as usize][var_pos as usize],
                          || -> bool {true},
                          |is_white: bool| -> bool { is_white != rook_white }
                        );
                    } else if self.board[const_pos as usize][var_pos as usize] != Piece::Empty {
                        return false;
                    }
                } else {
                    if var_pos == m.new_pos.0 {
                        return call_on_empty_status(
                          &self.board[var_pos as usize][const_pos as usize],
                          || -> bool {true},
                          |is_white: bool| -> bool { is_white != rook_white }
                        );
                    } else if self.board[var_pos as usize][const_pos as usize] != Piece::Empty {
                        return false;
                    }
                }
                if var_pos < 0 || var_pos > 7 { return false; }
            }
        };

        match m.moving_piece {
            Piece::Empty => false,
            Piece::Pawn{is_white: pawn_white} => {
                if pawn_white != self.turn {
                  return false;
                }
                let is_taking_different_color: bool;
                let is_taking_piece: bool = match m.taken_piece {
                    Some(p) => match p {
                        Piece::Empty => { is_taking_different_color = false; false },
                        Piece::Pawn   {is_white} | Piece::Rook   {is_white} |
                        Piece::King   {is_white} | Piece::Queen  {is_white} |
                        Piece::Bishop {is_white} | Piece::Knight {is_white} => {
                            is_taking_different_color = is_white != pawn_white;
                            true
                        }
                    },
                    None => { is_taking_different_color = false; false }
                };
                let moving_forward: bool;
                let first_move: bool;
                let last_move_moves_two_spaces: bool;
                let is_taking_with_en_passant: bool;

                if pawn_white {
                    first_move = m.old_pos.0 == 6;
                    moving_forward = m.old_pos.0 == m.new_pos.0 + 1;
                    last_move_moves_two_spaces = self.last_move.old_pos.0 == self.last_move.new_pos.0 - 2;
                    is_taking_with_en_passant = m.new_pos == (self.last_move.old_pos.0 + 1, self.last_move.old_pos.1);
                } else {
                    first_move = m.old_pos.0 == 1;
                    moving_forward = m.old_pos.0 == m.new_pos.0 - 1;
                    last_move_moves_two_spaces = self.last_move.old_pos.0 == self.last_move.new_pos.0 + 2;
                    is_taking_with_en_passant = m.new_pos == (self.last_move.old_pos.0 - 1, self.last_move.old_pos.1);
                }

                let only_moving_forward = moving_forward && m.old_pos.1 == m.new_pos.1;
                let moving_left = moving_forward && m.old_pos.1 == m.new_pos.1 + 1;
                let moving_right = moving_forward && m.old_pos.1 == m.new_pos.1 - 1;

                // Check if the last move was a first move two spaces forward by an opponent pawn
                // to validate en passant
                let last_move_is_pawn = match self.last_move.moving_piece {
                    Piece::Pawn{..} => true,
                    _ => false
                };
                let last_move_only_forward = self.last_move.old_pos.1 == self.last_move.new_pos.1;
                let last_move_allows_en_passant = last_move_is_pawn && last_move_only_forward &&
                  last_move_moves_two_spaces;

                if first_move && !is_taking_piece {
                    let old_row: usize = m.old_pos.0 as usize;
                    let old_col: usize = m.old_pos.1 as usize;
                    let moving_two_spaces; 
                    let spaces_are_empty;

                    if pawn_white {
                        moving_two_spaces = m.old_pos.0 == m.new_pos.0 + 2 && m.old_pos.1 == m.new_pos.1;
                        spaces_are_empty = self.board[old_row - 1][old_col] == Piece::Empty
                          && self.board[old_row - 2][old_col] == Piece::Empty;
                    } else {
                        moving_two_spaces = m.old_pos.0 == m.new_pos.0 - 2 && m.old_pos.1 == m.new_pos.1;
                        spaces_are_empty = self.board[old_row + 1][old_col] == Piece::Empty
                          && self.board[old_row + 2][old_col] == Piece::Empty;
                    }
                    only_moving_forward || moving_two_spaces && spaces_are_empty
                } else if last_move_allows_en_passant && is_taking_with_en_passant {
                    let valid_move = moving_left || moving_right;
                    if valid_move {
                      m.taken_piece = Some(self.last_move.moving_piece);
                      m.taken_pos = self.last_move.new_pos;
                    }
                    valid_move
                } else if is_taking_piece {
                    (moving_left || moving_right) && is_taking_different_color
                } else {
                    only_moving_forward
                }
            },

            Piece::Knight{is_white: knight_white} => {
                if knight_white != self.turn {
                  return false;
                }
                // These are the differences (delta x, delta y) between every possible position the
                // knight can move to and its current position.
                let possible_position_deltas = [(1,2), (1,-2), (-1,2), (-1,-2), (2,1), (2,-1),
                  (-2,1), (-2,-1)];
                for delta in possible_position_deltas.iter() {
                    let new_row = m.old_pos.0 + delta.0;
                    let new_col = m.old_pos.1 + delta.1;
                    if new_row < 0 || new_row > 7 || new_col < 0 || new_col > 7 {
                        continue;
                    }

                    // check that the new position is not occupied by piece of the same color
                    let are_different_colors = call_on_empty_status(
                      &self.board[new_row as usize][new_col as usize],
                      || -> bool { true },
                      |is_white: bool| -> bool { knight_white != is_white }
                    );
                    if are_different_colors { return true; }
                }
                false
            },

            Piece::Bishop{is_white: bishop_white} => check_bishop(bishop_white),
            Piece::Rook{is_white: rook_white} => check_rook(rook_white),
            Piece::Queen{is_white: queen_white} => check_bishop(queen_white) || check_rook(queen_white),

            Piece::King{is_white: king_white} => {
                let delta_row = m.new_pos.0 - m.old_pos.0;
                let delta_col = m.new_pos.1 - m.old_pos.1;
                if delta_row.abs() > 1 || delta_col.abs() > 1 { false }
                else {
                    call_on_empty_status(
                      &self.board[m.new_pos.0 as usize][m.new_pos.1 as usize],
                      || -> bool { true },
                      |is_white: bool| -> bool { king_white != is_white })
                }
            },
        }
    }

    pub fn make_move(&mut self, m: &mut Move) -> bool {
        println!("{:?}", m);
        if self.is_valid_move(m) {
            self.board[m.old_pos.0 as usize][m.old_pos.1 as usize] = Piece::Empty;
            self.last_move = *m;
            self.turn = !self.turn;
            match m.taken_piece {
                Some(p) => {
                    self.board[m.taken_pos.0 as usize][m.taken_pos.1 as usize] = Piece::Empty;
                    match p {
                        Piece::Pawn   {is_white} | Piece::Rook   {is_white} |
                        Piece::King   {is_white} | Piece::Queen  {is_white} |
                        Piece::Bishop {is_white} | Piece::Knight {is_white} => {
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
                Piece::Pawn{is_white, ..} => {
                    // if the pawn gets to the opposite side of the board, promote
                    if (is_white && m.new_pos.0 == 0) || (!is_white && m.new_pos.0 == 7) {
                        loop {
                            println!("Which piece would you like to upgrade to? (Q/q = Queen, K/k = \
                              Knight, B/b = Bishop, R/r = Rook");
                            print!("Enter piece > ");
                            io::stdout().flush().expect("IO error during flush");
                            let mut input = String::new();
                            match io::stdin().read_line(&mut input) {
                                Ok(n) => {
                                    if n != 2 {
                                        eprintln!("Invalid input: incorrect input length");
                                        continue;
                                    }
                                    match &input[0..1] {
                                        "Q" | "q" => {
                                            self.board[m.new_pos.0 as usize][m.new_pos.1 as usize] =
                                            Piece::Queen{is_white};
                                            break;
                                        }
                                        "R" | "r" => {
                                            self.board[m.new_pos.0 as usize][m.new_pos.1 as usize] =
                                            Piece::Rook{is_white};
                                            break;
                                        }
                                        "B" | "b" => {
                                            self.board[m.new_pos.0 as usize][m.new_pos.1 as usize] =
                                            Piece::Bishop{is_white};
                                            break;
                                        }
                                        "K" | "k" => {
                                            self.board[m.new_pos.0 as usize][m.new_pos.1 as usize] =
                                            Piece::Knight{is_white};
                                            break;
                                        }
                                        _ => {
                                            eprintln!("Invalid input: invalid value supplied");
                                            continue;
                                        }
                                    }
                                },
                                Err(e) => { println!("Error: {}", e) }
                            }
                        }
                    }
                },
                _ => {}
            };
            true
        } else {
            false
        }
    }

    // Returns (white score, black score)
    pub fn get_score(&self) -> (u8, u8) {
        let f = |acc: u8, piece: &Piece| {
            acc + piece_value(piece)
        };
        (self.taken_by_white.iter().fold(0, f), self.taken_by_black.iter().fold(0, f))
    }
}

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
            last_move: Move {
                moving_piece: Piece::Empty,
                taken_piece: None,
                old_pos: (0,0),
                new_pos: (0,0),
                taken_pos: (0,0),
            },
            taken_by_white: Vec::new(),
            taken_by_black: Vec::new(),
            turn: true
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
