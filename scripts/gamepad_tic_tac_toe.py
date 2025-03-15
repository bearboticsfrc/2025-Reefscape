from time import sleep

from adafruit_itertools import count, cycle
from adafruit_macropad import MacroPad, keypad

macropad = MacroPad()


class Team:
    RED = (255, 0, 0)
    BLUE = (0, 0, 255)
    BLANK = (0, 0, 0)


class Gameboard:
    def __init__(self, rows: int = 3, columns: int = 3) -> None:
        self.board = [[Team.BLANK] * rows for _ in range(columns)]

    def get_value(self, index: int):
        row, col = self.gamepad_to_matrix(index)
        return self.board[row][col]

    def gamepad_to_matrix(self, index: int) -> tuple[int, int]:
        return index // 3, index % 3

    def matrix_to_gamepad(self, row: int, column: int) -> int:
        return row * 3 + column

    def set_value(self, index: int, value: int):
        row, column = self.gamepad_to_matrix(index)
        self.board[row][column] = value


class TicTacToe:
    def __init__(self, board: Gameboard = None):
        self.board = board or Gameboard()

    def _is_row_win(self, plays: list[Team]):
        return sum(play is not Team.BLANK for play in plays) == 3 and all(
            play == plays[0] for play in plays
        )

    def _is_horizontal_win(self):
        for row in self.board:
            if self._is_row_win(row):
                return True
        return False

    def _is_vertical_win(self):
        for column in zip(*self.board):
            if self._is_row_win(column):
                return True
        return False

    def _is_diagnoal_win(self):
        return (
            self.board[0][0] == self.board[1][1] == self.board[2][2] != Team.BLANK
            or self.board[0][2] == self.board[1][1] == self.board[2][0] != Team.BLANK
        )

    def is_win(self) -> None:
        return any(
            (
                self._is_horizontal_win(),
                self._is_vertical_win(),
                self._is_diagnoal_win(),
            )
        )

    def is_draw(self, moves):
        return moves >= 8 and not self.is_win()  # n ** 3 - 1

    def get_winning_team(self) -> Team:
        flattend_board = sum(self.board, [])

        if flattend_board.count(Team.RED) > flattend_board.count(Team.BLUE):
            return Team.RED
        else:
            return Team.BLUE

    def get_winning_indices(self) -> tuple[int, int, int]:
        if self._is_horizontal_win():
            for row_index, row in enumerate(self.board):
                if self._is_row_win(row):
                    return (
                        self._matrix_to_gamepad(row_index, 0),
                        self._matrix_to_gamepad(row_index, 1),
                        self._matrix_to_gamepad(row_index, 2),
                    )

        if self._is_vertical_win():
            for column_index, column in enumerate(zip(*self.board)):
                if self._is_row_win(column):
                    return (
                        self._matrix_to_gamepad(0, column_index),
                        self._matrix_to_gamepad(1, column_index),
                        self._matrix_to_gamepad(2, column_index),
                    )

        if self._is_diagnoal_win():
            if self.board[0][0] == self.board[1][1] == self.board[2][2]:
                return 0, 4, 8
            else:
                return 2, 4, 6


def main() -> None:
    board = Gameboard()
    tictactoe = TicTacToe(board)

    turn = cycle((Team.RED, Team.BLUE))
    counter = count()

    macropad.pixels.fill(0)
    macropad.keys.events.clear()

    iterations = next(counter)
    playing = next(turn)

    while True:
        print("\n" * 5)
        print(f"Current Turn: {'RED' if playing[0] == 255 else 'BLUE'}")
        key_event: keypad.Event = macropad.keys.events
        while not key_event:
            if macropad.encoder_switch:
                return

        if macropad.encoder_switch:
            playing = Team.RED if playing == Team.BLUE else Team.BLUE

        key: int = key_event.get().key_number
        if key > 8 or board.get_value(key) != Team.BLANK:
            continue
        else:
            board.set_value(key, playing)
            macropad.pixels[key] = playing

        if board.is_win():
            print("\n" * 5)
            print(f"{'RED' if board.get_winning_team()[0] == 255 else 'BLUE'} wins!")
            break

        if board.is_draw(iterations):
            print("\n" * 5)
            print("Draw!")
            break

        playing = next(turn)
        iterations = next(counter)

    if board.is_draw(iterations):
        macropad.pixels.brightness = 0.1
        for _ in range(3):
            macropad.pixels.fill(0)
            sleep(0.5)
            macropad.pixels.fill((255, 255, 255))
            sleep(0.5)
        sleep(3)
        macropad.pixels.brightness = 1
        return

    winning = board.get_winning_indices()
    winning_color_cycle = cycle((0, board.get_winning_team()))

    for _ in range(8):
        winning_color = next(winning_color_cycle)
        for index in winning:
            macropad.pixels[index] = winning_color

        sleep(0.5)

    sleep(2)
