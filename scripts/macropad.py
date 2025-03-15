import gamepad_tic_tac_toe
import score


def main() -> None:
    while True:
        score.LocationMacropad(gamepad=score.GAMEPAD, macropad=score.MACROPAD).run()
        gamepad_tic_tac_toe.main()


if __name__ == "__main__":
    main()
