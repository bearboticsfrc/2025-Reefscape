import gamepad_tic_tac_toe
import score


from adafruit_macropad import MacroPad
from hid_gamepad import Gamepad
from usb_hid import devices
from time import sleep

GAMEPAD = Gamepad(devices)
MACROPAD = MacroPad()


def main() -> None:
    while True:
        score.LocationMacropad(gamepad=GAMEPAD, macropad=MACROPAD).run()
        sleep(0.5)
        gamepad_tic_tac_toe.main(gamepad=GAMEPAD, macropad=MACROPAD)
        sleep(0.5)


if __name__ == "__main__":
    main()
