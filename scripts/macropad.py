from __future__ import annotations

from adafruit_macropad import MacroPad
from hid_gamepad import Gamepad
from usb_hid import devices
from time import time


GAMEPAD = Gamepad(devices)
MACROPAD = MacroPad()


class LocationMacropad:
    SELECTED_RGB = (255, 255, 255)

    def __init__(self, gamepad, macropad):
        # Generate selected keys from range 1-12
        self.selected_key = None

        self.gamepad = gamepad
        self.macropad = macropad

        self.macropad.pixels.brightness = 1
        self.display_text = self.macropad.display_text(title="   Scoring Location")

    def to_button(self, key: int) -> int:
        return key + 1

    def fill_selected_pixels(self) -> None:
        if self.selected_key is not None:
            row_index = self.selected_key // 3
            start_index = row_index * 3
            # Highlight every button in the row
            for i in range(start_index, start_index + 3):
                self.macropad.pixels[i] = self.SELECTED_RGB

    def to_level(self, key: int):
        if key is None:
            return

        if key < 3:
            return "L4"
        elif key < 6:
            return "L3"
        elif key < 9:
            return "L2"
        elif key < 12:
            return "L1"

    def display_selected_location(self):
        level = self.to_level(self.selected_key) or "None"

        column_display = f"Selected Level: {level}"

        if column_display == self.display_text[1].text:
            return

        self.display_text[1].text = column_display
        self.display_text.show()

    def initalize_display(self) -> None:
        self.display_text[1].text = "Selected Level: None"
        self.display_text.show()

    def reset(self):
        self.selected_key = None
        self.gamepad.release_all_buttons()
        self.macropad.pixels.fill(0)

    def run(self) -> None:
        self.initalize_display()
        last_pressed = 0

        while True:
            if MACROPAD.encoder <= -100:
                return

            event = self.macropad.keys.events.get()

            if self.macropad.encoder_switch or (
                self.selected_key is not None and (time() - last_pressed) > 180
            ):
                self.reset()

            if not event or not event.pressed:
                self.display_selected_location()
                self.fill_selected_pixels()
                continue

            last_pressed = time()

            self.selected_key = event.key_number
            self.gamepad.release_all_buttons()
            self.gamepad.press_buttons(self.to_button(3 - self.selected_key // 3))
            self.macropad.pixels.fill(0)


LocationMacropad(gamepad=GAMEPAD, macropad=MACROPAD).run()
