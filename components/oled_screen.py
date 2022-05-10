from lib.ssd1306 import SSD1306_I2C


# example use of this module:
#   import screen
#   from machine import I2C, Pin
#   bus = I2C(0, sda=Pin(), scl=Pin())
#   screen = screen.Screen(bus)
#   screen.print("Hello, World!")


# screen size constants
_WIDTH = 128  # px
_HEIGHT = 64  # px
_CHAR_WIDTH = 8  # px
_CHAR_HEIGHT = 8  # px
_COL_SIZE = _WIDTH // _CHAR_WIDTH  # chars
_ROW_SIZE = _HEIGHT // _CHAR_WIDTH  # chars
_SCREEN_SIZE = _COL_SIZE * _ROW_SIZE  # chars


class Screen:
    def __init__(self, bus):
        """Initialise Screen object"""
        # create oled object with correct screen size (128px * 64px)
        self.oled = SSD1306_I2C(_WIDTH, _HEIGHT, bus)

    def clear(self):
        self.oled.fill(0)
        self.oled.show()

    def fill_rect(self, col, row, width, height, colour=1):
        """Simple wrapper for oled.rect(...) NOTE: DOES NOT SHOW"""
        self.oled.fill_rect(col*_CHAR_WIDTH, row*_CHAR_HEIGHT, width*_CHAR_WIDTH, height*_CHAR_HEIGHT, colour)

    def print_unformatted(self, message, col, row, colour=1):
        """Simple wrapper for oled.text(...)"""
        self.oled.text(message, col*_CHAR_WIDTH, row*_CHAR_HEIGHT, colour)
        self.oled.show()

    def print_variable(self, message, col, row):
        """Print a message in a space that must be cleared first. This is useful for printing
        *variable* data that is constantly updating. Has '\n' support, but note that col will
        always stay the same. E.g. if you specify a col of 4, each line will start at col 4.
        Finally, be careful as lines that are too long will run off!"""
        lines = message.split('\n')
        for i in range(0, len(lines)):
            self.oled.fill_rect(col*_CHAR_WIDTH, (row+i)*_CHAR_HEIGHT, len(lines[i])*_CHAR_WIDTH, 1*_CHAR_HEIGHT, 0)
            self.oled.text(lines[i], col*_CHAR_WIDTH, row*_CHAR_HEIGHT, 1)
        self.oled.show()

    def print(self, message):
        """
            Fits a message onto the screen by breaking words apart without mercy.
            Note that print will overwrite any previous prints. Also note that a
            message that is too long is cut off! Now has '\n' support!
            :type message: str
        """
        self.clear()
        row = 0
        col = 0
        i = 0
        cur_line = ""
        while i < len(message):
            if col >= _COL_SIZE:  # print each complete row and then increment to the next row
                self.oled.text(cur_line, 0, row * _CHAR_HEIGHT)
                row += 1
                col = 0
                cur_line = ""
            elif message[i] == '\n':  # handle newlines correctly
                self.oled.text(cur_line, 0, row * _CHAR_HEIGHT)
                row += 1
                col = 0
                cur_line = ""
                i += 1
            else:  # buffer the message
                if (cur_line != "") or (message[i] != " "):  # skip any whitespaces beginning a newline
                    cur_line += message[i]
                i += 1
                col += 1
        self.oled.text(cur_line, 0, row*_CHAR_HEIGHT)
        self.oled.show()

    def print_art(self, message):
        """
            Same as print but preserves whitespace
            :type message: str
        """
        self.clear()
        row = 0
        col = 0
        i = 0
        cur_line = ""
        while i < len(message):
            if col >= _COL_SIZE:  # print each complete row and then increment to the next row
                self.oled.text(cur_line, 0, row * _CHAR_HEIGHT)
                row += 1
                col = 0
                cur_line = ""
            elif message[i] == '\n':  # handle newlines correctly
                self.oled.text(cur_line, 0, row * _CHAR_HEIGHT)
                row += 1
                col = 0
                cur_line = ""
                i += 1
            else:  # buffer the message
                cur_line += message[i]
                i += 1
                col += 1
        self.oled.text(cur_line, 0, row * _CHAR_HEIGHT)
        self.oled.show()
