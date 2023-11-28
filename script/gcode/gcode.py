import re


class GcodeParser():
    def __init__(self, file_path):
        self.gcode = open(file_path, "r+")


    def get_line_coordianates(self, line_index):
        gcode_line = self.gcode.readline(line_index)
        
        # Regular expression pattern to match X, Y, and Z coordinates in a G-code line
        pattern = re.compile(r'X([-+]?\d*\.?\d+)?\s*Y([-+]?\d*\.?\d+)?\s*Z([-+]?\d*\.?\d+)?')

        # Search for the pattern in the G-code line
        match = pattern.search(gcode_line)

        # Initialize coordinates to None
        x, y, z = None, None, None

        # Extract X, Y, and Z coordinates if found
        if match:
            x = float(match.group(1)) if match.group(1) is not None else None
            y = float(match.group(2)) if match.group(2) is not None else None
            z = float(match.group(3)) if match.group(3) is not None else None

        return x, y, z
    