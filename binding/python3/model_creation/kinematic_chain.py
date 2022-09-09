from .segment_real import SegmentReal


class KinematicChain:
    def __init__(self, segments: tuple[SegmentReal, ...]):
        self.segments = segments

    def __str__(self):
        out_string = "version 4\n\n"
        for segment in self.segments:
            out_string += str(segment)
            out_string += "\n\n\n"  # Give some space between segments
        return out_string

    def write(self, file_path: str):
        # Method to write the current KinematicChain to a file
        with open(file_path, "w") as file:
            file.write(str(self))
