class KinematicChain:
    def __init__(self):
        from .segment_real import SegmentReal  # Imported here to prevent from circular imports
        self.segments: list[SegmentReal, ...] = []

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
