class BiomechanicalModelReal:
    def __init__(self):
        from .segment_real import SegmentReal  # Imported here to prevent from circular imports

        self.segments: dict[str:SegmentReal, ...] = {}
        # From Pythom 3.7 the insertion order in a dict is preserved. This is important because when writing a new
        # .bioMod file, the order of the segment matters

    def __getitem__(self, item: str):
        return self.segments[item]

    def __setitem__(self, key, value):
        self.segments[key] = value

    def __str__(self):
        out_string = "version 4\n\n"
        for name in self.segments:
            out_string += str(self.segments[name])
            out_string += "\n\n\n"  # Give some space between segments
        return out_string

    def write(self, file_path: str):
        # Method to write the current KinematicChain to a file
        with open(file_path, "w") as file:
            file.write(str(self))
