class KinematicChain:
    def __init__(self):
        from .segment_real import SegmentReal  # Imported here to prevent from circular imports
        self.segments: dict[str: SegmentReal, ...] = {}

    def add_segment(self, segment) -> None:
        """
        Add a new segment to the kinematic chain. The order of the insertion is preserved
        Parameters
        ----------
        segment
            The new segment
        """

        # From Pythom 3.7 the insertion order in a dict is preserved. THis is important because when writting a new
        # .bioMod file, the order of the segment matters
        self.segments[segment.name] = segment

    def __getitem__(self, item: str):
        return self.segments[item]

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
