class BiomechanicalModelReal:
    def __init__(self):
        from .segment_real import SegmentReal  # Imported here to prevent from circular imports
        from .muscle_group import MuscleGroup
        from .muscle_real import MuscleReal

        self.segments: dict[str:SegmentReal, ...] = {}
        # From Pythom 3.7 the insertion order in a dict is preserved. This is important because when writing a new
        # .bioMod file, the order of the segment matters
        self.muscle_groups: dict[str:MuscleGroup, ...] = {}
        self.muscles: dict[str:MuscleReal, ...] = {}


    def __str__(self):
        out_string = "version 4\n\n"

        out_string += "// --------------------------------------------------------------"
        out_string += "// SEGMENTS"
        out_string += "// --------------------------------------------------------------"
        for name in self.segments:
            out_string += str(self.segments[name])
            out_string += "\n\n\n"  # Give some space between segments

        out_string += "// --------------------------------------------------------------"
        out_string += "// MUSCLE GROUPS"
        out_string += "// --------------------------------------------------------------"
        for name in self.muscle_groups:
            out_string += str(self.muscle_groups[name])

        out_string += "// --------------------------------------------------------------"
        out_string += "// MUSCLES"
        out_string += "// --------------------------------------------------------------"
        for name in self.muscles:
            out_string += str(self.muscles[name])
            out_string += "\n\n\n"  # Give some space between muscles
        return out_string

    def write(self, file_path: str):
        # Method to write the current KinematicChain to a file
        with open(file_path, "w") as file:
            file.write(str(self))
