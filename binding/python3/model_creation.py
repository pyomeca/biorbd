import numpy as np


class Marker():
    def __init__(
        self,
        name: str,
        parent_name: str,
        position: tuple[int|float, int|float, int|float],
    ):
        self.name = name
        self.parent_name = parent_name
        self.position = position

    def __str__(self):
        # Define the print function so it automatically format things in the file properly<
        out_string = f"marker {self.name}\n"
        out_string += f"\tparent {self.parent_name}\n"
        out_string += f"\tposition {self.position[0]} {self.position[1]} {self.position[2]}\n"
        out_string += "endmarker\n"
        return out_string


class Segment():
    def __init__(
        self, 
        name, 
        parent_name: str = "",
        rt: str = "",
        translations: str = "",
        rotations: str = "",
        mass: float|int = 0,
        center_of_mass: tuple[tuple[int|float, int|float, int|float]] = None,
        inertia_xxyyzz: tuple[tuple[int|float, int|float, int|float]] = None,
        mesh: tuple[tuple[int|float, int|float, int|float], ...]=None,
        markers: tuple[Marker, ...]=None,
    ):
        self.name = name
        self.parent_name = parent_name
        self.rt = rt
        self.translations = translations
        self.rotations = rotations
        self.mass = mass
        self.center_of_mass=center_of_mass
        self.inertia_xxyyzz=inertia_xxyyzz
        self.mesh = mesh
        self.markers = markers

    def __str__(self):
        # Define the print function so it automatically format things in the file properly<
        out_string = f"segment {self.name}\n"
        if (self.parent_name):
            out_string += f"\tparent {self.parent_name}\n"
        if (self.rt):
            out_string += f"\tRT {self.rt}\n"
        if (self.translations):
            out_string += f"\ttranslations {self.translations}\n"
        if (self.rotations):
            out_string += f"\trotations {self.rotations}\n"
        if (self.mass):
            out_string += f"\tmass {self.mass}\n"
        if (self.center_of_mass):
            out_string += f"\tcom {self.center_of_mass[0]} {self.center_of_mass[1]} {self.center_of_mass[2]}\n"
        if (self.inertia_xxyyzz):
            out_string += f"\tinertia {self.inertia_xxyyzz[0]} 0 0\n" + \
                          f"\t        0 {self.inertia_xxyyzz[1]} 0\n" + \
                          f"\t        0 0 {self.inertia_xxyyzz[2]}\n"
        if (self.mesh):
            for m in self.mesh:
                out_string += f"\tmesh {m[0]} {m[1]} {m[2]}\n"
        out_string += "endsegment\n"
        
        # Also print the markers attached to the segment
        if (self.markers):
            for marker in self.markers:
                out_string += str(marker)
        return out_string


class KinematicChain():
    def __init__(self, segments: tuple[Segment, ...]):
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


class DeLeva:
    # The DeLeva class is based on DeLeva (1996) "Adjustments to Zatsiorsky-Seluyanov's segment inertia parameters"
    class Param:    
        def __init__(
            self,
            marker_names: tuple[str, ...],  # The name of the markers medial/lateral
            mass: float|int,  # Percentage of the total body mass
            center_of_mass: tuple[float|int, float|int, float|int],  # Position of the center of mass as a percentage of the distance from medial to distal
            radii: tuple[float|int, float|int, float|int],  # [Sagittal, Transverse, Longitudinal] radii of giration
        ):
            self.marker_names = marker_names
            self.mass = mass
            self.center_of_mass = center_of_mass
            self.radii = radii

    def __init__(self, sex: str, mass: float|int, model: biorbd.Model):
        self.sex = sex  # The sex of the subject
        self.mass = mass  # The mass of the subject
        self.model = model  # The biorbd model. This is to compute lengths
        
        # Produce some easy to access variables
        self.q_zero = numpy.zeros((model.nbQ()))
        self.marker_names = [name.to_string() for name in model.markerNames()]
        
        # This is the actual copy of the De Leva table
        self.table = {}
        self.table["male"] = {}
        self.table["male"]["HEAD"]      = DeLeva.Param(marker_names=("TOP_HEAD", "SHOULDER"), mass=0.0694  , center_of_mass=0.5002, radii=(0.303, 0.315, 0.261))
        self.table["male"]["TRUNK"]     = DeLeva.Param(marker_names=("SHOULDER", "PELVIS")  , mass=0.4346  , center_of_mass=0.5138, radii=(0.328, 0.306, 0.169))
        self.table["male"]["UPPER_ARM"] = DeLeva.Param(marker_names=("SHOULDER", "ELBOW")   , mass=0.0271*2, center_of_mass=0.5772, radii=(0.285, 0.269, 0.158))
        self.table["male"]["LOWER_ARM"] = DeLeva.Param(marker_names=("ELBOW", "WRIST")      , mass=0.0162*2, center_of_mass=0.4574, radii=(0.276, 0.265, 0.121))
        self.table["male"]["HAND"]      = DeLeva.Param(marker_names=("WRIST", "FINGER")     , mass=0.0061*2, center_of_mass=0.7900, radii=(0.628, 0.513, 0.401))
        self.table["male"]["THIGH"]     = DeLeva.Param(marker_names=("PELVIS", "KNEE")      , mass=0.1416*2, center_of_mass=0.4095, radii=(0.329, 0.329, 0.149))
        self.table["male"]["SHANK"]     = DeLeva.Param(marker_names=("KNEE", "ANKLE")       , mass=0.0433*2, center_of_mass=0.4459, radii=(0.255, 0.249, 0.103))
        self.table["male"]["FOOT"]      = DeLeva.Param(marker_names=("ANKLE", "TOE")        , mass=0.0137*2, center_of_mass=0.4415, radii=(0.257, 0.245, 0.124))

        self.table["female"] = {}
        self.table["female"]["HEAD"]      = DeLeva.Param(marker_names=("TOP_HEAD", "SHOULDER"), mass=0.0669  , center_of_mass=0.4841, radii=(0.271, 0.295, 0.261))
        self.table["female"]["TRUNK"]     = DeLeva.Param(marker_names=("SHOULDER", "PELVIS")  , mass=0.4257  , center_of_mass=0.4964, radii=(0.307, 0.292, 0.147))
        self.table["female"]["UPPER_ARM"] = DeLeva.Param(marker_names=("SHOULDER", "ELBOW")   , mass=0.0255*2, center_of_mass=0.5754, radii=(0.278, 0.260, 0.148))
        self.table["female"]["LOWER_ARM"] = DeLeva.Param(marker_names=("ELBOW", "WRIST")      , mass=0.0138*2, center_of_mass=0.4559, radii=(0.261, 0.257, 0.094))
        self.table["female"]["HAND"]      = DeLeva.Param(marker_names=("WRIST", "FINGER")     , mass=0.0056*2, center_of_mass=0.7474, radii=(0.531, 0.454, 0.335))
        self.table["female"]["THIGH"]     = DeLeva.Param(marker_names=("PELVIS", "KNEE")      , mass=0.1478*2, center_of_mass=0.3612, radii=(0.369, 0.364, 0.162))
        self.table["female"]["SHANK"]     = DeLeva.Param(marker_names=("KNEE", "ANKLE")       , mass=0.0481*2, center_of_mass=0.4416, radii=(0.271, 0.267, 0.093))
        self.table["female"]["FOOT"]      = DeLeva.Param(marker_names=("ANKLE", "TOE")        , mass=0.0129*2, center_of_mass=0.4014, radii=(0.299, 0.279, 0.124))

    def segment_mass(self, segment: Segment):
        return self.table[self.sex][segment].mass * self.mass
    
    def segment_length(self, segment: Segment):
        table = self.table[self.sex][segment]
        
        # Find the position of the markers when the model is in resting position
        marker_positions = numpy.array([marker.to_array() for marker in self.model.markers(self.q_zero)]).transpose()
        
        # Find the index of the markers required to compute the length of the segment
        idx_proximal = self.marker_names.index(table.marker_names[0])
        idx_distal = self.marker_names.index(table.marker_names[1])
        
        # Compute the Euclidian distance between the two positions
        return numpy.linalg.norm(marker_positions[:, idx_distal] - marker_positions[:, idx_proximal])
        
    def segment_center_of_mass(self, segment: Segment, inverse_proximal: bool = False):
        # This method will compute the length of the required segment based on the biorbd model and required markers
        # If inverse_proximal is set to True, then the value is returned from the distal position
        table = self.table[self.sex][segment]
        
        # Find the position of the markers when the model is in resting position
        marker_positions = numpy.array([marker.to_array() for marker in self.model.markers(self.q_zero)]).transpose()
        
        # Find the index of the markers required to compute the length of the segment
        idx_proximal = self.marker_names.index(table.marker_names[0])
        idx_distal = self.marker_names.index(table.marker_names[1])
        
        # Compute the position of the center of mass
        if inverse_proximal:
            center_of_mass = (1-table.center_of_mass) * (marker_positions[:, idx_proximal] - marker_positions[:, idx_distal])
        else:
            center_of_mass = table.center_of_mass * (marker_positions[:, idx_distal] - marker_positions[:, idx_proximal])
        return tuple(center_of_mass)  # convert the result to a Tuple which is good practise
        
    def segment_moment_of_inertia(self, segment: Segment):
        mass = self.segment_mass(segment)
        length = self.segment_length(segment)
        radii = self.table[self.sex][segment].radii
        
        return (mass * (length*radii[0])**2, mass * (length*radii[1])**2, mass * (length*radii[2])**2)
