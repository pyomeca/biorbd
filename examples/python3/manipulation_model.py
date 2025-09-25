from pathlib import Path

import numpy as np
from biorbd import Biorbd


#
# This examples shows how get and modify the core model
# Please note that this example will work only with the Eigen backend
#


def main(show: bool = True):
    # Load a predefined model
    current_file_dir = Path(__file__).parent
    model = Biorbd(f"{current_file_dir}/../pyomecaman.bioMod")
    q = [0] * model.nb_q

    # Get the mass
    print("Mass of the model:", model.mass())

    # Get the center of mass at a specific pose. If q is not sent, then it uses the previously set q
    print(f"The center of mass at q: {model.center_of_mass(q)}")


if __name__ == "__main__":
    main()
