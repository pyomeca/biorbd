"""
This examples shows how to
    1. Load a model
    2. Modify one of its properties (here, the mass of a segment)
    3. Write the modified model to a new .bioMod file
    4. Reload the written file to confirm the change was saved
"""

from pathlib import Path

import biorbd


def main():
    # Load a predefined model
    current_file_dir = Path(__file__).parent
    model = biorbd.Model(f"{current_file_dir}/../pyomecaman.bioMod")

    # Modify the mass of the first segment
    segment = model.segment(0)
    print(f"Original mass of {segment.name().to_string()}: {segment.characteristics().mass()}")
    segment.characteristics().setMass(segment.characteristics().mass() * 2)

    # Write the modified model to a new file
    output_path = str(current_file_dir / "write_model_output.bioMod")
    biorbd.Writer.writeModel(model, output_path)

    # Reload the file to confirm the modification was properly saved
    reloaded_model = biorbd.Model(output_path)
    print(
        f"Mass of {segment.name().to_string()} after write/reload: "
        f"{reloaded_model.segment(0).characteristics().mass()}"
    )


if __name__ == "__main__":
    main()
