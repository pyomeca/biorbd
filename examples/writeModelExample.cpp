#include "biorbd.h"

///
/// \brief main Modify a model and write it back to a new .bioMod file
/// \return Nothing
///
/// This examples shows how to
///     1. Load a model
///     2. Modify one of its properties (here, the mass of a segment)
///     3. Write the modified model to a new .bioMod file
///     4. Reload the written file to confirm the change was saved
///

using namespace BIORBD_NAMESPACE;

int main() {
  // Load a predefined model
  Model model("pyomecaman.bioMod");

  // Modify the mass of the first segment
  rigidbody::Segment& segment = model.segment(0);
  std::cout << "Original mass of " << segment.name() << ": "
            << segment.characteristics().mass() << std::endl;
  segment.characteristics().setMass(segment.characteristics().mass() * 2);

  // Write the modified model to a new file
  utils::Path outputPath("writeModelExample_output.bioMod");
  Writer::writeModel(model, outputPath);

  // Reload the file to confirm the modification was properly saved
  Model reloadedModel(outputPath.relativePath());
  std::cout << "Mass of " << segment.name() << " after write/reload: "
            << reloadedModel.segment(0).characteristics().mass() << std::endl;

  return 0;
}
