#include "biorbd.h"

///
/// \brief main Compute few steps of a Kalman filter to perform inverse kinematics
/// \return Nothing
///
/// This examples shows how to
///     1. Load a model
///     2. Generate data (should be acquired via real data)
///     3. Create a Kalman filter
///     4. Apply the Kalman filter (inverse kinematics)
///     5. Plot the kinematics (Q), velocity (Qdot) and acceleration (Qddot)
///
/// Please note that this example will work only with the Eigen backend.
/// Please also note that kalman will be VERY slow if compiled in debug
///
int main()
{
    // Load a predefined model
    biorbd::Model model("pyomecaman.bioMod");

    // Generate random data (3 frames)
    biorbd::rigidbody::GeneralizedCoordinates Q_simulated(model);
    Q_simulated.setRandom();
    std::cout << "Target Q = " << Q_simulated.transpose() << std::endl;
    std::vector<biorbd::rigidbody::NodeSegment> bodyMarkers = model.markers(Q_simulated);
    std::vector< std::vector<biorbd::rigidbody::NodeSegment> > markersOverFrames;
    markersOverFrames.push_back(bodyMarkers);
    markersOverFrames.push_back(bodyMarkers);
    markersOverFrames.push_back(bodyMarkers);

    // Create a Kalman filter
    biorbd::rigidbody::KalmanReconsMarkers kalman(model);

    // Perform the kalman filter for each frame (the first frame is much longer than the next)
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedAcceleration Qddot(model);
    for (std::vector<biorbd::rigidbody::NodeSegment> targetMarkers : markersOverFrames){
        kalman.reconstructFrame(model, targetMarkers, &Q, &Qdot, &Qddot);

        // Print the kinematics to the console
        std::cout << Q.transpose() << std::endl;
    }

    return 0;
}