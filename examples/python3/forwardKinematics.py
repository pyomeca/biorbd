import numpy as np
import biorbd

try:
    from BiorbdViz.biorbd_vtk import VtkModel, VtkWindow
    import pyomeca
    biorbd_viz_found = True
except ModuleNotFoundError:
    biorbd_viz_found = False

#
# This examples shows how to
#     1. Load a model
#     2. Position the model at a chosen position (Q)
#     3. Compute the position of the skin markers at that position (Forward kinematics)
#     4. Print them to the console
#
# Please note that this example will work only with the Eigen backend
#


# Load a predefined model
model = biorbd.Model("../pyomecaman.bioMod")
nq = model.nbQ()
n_mark = model.nbMarkers()
n_frames = 20

# Generate clapping gesture data
qinit = np.array([0, 0, -.3, 0.35, 1.15, -0.35, 1.15, 0, 0, 0, 0, 0, 0])
qmid = np.array([0, 0, -.3, 0.5, 1.15, -0.5, 1.15, 0, 0, 0, 0, 0, 0])
qfinal = np.array([0, 0, -.3, 0.35, 1.15, -0.35, 1.15, 0, 0, 0, 0, 0, 0])
q = np.concatenate((np.linspace(qinit, qmid, n_frames).T, np.linspace(qmid, qfinal, n_frames).T), axis=1)

# Proceed with the forward kinematics
markers = np.ndarray((3, model.nbMarkers(), 2*n_frames))
for i, q in enumerate(q.T):
    markers[:, :, i] = np.array([mark.to_array() for mark in model.markers(q)]).T

# Print the first frame in the console
print(markers[:, :, 0])

# Animate the markers 
if biorbd_viz_found:
    vtkWindow = VtkWindow(background_color=(0.5, 0.5, 0.5))
    vtkModel = VtkModel(vtkWindow, markers_color=(0, 0, 1), markers_size=0.01)
    i = 0
    markers = pyomeca.Markers(markers)
    while vtkWindow.is_active:
        # Update the graph
        vtkModel.update_markers(markers[:, :, [i]])
        vtkWindow.update_frame()
        i = (i + 1) % markers.shape[2]

