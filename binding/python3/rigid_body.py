def marker_index(model, marker_name):
    try:
        return [n.to_string() for n in model.markerNames()].index(marker_name)
    except ValueError:
        raise ValueError(f"{marker_name} is not in the biorbd model")


def segment_index(model, segment_name):
    try:
        return [model.segment(i).name().to_string() for i in range(model.nbSegment())].index(segment_name)
    except ValueError:
        raise ValueError(f"{segment_name} is not in the biorbd model")
