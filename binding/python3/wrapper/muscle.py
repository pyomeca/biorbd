from ..biorbd import Muscle as MuscleBiorbd


class Muscle:
    def __init__(self, muscle: MuscleBiorbd):
        self._muscle = muscle

    @property
    def internal(self) -> MuscleBiorbd:
        """
        Get the internal muscle of the Muscle instance.

        Returns
        -------
        The internal muscle. It can be used to access any resources that are not yet wrapped in Python binder.
        """
        return self._muscle
