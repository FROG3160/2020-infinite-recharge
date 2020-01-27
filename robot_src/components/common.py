class PID:
    """Class that holds contants for PID controls"""

    # TODO: Add methods to apply to motor?

    def __init__(
        self, slot: int = 0, p: float = 0, i: float = 0,
        d: float = 0, f: float = 0
    ):
        self.slot = slot
        self.p = p
        self.i = i
        self.d = d
        self.f = f


