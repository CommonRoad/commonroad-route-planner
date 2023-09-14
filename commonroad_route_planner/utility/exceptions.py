class NoSourceLaneletIdException(Exception):
    def __init__(self):
        self.message = "<RoutePlanner> No initial position given."


class NoPathFoundException(Exception):
    def __init__(self, message):
        self.message = message
