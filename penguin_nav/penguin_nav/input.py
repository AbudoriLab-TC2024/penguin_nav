import threading


class Input(threading.Thread):
    """Non-blocking input class which runs in a threading."""

    def __init__(self):
        super().__init__(daemon=True)
        self._txt = None
        self._prompted = False
        self.start()

    def run(self):
        while True:
            self._txt = input()

    def input(self, message):
        """Capture the input

        This will return the contents if some input exists, None is returned if no input exists.
        """
        if not self._prompted:
            print(message)
            self._prompted = True

        txt = self._txt
        if txt is not None:
            self._prompted = False
        self._txt = None
        return txt
