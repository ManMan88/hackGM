import logging


class LowLevelDriverBase(object):
    def __init__(self):
        self.speed_override = False
        self._logger = logging.getLogger().getChild(self.__class__.__name__)

    def drive(self, state, control):
        raise NotImplementedError