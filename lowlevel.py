import logging


class LowLevelDriverBase(object):
    def __init__(self, parent):
        self.parent = parent
        self.speed_override = False
        self._logger = logging.getLogger().getChild(self.__class__.__name__)
        self._logger.debug('inited')

    def drive(self, state, control):
        raise NotImplementedError