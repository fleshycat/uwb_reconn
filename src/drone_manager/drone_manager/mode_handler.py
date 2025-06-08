import threading

from enum import Enum

class Mode(Enum):
    QHAC = 0
    SEARCH = 1
    HAVE_TARGET = 2
    CONVERGED = 3
    COLLECTION = 4
    RETURN = 5
    COMPLETED = 6
    DONE = 7

class ModeHandler():
    def __init__(self):
        self._timer = None
        self.mode = Mode.QHAC

    def _on_timer_finish(self, mode):
        self._timer = None
        self.change_mode(mode)

    def change_mode_delay(self, mode, delay_seconds = 5.0):
        delay = delay_seconds
        if self._timer is None:
            self._timer = threading.Timer(delay, self._on_timer_finish,args=(mode,))
            self._timer.start()
        else:
            return -1

    def change_mode(self, mode):
        if self._timer is not None:
            return -1
        if mode == self.mode:
            return
        if mode == Mode.QHAC:
            self.collection_step = 0
        elif mode == Mode.SEARCH:                       ## Search Mode is agents searching for target
            self.have_target = False
        elif mode == Mode.HAVE_TARGET:                  ## Have Target Mode is agents have target and adjust formation
            self.have_target = True
            # self.change_ocm_msg_velocity()
        elif mode == Mode.COLLECTION:                   ## Collection Mode is agents collecting target
            pass
        elif mode == Mode.RETURN:                       ## Return Mode is agents returning to home position
            pass
        elif mode == Mode.COMPLETED:                      ## Landing Mode is agents landing
            pass
        self.mode = mode
        return self.mode
        
    def is_in_mode(self, mode):
        return self.mode == mode
    
    def get_mode(self):
        return self.mode