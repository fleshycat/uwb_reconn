import threading

from enum import Enum

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from drone_manager.drone_manager import DroneManager

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
    def __init__(self, drone_manager: "DroneManager"):
        self._timer = None
        self.mode = Mode.QHAC
        self.drone_manager = drone_manager

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
            self.drone_manager.calculate_takeoff_offset()
            self.collection_step = 0
        elif mode == Mode.SEARCH:                       ## Search Mode is agents searching for target
            self.drone_manager.particle_filter.set_num_particles(self.drone_manager.num_particles)
            self.drone_manager.have_target = False
            self.drone_manager.target = []
            self.drone_manager.desired_yaw = 0.0
        elif mode == Mode.HAVE_TARGET:                  ## Have Target Mode is agents have target and adjust formation
            self.drone_manager.have_target = True
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