from enum import IntEnum

class StateNum(IntEnum): 
    SCHOOL_ZONE_SIGN_RECOGNITION        = 1
    SCHOOL_ZONE_CROSSING_RECOGNITION    = 2
    SCHOOL_ZONE_RESTART                 = 3
    DYNAMIC_OBSTACLE                    = 4
    RUBBERCON_DRIVING                   = 5
    STATIC_OBSTACLE                     = 6