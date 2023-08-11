from enum import IntEnum

class StateNum(IntEnum):
    NORMAL_DRIVING_WITH_YOLO            =-1
    NORMAL_DRIVING                      = 0
    SCHOOL_ZONE_SIGN_RECOGNITION        = 1
    SCHOOL_ZONE_CROSSING_RECOGNITION    = 2
    SCHOOL_ZONE_RESTART                 = 3
    DYNAMIC_OBSTACLE                    = 4
    RUBBERCON_DRIVING                   = 5
    STATIC_OBSTACLE                     = 6

class YoloNum(IntEnum):
    NOTHING                             =-1
    STATIC_OBSTACLE                     = 0
    DYNAMIC_OBSTACLE                    = 1
    RUBBERCONE                          = 2
    CROSS_WALK                          = 3
    ACURO_MARKER                        = 4