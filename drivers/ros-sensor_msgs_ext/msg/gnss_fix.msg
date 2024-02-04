# Describes the status of a GNSS receiver's fix.

uint8 type                            # The current fix type.
uint8 TYPE_NO_FIX = 0
uint8 TYPE_FIX = 1
uint8 TYPE_DIFFERENTIAL = 2
uint8 TYPE_RTK_FIXED = 4
uint8 TYPE_RTK_FLOAT = 5
uint8 TYPE_DEAD_RECKONING = 6

uint8 mode_selection                  # The selection type for the operating mode.
uint8 MODE_SELECTION_MANUAL = 0
uint8 MODE_SELECTION_AUTOMATIC = 1

uint8 mode                            # The current operating mode.
uint8 MODE_NONE = 0
uint8 MODE_2D = 1
uint8 MODE_3D = 2

uint8 satellite_count                 # The number of satellites used in the fix solution.