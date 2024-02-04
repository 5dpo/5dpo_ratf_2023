# A global position measurement from a GNSS sensor.

bool fix_3d                 # Indicates if the fix is 3-dimensional (includes altitude/separation).

float64 latitude            # Degrees latitude (positive = N, negative = S)
float64 longitude           # Degrees longitude (positive = E, negative = W)
float32 altitude            # Orthometric altitude above geoid/MSL [m]
float32 separation          # Geoid separation above the ellipsoid [m]