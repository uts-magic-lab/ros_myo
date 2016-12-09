# Arm message for the Thalmic Myo. Using firmware > 1.0, 
# Thalmic Gesture Recognition is possible after sync gesture is performed.

# arm represents which arm the device is on
uint8 UNKNOWN=0
uint8 RIGHT=1
uint8 LEFT=2
uint8 arm

# xdir represents the direction of the x-axis
# Sharing UNKNOWN=0
uint8 X_TOWARD_WRIST=11
uint8 X_TOWARD_ELBOW=12
uint8 xdir
