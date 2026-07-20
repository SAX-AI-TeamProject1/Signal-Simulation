# A discrete command produced by the gesture-recognition model, which lives in
# the separate perception repository. This package owns the wire format for that
# hand-off, so both repositories build against this definition.

uint8 UNKNOWN=0
uint8 STOP=1
uint8 FORWARD=2
uint8 TURN_LEFT=3
uint8 TURN_RIGHT=4

std_msgs/Header header

# One of the constants above.
uint8 command

# Model confidence in [0.0, 1.0]. The controller refuses to act on anything
# below its configured threshold and stops instead.
float32 confidence
