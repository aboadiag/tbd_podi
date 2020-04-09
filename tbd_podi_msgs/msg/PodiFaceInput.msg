Header header # When did the input happened


# Where on the X-axis and Y-axis of the screen the input happened, normalized to 0-100
float32 point_x 
float32 point_y 

# The original location of where it happened on the screen
uint32 raw_point_x
uint32 raw_point_y 

# type of input (Direct Mapping from Android's Motion Event)
# https://developer.android.com/reference/android/view/MotionEvent.html
int32 input_type

# Type of Input
int32 ACTION_DOWN = 0
int32 ACTION_UP = 1


