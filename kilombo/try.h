
#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif


// declare motion variable type
typedef enum {
    STOP,
    FORWARD,
    LEFT,
    RIGHT
} motion_t;

// declare state variable type
typedef enum {
    KILOBOT_TOOCLOSE,
    KILOBOT_NORMAL,
    KILOBOT_WIDE,
} kilobot_state_t;

// declare variables

typedef struct 
{
  kilobot_state_t kilobot_state; // self declare
  uint8_t cur_distance;
  uint8_t new_message;
  distance_measurement_t dist; // kilolib.h 

  message_t transmit_msg; // kilolib.h
} USERDATA;




