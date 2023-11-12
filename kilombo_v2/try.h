
#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif

#include <math.h>

 // brauchen wir evtl. 
#define GENES 6
#define DIFFGENES 6  // Maximal number of diffusing genes, limited by the message length.
#define MAXNUMBER 20 //MAXNUMBER of Neighbors
#define RB_SIZE 16  // Ring buffer size. Choose a power of two for faster code
                    // memory usage: 16*RB_SIZE
                    // 8 works too, but complains in the simulator
                    // when the bots are very dense


// declare motion variable type
typedef enum {
    LEFT,
    RIGHT,
    STOP,
    FORWARD,
    
} motion_t;

// declare state variable type
typedef enum {
    KILOBOT_TOOCLOSE,
    KILOBOT_NORMAL,
    KILOBOT_WIDE,
    KILOBOT_STOPPED, 
} kilobot_state_t;

typedef enum {
  HOME_0,
  HOME_1,
  FOOD,
  BEACON_HOME,
  BEACON_FOOD,
  SEARCHER,
  WALKER,

} bot_type_t;

typedef struct{
  message_t msg;
  distance_measurement_t dist;
}received_message_t;


typedef struct{
  uint16_t ID;
  uint8_t dist;
  uint8_t n_bot_type;
  uint8_t N_Neighbors;
  uint32_t timestamp;
} Neighbor_t; 

typedef struct {
  kilobot_state_t kilobot_state; // self declare
  uint8_t cur_distance;
  uint8_t new_message;
  distance_measurement_t dist; // kilolib.h

  message_t transmit_msg; // kilolib.h
  uint8_t new_message;
  received_message_t RXBuffer[RB_SIZE];
  uint8_t RXHead, RXTail; 
  int N_Neighbors;
  char message_lock;

  uint8_t bot_type;
  uint8_t move_type;
  uint8_t detect_food;
  uint8_t receive_food;
  uint8_t detect_home;
  Neighbor_t neighbors[MAXNUMBER];
   
} MyUserdata;




extern uint8_t NGenes;
// brauchen wir evtl. 




// Ring buffer operations. Taken from kilolib's ringbuffer.h
// but adapted for use with mydata->

// Ring buffer operations indexed with head, tail
// These waste one entry in the buffer, but are interrupt safe:
//   * head is changed only in popfront
//   * tail is changed only in pushback
//   * RB_popfront() is to be called AFTER the data in RB_front() has been used
//   * head and tail indices are uint8_t, which can be updated atomically
//     - still, the updates need to be atomic, especially in RB_popfront()
#define RB_init() {	\
    mydata->RXHead = 0; \
    mydata->RXTail = 0;\
}

#define RB_empty() (mydata->RXHead == mydata->RXTail)

#define RB_full()  ((mydata->RXHead+1)%RB_SIZE == mydata->RXTail)

#define RB_front() mydata->RXBuffer[mydata->RXHead]

#define RB_back() mydata->RXBuffer[mydata->RXTail]

#define RB_popfront() mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;

#define RB_pushback() {\
    mydata->RXTail = (mydata->RXTail+1)%RB_SIZE;\
    if (RB_empty())\
      { mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;	\
	printf("Full.\n"); }				\
  }
