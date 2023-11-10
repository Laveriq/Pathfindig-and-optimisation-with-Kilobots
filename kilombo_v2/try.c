/* The planet orbit demonstration from the kilobotics-labs
 * https://www.kilobotics.com/labs#lab4-orbit
 *
 * Lightly modified to work in the simulator, in particular:
 * - mydata->variable for global variables
 * - callback function cb_botinfo() to report bot state back to the simulator for display
 * - spin-up motors only when required, using the helper function  smooth_set_motors()
 *
 * Modifications by Fredrik Jansson 2015
 */

#include <math.h>
#include <kilombo.h>
#include "try.h"




REGISTER_USERDATA(MyUserdata)

#ifdef SIMULATOR
#include <stdio.h>
#else
#define DEBUG 
#include "debug.h"
#endif

/* declare constants
static const uint8_t TOOCLOSE_DISTANCE = 40; // 40 mm
static const uint8_t DESIRED_DISTANCE = 60; // 60 mm
static const uint8_t WIDE_DISTANCE = 90; // 90 mm
*/

////////////////////////////    RXbuffer    /////////////////////////////////////////////

// message rx callback function. Pushes message to ring buffer.
void rxbuffer_push(message_t *msg, distance_measurement_t *dist) {
    // fprintf(fp,"%d,%d\n", kilo_uid, kilo_ticks);

    received_message_t *rmsg = &RB_back();
    rmsg->msg = *msg;
    rmsg->dist = *dist;
    RB_pushback();
}

////////////////////////////    MOTION_KB STEUERUNG    /////////////////////////////////////////////

void smooth_set_motors(uint8_t ccw, uint8_t cw)
  {
  // OCR2A = ccw;  OCR2B = cw;  //spin motor clockwise or counter clockwise 
  //spin up means     set_motors(255, 255);
      //              delay(15); -> maximum speed for both motors to bypass static friction. (Haftreibung)
  #ifdef KILOBOT 
    uint8_t l = 0, r = 0;
    if (ccw && !OCR2A) // we want left motor on, and it's off
      l = 0xff;
    if (cw && !OCR2B)  // we want right motor on, and it's off
      r = 0xff;
    if (l || r)        // at least one motor needs spin-up
      {
        set_motors(l, r);
        delay(15);
      }
  #endif
    // spin-up is done, now we set the real value
    set_motors(ccw, cw); //spin motor counter clockwise (ccw) or clockwise (cw)
  }


  void set_motion(motion_t new_motion) //switch bei motion 
  {
    switch(new_motion) {
    case STOP:
      smooth_set_motors(0,0);
      break;
    case FORWARD:
      smooth_set_motors(kilo_straight_left, kilo_straight_right);
      break;
    case LEFT:
      smooth_set_motors(kilo_turn_left, 0); 
      break;
    case RIGHT:
      smooth_set_motors(0, kilo_turn_right); 
      break;
    }
  }
////////////////////////////    BOTTYPE   /////////////////////////////////////////////     

////////////////////////////    RANDOMWALK    ///////////////////////////////////////////// 
  void randomwalk()
{            
            // Generate an 8-bit random number (between 0 and 2^8 - 1 = 255).
            int random_number = rand_hard();
            
            // Compute the remainder of random_number when divided by 4.
            // This gives a new random number in the set {0, 1, 2, 3}.
            int random_direction = (random_number % 4);
            
            // There is a 50% chance of random_direction being 0 OR 1, in which
            // case set the LED green and move forward.
            if ((random_direction == 0) || (random_direction == 1))
            {
                set_color(RGB(0, 1, 0));
                set_motion(FORWARD);
            }
            // There is a 25% chance of random_direction being 2, in which case
            // set the LED red and move left.
            else if (random_direction == 2)
            {
                set_color(RGB(1, 0, 0));
                set_motion(LEFT);
            }
            // There is a 25% chance of random_direction being 3, in which case
            // set the LED blue and move right.
            else if (random_direction == 3)
            {
                set_color(RGB(0, 0, 1));
                set_motion(RIGHT);
            }
      

 }
// Randomwalk till beacon 

void walk_till_beacon()
{ 
  // int8_t dist_b = neighbors.dist;
  if (estimate_distance < 80) {
    randomwalk();
  }
  else {
    set_motion(STOP);
    mydata->bot_type = BEACON_HOME; 

  }
}
////////////////////////////    EDGE    /////////////////////////////////////////////

uint8_t find_nearest_N_dist()
{
  uint8_t i;
  uint8_t dist = 90;

  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].dist < dist)
    {
      dist = mydata->neighbors[i].dist;
    }
  }
  return dist;
}

void edge_follow()
{
  double DESIRED_DISTANCE;
  if (find_nearest_N_dist() > DESIRED_DISTANCE)  //change made here from double desired_dist = 55 ->
  {
    set_motion(LEFT);
    printf("edge_follow - move_left\n");
  }
  // if(find_nearest_N_dist() < desired_dist)
  else
  {
    set_motion(RIGHT);
    printf("edge_follow - move_right\n");
  }
}

////////////////////////////    SETUP_MESSAGE    /////////////////////////////////////////////
void setup_message(void)
{
  mydata->message_lock = 1; // don't transmit while we are forming the message
  mydata->transmit_msg.type = NORMAL;
  mydata->transmit_msg.data[0] = kilo_uid;     // 0 low  ID
  mydata->transmit_msg.data[1] = mydata->N_Neighbors; // 1 number of neighbors
  mydata->transmit_msg.data[2] = mydata->bot_type; // 2 bottype (FOOD/HOME/...)
//  mydata->transmit_msg.data[3] = mydata->dist; // 3 numer of dist 

  mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
  mydata->message_lock = 0;
}

message_t *message_tx()
{
  if (mydata->message_lock)
    return 0;
  return &mydata->transmit_msg;
}

////////////////////////////    SETUP    /////////////////////////////////////////////
void setup ()
{
rand_seed(kilo_uid);
if (kilo_uid == 0)  //HOME_BOT
  {
    mydata->bot_type = HOME_0; //HOME ohne verbindung zu Food location
    set_color(RGB(3,0,0)); //rot
  }
  else if (kilo_uid == 100) //FOOD_BOT
  {
    mydata->bot_type = FOOD;
    set_color(RGB(3,1,0)); //orange 
  }
  else if (kilo_uid == 2)
  {
     mydata->bot_type = WALKER;
    set_color(RGB(0,3,3)); //Türkis 
  }
  else 
  {
    mydata->bot_type = SEARCHER;
    set_color(RGB(0,3,0)); //Grün
    mydata->detect_home = 1; //BOTS müssen in nähe von Home starten 
  }  
 
set_motion(STOP); 
mydata->N_Neighbors = 0; 
mydata->message_lock = 0;
setup_message();
  
}


////////////////////////////    Receive_Inputs    /////////////////////////////////////////////

void process_message()
{
  uint8_t i;
  uint16_t ID;

  uint8_t *data = RB_front().msg.data;
  ID = data [0];
  uint8_t d = estimate_distance(&RB_front().dist);

  // search the neighbor list by ID
  for (i = 0; i < mydata->N_Neighbors; i++)
    if (mydata->neighbors[i].ID == ID)
    { // found it
      break;
    }

  if (i == mydata->N_Neighbors)
  {                                     // this neighbor is not in list
    if (mydata->N_Neighbors < MAXNUMBER - 1) // if we have too many neighbors,
      mydata->N_Neighbors++;            // we overwrite the last entry
                                        // sloppy but better than overflow
  }

  // i now points to where this message should be stored
  mydata->neighbors[i].ID = ID;
  mydata->neighbors[i].timestamp = kilo_ticks;
  mydata->neighbors[i].N_Neighbors = data[2]; // 1 number of neighbors
  mydata->neighbors[i].n_bot_type = data[3]; // 2 bottype (FOOD/HOME/...) -> BOTTYPE hinzufügen (n_bot_type)
  mydata->neighbors[i].dist = d; // 3 numer of dist
    //nachricht durchreichen 
}
void receive_inputs() //wird im loop aufgerufen
{
  while (!RB_empty())
  {
    process_message();  //
    RB_popfront();
  }
  purgeNeighbors();
}

/* Process a received message at the front of the ring buffer.
 * Go through the list of neighbors. If the message is from a bot
 * already in the list, update the information, otherwise
 * add a new entry in the list
 */


////////////////////////////    prugeNeighbors     /////////////////////////////////////////////
/* Go through the list of neighbors, remove entries older than a threshold,
 * currently 2 seconds.
 */
void purgeNeighbors(void)
{
  int8_t i;
  for (i = mydata->N_Neighbors - 1; i >= 0; i--)
    if (kilo_ticks - mydata->neighbors[i].timestamp > 64) // 32 ticks = 1 s
    {                                                     // this one is too old.
      mydata->neighbors[i] = mydata->neighbors[mydata->N_Neighbors - 1];
      // replace it by the last entry
      mydata->N_Neighbors--;
    }
}



////////////////////////////    LOOP    /////////////////////////////////////////////
void loop (){
  receive_inputs();
  int8_t i;
  for (i = mydata->N_Neighbors - 1; i >= 0; i--)
  if (mydata->bot_type == FOOD)
   {}
  else if(mydata->bot_type == HOME_0){ //HOME KB ohne Food verbindung
    if (mydata->neighbors[i].n_bot_type == BEACON_FOOD){ 
        mydata->bot_type = HOME_1;  //HOME KB mit Food Verbindung
    }
  
  else if(mydata->bot_type == BEACON_HOME) { //Beacon hat verbindung zum HOME
    set_color(RGB(3,0,3));
    set_motion(STOP);
    if (mydata->neighbors[i].n_bot_type == BEACON_FOOD){
      mydata->bot_type = BEACON_FOOD;
    }
    else {}}
  else if (mydata->bot_type == BEACON_FOOD){//Beacon hat verbindung zu FOOD
    set_color(RGB(3,3,0));
    set_motion(STOP);
  }
  else if (mydata->bot_type == WALKER){
    if (mydata->neighbors[i].n_bot_type == HOME_1){

      if (mydata->neighbors[i].n_bot_type == FOOD) {
        set_color(RGB(3,3,3));
        edge_follow();
      }
      else {      
        edge_follow(); 
      }
    }
  }
  else if (mydata->bot_type == SEARCHER){

    
    walk_till_beacon;
    //Wenn Bot_type Food gefunden wurde dann bleibe stehen und werde zu beacon  (Beacon Funktion) und sende Nachricht Food_found = 1 zu nächstem Beacon
    //befolge Funktion Search (Randomwalk und Distanzermittlung[find_nearest_N_dist] und bei niedrigster Distanz dann gehe zu Funktion Beacon -> werde zum Beacon und bleibe stehen
  }
}
setup_message();

}



#ifdef SIMULATOR // シミュレータ上に情報を表示させるときに使用する
/* provide a text string for the simulator status bar about this bot */
static char botinfo_buffer[10000];
char *cb_botinfo(void)
{
  char *p = botinfo_buffer;
  p += sprintf (p, "ID: %d, Distanz: %d, Anzahl Nachbarn: %d \n", kilo_uid, mydata->dist, mydata->N_Neighbors);
  if (mydata->kilobot_state == KILOBOT_NORMAL)
    p += sprintf (p, "Distanz: KILOBOT_NORMAL\n");
  if (mydata->kilobot_state == KILOBOT_TOOCLOSE)
    p += sprintf (p, "State: KILOBOT_TOOCLOSE\n");
  if (mydata->kilobot_state == KILOBOT_WIDE)
    p += sprintf (p, "State: KILOBOT_WIDE\n");
  if (mydata->kilobot_state == KILOBOT_STOPPED)
    p += sprintf (p, "State: KILOBOT_STOPPED\n");
  p += sprintf (p, "Distance: %d ", mydata->cur_distance );
  
  return botinfo_buffer;
}
#endif



int main(void) {
  
    kilo_init(); //ハードウェアでkilobotを動かすときに使用する
    
    #ifdef DEBUG
      debug_init();
    #endif  

    SET_CALLBACK(botinfo, cb_botinfo);

    RB_init();
    kilo_message_rx = rxbuffer_push; 
    kilo_message_tx = message_tx;//メッセージを受信したときに実行されるCallBack関数

	// カーソルでフォーカスしているロボット(bot)の情報をシミュレーターに表示させる
	// manual.mdの75行目
    
  //  SET_CALLBACK(reset, setup)
    
    // bot 0 is stationary and transmits messages. Other bots orbit around it.
   // メッセージを送信しようとした時に実行されるCallBack関数
    
    kilo_start(setup, loop);

    return 0;
}
     
    
    
    
    
 
