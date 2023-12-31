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

#ifdef SIMULATOR
#include <stdio.h> // for printf
#else
#include <avr/io.h>  
#endif

REGISTER_USERDATA(USERDATA)

// declare constants
static const uint8_t TOOCLOSE_DISTANCE = 40; // 40 mm
static const uint8_t DESIRED_DISTANCE = 60; // 60 mm
static const uint8_t WIDE_DISTANCE = 80; // 80 mm

/* Helper function for setting motor speed smoothly
 */
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

// 前後左右の動きを呼び出しやすいように定義した
void set_motion(motion_t new_motion)
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
       
 
void kilobot_normal() 
{
  if (mydata->cur_distance < TOOCLOSE_DISTANCE) {
        mydata->kilobot_state = KILOBOT_TOOCLOSE;
    } else {
        if (mydata->cur_distance < DESIRED_DISTANCE)
            set_motion(LEFT);
      else mydata->kilobot_state = KILOBOT_WIDE;
    }
}


void kilobot_tooclose() {
  if ((mydata->cur_distance >= DESIRED_DISTANCE) && (mydata->cur_distance < WIDE_DISTANCE))
    mydata->kilobot_state = KILOBOT_NORMAL;
  else
    set_motion(FORWARD);

}


void kilobot_wide() {
  if (mydata->cur_distance < TOOCLOSE_DISTANCE) {
        mydata->kilobot_state = KILOBOT_TOOCLOSE;
    } else {
        if (mydata->cur_distance < DESIRED_DISTANCE)
            mydata->kilobot_state = KILOBOT_NORMAL;

        else if (mydata->cur_distance >= WIDE_DISTANCE){
        mydata -> kilobot_state = KILOBOT_STOPPED;
        }
    }

}

void kilobot_stopped() {
  if (mydata->cur_distance >= WIDE_DISTANCE) {
    set_motion(STOP);
    set_color(RGB(0,1,2)); // bilnking (green) if a bot is not mooving
  }
  
}




void loop() {
    // Update distance estimate with every message
    if (mydata->new_message) {// message
        mydata->new_message = 0;
        mydata->cur_distance = estimate_distance(&mydata->dist);//uint8

	
    } else if (mydata->cur_distance == 0) // skip state machine if no distance measurement available
        return;

    // bot 0 is stationary. Other bots orbit around it.
    if (kilo_uid == 0 )//BOT 0 = Start Punk 
      return;
    if (kilo_uid == 100)//BOT 100 = Ziel Punk/Futterquelle
      return;
    
  

    switch(mydata->kilobot_state) {
            case KILOBOT_NORMAL:
                kilobot_normal();
                break;
            case KILOBOT_TOOCLOSE:
                kilobot_tooclose();
                break; 
            case KILOBOT_WIDE:
                kilobot_wide();
                break;
            case KILOBOT_STOPPED:
                kilobot_stopped();
                break;
    }
}

void message_rx(message_t *m, distance_measurement_t *d) {
    mydata->new_message = 1;
    mydata->dist = *d;
} 
void setup_message(void)
{
  mydata->transmit_msg.type = NORMAL;
  mydata->transmit_msg.data[0] = kilo_uid & 0xff; //low byte of ID, currently not really used for anything
  
  //finally, calculate a message check sum
  mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
  }
  message_t *message_tx() 
{
  return &mydata->transmit_msg;
}
void setup() {

    mydata->kilobot_state = KILOBOT_NORMAL;
    mydata->cur_distance = 0;
    mydata->new_message = 0;

  setup_message();

  if (kilo_uid == 0)
    {set_color(RGB(0,0,3));} // color of the stationary bot (blue)
  else if (kilo_uid == 100)
    {set_color(RGB(3,0,0));} // color for food destination (red)
   else
    set_color(RGB(0,3,0)); // color of the moving bot (green)
}


#ifdef SIMULATOR // シミュレータ上に情報を表示させるときに使用する
/* provide a text string for the simulator status bar about this bot */
static char botinfo_buffer[10000];
char *cb_botinfo(void)
{
  char *p = botinfo_buffer;
  p += sprintf (p, "ID: %d \n", kilo_uid);
  if (mydata->kilobot_state == KILOBOT_NORMAL)
    p += sprintf (p, "State: KILOBOT_NORMAL\n");
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


int main() {
    kilo_init(); //ハードウェアでkilobotを動かすときに使用する
    kilo_message_rx = message_rx; //メッセージを受信したときに実行されるCallBack関数

	// カーソルでフォーカスしているロボット(bot)の情報をシミュレーターに表示させる
	// manual.mdの75行目
    SET_CALLBACK(botinfo, cb_botinfo);
    
    // bot 0 is stationary and transmits messages. Other bots orbit around it.
    if (kilo_uid == 0)
      kilo_message_tx = message_tx; // メッセージを送信しようとした時に実行されるCallBack関数
    
    kilo_start(setup, loop);

    return 0;
}

    
    
    
    
  
