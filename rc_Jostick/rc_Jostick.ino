



#include <mavlink.h>


//main loop
#define DELAY_TIME 50 //ms    1000/DELAY_TIME =  rate
#define MAVLINK_CONNECT_TIMEOUT 60 // 3s: 3000/DELAY_TIME
#define DEBUG_RC 0
#define DEBUG_MAVLINK 0
//********************************** rc channel 

#define CHAN_COUNT 8
#define ROLL_ID 0
#define PITCH_ID 1
#define THR_ID 2
#define YAW_ID 3
#define CHAN_5_ID 4
#define CHAN_6_ID 5
#define CHAN_7_ID 6
#define CHAN_8_ID 7

#define CHAN_GPIO_TYPE 1
#define CHAN_ANALOG_TYPE 0
#define CHAN_RC_MIN_VALUE 1000
#define CHAN_RC_SCALE(x) x+ CHAN_RC_MIN_VALUE

int chan_rc_value[8]={ 0,0,0,0,  0,0,0,0 };
uint8_t chan_rc_pin_type[8]={ CHAN_ANALOG_TYPE ,CHAN_ANALOG_TYPE ,CHAN_ANALOG_TYPE ,CHAN_ANALOG_TYPE,  CHAN_ANALOG_TYPE ,CHAN_ANALOG_TYPE,     CHAN_GPIO_TYPE, CHAN_GPIO_TYPE };
int chan_rc_pin[8]= { A0,A1,A2,A3,A4,A5,8,9}; // roll pitch,thr,roll, adc key 1, adc key 2, gpio 4, gpio 5
int chan_rc_sensor_max_min_value[8][2]={
  {390,640},
  {390,640},
  {390,640},
  {330,680},
  {0,1023},
  {0,1023},
  {0,1000},//gpio
  {0,1000} //gpio
  };
void setup_chan_pin_type()
{
  int i;
  for( i = 0; i< CHAN_COUNT; i++){
    if( CHAN_GPIO_TYPE == chan_rc_pin_type[i] )
      pinMode( chan_rc_pin[i] , INPUT);
  }
}

//********************************** key status
#define KEY_MAX_COUNT 2
typedef enum KEY_FUNCTION_ID_TT
{
  FUNC_ARM = 0 , /* arm = 1, disarm = 0*/
  FUNC_LAND , /* do land = 1*/
} KEY_FUNCTION_ID;
int key_pin[2] = {9, 8};
uint8_t key_value[2]={0,0};
uint8_t key_function_status[2]={0 , 0};//the status changed after triggle happen

typedef enum LED_FUNCTION_ID_TT
{
  LED_CONNECTED = 0 , /* arm = 1, disarm = 0*/
  LED_COUNT , /* do land = 1*/
} LED_FUNCTION_ID;
uint8_t led_pin[1]={7};

void setup_key_pin_mode()
{
  int i , ret ,val;
  for ( i = 0; i< KEY_MAX_COUNT ; i++ )
  {
    pinMode( key_pin[i] , INPUT);
  }
}
void setup_led_pin_mode()
{
  int i , ret ,val;
  for ( i = 0; i< LED_COUNT ; i++ )
  {
    pinMode( led_pin[i] , OUTPUT);
    digitalWrite(led_pin[i], LOW);
  }
}

//********************************** mavlink 
#define MAVLINK_SYSID 255
#define MAVLINK_COMPID 190
uint8_t copter_sysid = 0;
uint8_t copter_compid = 0;

enum rc_autopilot_modes {
    STABILIZE =     0,  // manual airframe angle with manual throttle
    ACRO =          1,  // manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  // manual airframe angle with automatic throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5,  // automatic horizontal acceleration with automatic throttle
    RTL =           6,  // automatic return to launching point
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    LAND =          9,  // automatic landing with horizontal position control
    OF_LOITER =    10,  // deprecated
    DRIFT =        11,  // semi-automous position, yaw and throttle control
    SPORT =        13,  // manual earth-frame angular rate control with manual throttle
    FLIP =         14,  // automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    BRAKE =        17   // full-brake using inertial/GPS system, no pilot input
};
#define   ERROR_MODE  -1
    
uint8_t last_time_recived = MAVLINK_CONNECT_TIMEOUT;
mavlink_heartbeat_t g_current_heartbeat; ////custom_mode is stabliable ALT_HOLD ... ; base_mode is arm disarm
uint8_t need_init_first_mode_status = 1;
uint8_t ready_for_send_rc = 0;


//*************************************************************  rc Function

int scale_rc_value(int id, int val)
{
  return map( val, chan_rc_sensor_max_min_value[id][0], chan_rc_sensor_max_min_value[id][1], 1000, 2000);
}
int get_rc_pin_value(int id)
{
    int val;
    if( CHAN_GPIO_TYPE == chan_rc_pin_type[id] ){
      val = digitalRead(chan_rc_pin[id]);
      if( val == 1 ) val = 1000;
    }else{
       val = analogRead(chan_rc_pin[id]);
    }
    //return CHAN_RC_SCALE(val);
    return scale_rc_value(id,val);
}
void update_chan_rc_value()
{
   int i;
   
#if DEBUG_RC
  Serial.print("Rc :");
#endif
  for( i = 0; i< CHAN_COUNT; i++)
  {
    chan_rc_value[i] = get_rc_pin_value(i);
    if( chan_rc_value[i] < CHAN_RC_MIN_VALUE )
      chan_rc_value[i] = CHAN_RC_MIN_VALUE;
#if DEBUG_RC
  Serial.print(chan_rc_value[i]);
  Serial.print(",");
#endif
  }
#if DEBUG_RC
  Serial.println(";");
#endif
}

void update_rc_loop()
{
  update_chan_rc_value();
}
int get_rc(int id)
{
  return chan_rc_value[id];
}



//************************************************************* mavlink Function
int is_copter_connected()
{
  if( last_time_recived < MAVLINK_CONNECT_TIMEOUT ){
    return 1;
  }else{
    last_time_recived = MAVLINK_CONNECT_TIMEOUT;
    switch_led(LED_CONNECTED, 0);
    //Serial.flush();
    //send_heartbeat_messages();
    return 0;
  }
}
int is_copter_armed()
{
  if( (g_current_heartbeat.base_mode & (uint8_t)MAV_MODE_FLAG_SAFETY_ARMED) == 0 )
    return 0;
  else
    return 1;
}
int get_copter_mode()
{
  if( is_copter_connected() )
  {
    return g_current_heartbeat.custom_mode;
  }
  return ERROR_MODE;
}

void check_copter_mode()
{
  if( is_copter_connected() )
  {
    if(  need_init_first_mode_status )
    {
      if( is_copter_armed() ){
        need_init_first_mode_status = 0;
        ready_for_send_rc = 1;
      }
      if( STABILIZE == get_copter_mode() )
      {
        need_init_first_mode_status = 0;
        ready_for_send_rc = 1;
      }else{
        send_setmode_message(STABILIZE);
        delay(DELAY_TIME);
      }
    }
  }
}
void update_mavlink_status()
{
  if( is_copter_connected() ){
    //switch_led(LED_CONNECTED, 1);
    check_copter_mode();
  }else{
    ;
  }
  
}
int send_mavlink_message(mavlink_message_t *message)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, message);
  int ret;

  ret = Serial.write(buf, len);
  ret= ret==len ? 1:0;
#if DEBUG_MAVLINK
  if( ret == 0 )
    Serial.println("mavlink message send failed");
#endif
  return ret;
}
int send_rc_override_messages()
{
  int ret = 0;
  mavlink_rc_channels_override_t sp;
  mavlink_message_t message;
  
  
// fill with the sp 
  sp.chan1_raw = get_rc(ROLL_ID);
  sp.chan2_raw = get_rc(PITCH_ID);
  sp.chan3_raw = get_rc(THR_ID);
  sp.chan4_raw = get_rc(YAW_ID);
  sp.chan5_raw = get_rc(CHAN_5_ID);
  sp.chan6_raw = get_rc(CHAN_6_ID);
  sp.chan7_raw = get_rc(CHAN_7_ID);
  sp.chan8_raw = get_rc(CHAN_8_ID);
  sp.target_component = 1;
  sp.target_system = 1;
  

  mavlink_msg_rc_channels_override_encode(MAVLINK_SYSID, MAVLINK_COMPID ,&message, &sp);
  ret = send_mavlink_message(&message);
  return ret;
}

int send_setmode_message(int mode)
{
  mavlink_set_mode_t mode_sp;
  mavlink_message_t message;
  int ret;
  
  if( 0 == is_copter_connected() )
    return 0;
    
	mode_sp.base_mode = (uint8_t)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;// g_current_heartbeat.base_mode;// MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  ;// MAV_MODE_FLAG_DECODE_POSITION_SAFETY ;
  mode_sp.custom_mode = (uint32_t)mode;
  mavlink_msg_set_mode_encode(copter_sysid, copter_compid, &message , &mode_sp);
  ret = send_mavlink_message(&message);
  return ret;
  
}

int send_arm_disarm_message(int arm)
{
  mavlink_command_long_t sp;
  mavlink_message_t message;
  int ret;
  
  if( 0 == is_copter_connected() )
    return 0;
    
	sp.command = (uint16_t)MAV_CMD_COMPONENT_ARM_DISARM;
	sp.target_system = (uint8_t)MAVLINK_SYSID;//control_data.system_id;
	sp.target_component = (uint8_t)MAVLINK_COMPID; 
  sp.param1= (float)arm;

  mavlink_msg_command_long_encode(copter_sysid, copter_compid  ,&message,&sp);
	ret = send_mavlink_message(&message);
  return ret;
}

int send_heartbeat_messages()
{
  int ret = 0;
  mavlink_heartbeat_t sp;
  mavlink_message_t message;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  

// fill with the sp 
  sp.type = 6;//MAV_TYPE_GCS;

  mavlink_msg_heartbeat_encode(MAVLINK_SYSID, MAVLINK_COMPID ,&message, &sp);
  ret = send_mavlink_message(&message);
  return ret;
}
void sync_g_heartbeat_message(mavlink_message_t *msg)
{
  g_current_heartbeat.custom_mode = mavlink_msg_heartbeat_get_custom_mode(msg);
  g_current_heartbeat.type = mavlink_msg_heartbeat_get_type(msg);
  g_current_heartbeat.autopilot = mavlink_msg_heartbeat_get_autopilot(msg);
  g_current_heartbeat.base_mode = mavlink_msg_heartbeat_get_base_mode(msg);
  g_current_heartbeat.system_status = mavlink_msg_heartbeat_get_system_status(msg);
  g_current_heartbeat.mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(msg);
  copter_sysid = msg->sysid;
  copter_compid = msg->compid;
}

void receive_and_handleMessage() { 
  mavlink_message_t msg; 
  mavlink_status_t status;
  int recived = 0;
  
  //receive data over serial 
  while(Serial.available() > 0) { 
    uint8_t c = Serial.read();
    if( mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status) ) { 
      // Handle message
      recived = 1;
      
  #if DEBUG_MAVLINK
      Serial.print("get a mavlink package; sysid=");
      Serial.print(msg.sysid);
      Serial.print(", compid=");
      Serial.print(msg.compid);
      Serial.print(", msgid=");
      Serial.println(msg.msgid);
 #endif
      switch(msg.msgid) {
              case MAVLINK_MSG_ID_HEARTBEAT: {
                sync_g_heartbeat_message(&msg);
                send_heartbeat_messages();
                triggle_led(LED_CONNECTED);
                break;
              }
              default:
                //Do nothing
                break;
      }
    }
  }
  if( recived ){
    last_time_recived = 0;
  }else{
    last_time_recived ++;
  }
  update_mavlink_status();  

}





void mavlink_msg_loop() { 
  //if( 1 == ready_for_send_rc )
    //send_rc_override_messages();
  receive_and_handleMessage();
}




//********************************************key function


void do_key_func_arm_disarm()
{
  int arm=0;
  int mode;
  
  mode  = get_copter_mode();
  if( is_copter_connected())
  {
    arm = is_copter_armed()==1 ? 0 : 1;
    //if( arm==1 && mode != STABILIZE )
      //return; // if need armed , but mode is not stablize , do failed
    send_arm_disarm_message(arm);
  }
}
void do_key_func_land()
{
  if( is_copter_connected() && is_copter_armed())
    send_setmode_message(LAND);
}
void do_key_event(int id)
{
  if( id == FUNC_ARM ){
    do_key_func_arm_disarm();
  }else if( id == FUNC_LAND ){
    do_key_func_land();
  }
}
void update_key_loop()
{
  int i , ret ,val;
  for ( i = 0; i< KEY_MAX_COUNT ; i++ )
  {
    val = digitalRead(key_pin[i]);
    if( val !=   key_value[i] )
    {
      if( val == 0 ){
        // happen a triggle
        do_key_event(i); 
      }
      key_value[i] = val;
    }
  }
}




//*****************************************led function
void switch_led(int id, int on)
{
    if(id > LED_COUNT)
      return;
    digitalWrite(led_pin[id], on? HIGH:LOW);
}
void triggle_led(int id)
{
  if(id > LED_COUNT)
    return;
   int led = digitalRead(led_pin[id]);
   digitalWrite(led_pin[id], led? LOW:HIGH);
}




//***************************************************** main function

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(57600);
  setup_chan_pin_type();
  setup_key_pin_mode();
  setup_led_pin_mode();
}

void loop() {
  update_rc_loop();
  update_key_loop();
  mavlink_msg_loop();
  
  delay(DELAY_TIME);
}


