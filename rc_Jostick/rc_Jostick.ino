



#include <mavlink.h>


//main loop
#define DELAY_TIME 50 //ms    1000/DELAY_TIME =  rate
#define MAVLINK_CONNECT_TIMEOUT 60 // 3s: 3000/DELAY_TIME
#define DEBUG_RC 0
#define DEBUG_MAVLINK 1
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

//********************************** key status
#define KEY_MAX_COUNT 2
typedef enum KEY_FUNCTION_ID_TT
{
  FUNC_ARM = 0 , /* arm = 1, disarm = 0*/
  FUNC_LAND , /* do land = 1*/
} KEY_FUNCTION_ID;
int key_pin[2] = {8, 9};
uint8_t key_value[2]={0,0};
uint8_t key_function_status[2]={0 , 0};//the status changed after triggle happen

//********************************** mavlink 
#define MAVLINK_SYSID 255
#define MAVLINK_COMPID 190

uint8_t last_time_recived = DELAY_TIME;
mavlink_heartbeat_t g_current_heartbeat; ////custom_mode is stabliable ALT_HOLD ... ; base_mode is arm disarm


//*************************************************************  rc Function



void setup_chan_pin_type()
{
  int i;
  for( i = 0; i< CHAN_COUNT; i++){
    if( CHAN_GPIO_TYPE == chan_rc_pin_type[i] )
      pinMode( chan_rc_pin[i] , INPUT);
  }
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
    return CHAN_RC_SCALE(val);
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
    return 0;
  }
}
int is_copter_armed()
{
  if( g_current_heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED != 0 )
    return 1;
  else
    return 0;
}
void update_mavlink_status()
{
  if( is_copter_connected() ){
    ;
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
    
	mode_sp.base_mode = g_current_heartbeat.base_mode;// MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  ;// MAV_MODE_FLAG_DECODE_POSITION_SAFETY ;
  mode_sp.custom_mode = mode;
  mavlink_msg_set_mode_encode(MAVLINK_SYSID, MAVLINK_COMPID , &message , &mode_sp);
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
   
	sp.command = MAV_CMD_COMPONENT_ARM_DISARM;
	sp.target_system = MAVLINK_SYSID;//control_data.system_id;
	sp.target_component == MAVLINK_COMPID; 
  sp.param1= arm;

  mavlink_msg_command_long_encode(MAVLINK_SYSID, MAVLINK_COMPID ,&message,&sp);
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
  send_rc_override_messages();
  receive_and_handleMessage();
}




//********************************************key function

void setup_key_pin_mode()
{
  int i , ret ,val;
  for ( i = 0; i< KEY_MAX_COUNT ; i++ )
  {
    pinMode( key_pin[i] , INPUT);
  }
}

void do_key_func_arm_disarm()
{
  int arm=0;
  if( is_copter_connected() )
  {
    if( is_copter_armed() )
      arm  = 1;
     send_arm_disarm_message(arm);
  }
}
void do_key_func_land()
{
  ;
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










//***************************************************** main function

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(57600);
  setup_chan_pin_type();
  setup_key_pin_mode();
}

void loop() {
  update_rc_loop();
  update_key_loop();
  mavlink_msg_loop();
  
  delay(DELAY_TIME);
}


