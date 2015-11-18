



#include <mavlink.h>



#define BOARD_PRO_MICRO 1
#define BOARD_UNO 2
//############################# change this for you board
#define MY_BOARD_TYPE BOARD_PRO_MICRO
//#############################
#define gcsSerial Serial
#define teleSerial Serial2
//#define lcdSerial Serial
#define wifiSerial Serial3 
#define gcsConfigSerial Serial1

uint8_t gcsCOM =MAVLINK_COMM_0;
uint8_t wifiCOM= MAVLINK_COMM_1;
uint8_t teleCOM =MAVLINK_COMM_2;
uint8_t gcsConfigCOM = MAVLINK_COMM_3;

#define GCS_ID 0
#define WIFI_ID 1
#define TELEM_ID 2
#define GCS_CONFIG_ID 3

uint8_t connectStatus = 1<<TELEM_ID ; //1<<TELEM_ID = telem 1<<GCS_ID = phone 4G just one use for connect
unsigned long lastPackageTime[3]={0,0,0}; // 0=gcs; 2=telem; 1=wifi  
#define UART_BUFFER_LEN 2048
uint8_t uartBuffer[UART_BUFFER_LEN];
uint8_t mavlinkBuffer[MAVLINK_MAX_PACKET_LEN];
uint8_t isActivityPath[3]={0,0,0};

#define COPTER_SEND_RC_MS 20
#define GCS_SEND_RC_MS 300
#define REPORT_STATUS_MS 200
unsigned long copter_last_send_rc_ms=0;
unsigned long gcs_last_send_rc_ms=0;
unsigned long last_report_gcs_ms =0;

uint8_t bleConnected = 0;//as using ble connect to gcs for config, I send rc and msg after gcs sended a package to me
#define GCS_USE_BLE 1 //1 gcs 0 gcsConfig

//main loop
#define DELAY_TIME 20 //ms    1000/DELAY_TIME =  rate
#define MAVLINK_CONNECT_TIMEOUT 150 // 3s: 3000/DELAY_TIME
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

#define CHAN_MODE_TYPE 2
#define CHAN_GPIO_TYPE 1
#define CHAN_ANALOG_TYPE 0
#define CHAN_RC_MODE_ID   CHAN_5_ID
#define CHAN_RC_MODE_MIN   100
#define CHAN_RC_MODE_MAX   1900
#define CHAN_RC_MODE_1_VALUE   1000
#define CHAN_RC_MODE_2_VALUE   1300
#define CHAN_RC_MODE_3_VALUE   1400
#define CHAN_RC_MODE_4_VALUE   1550
#define CHAN_RC_MODE_5_VALUE   1700
#define CHAN_RC_MODE_6_VALUE   1800

#define CHAN_RC_MIN_VALUE 1000
#define CHAN_RC_MAX_VALUE 2000
#define CHAN_RC_SCALE(x) x+ CHAN_RC_MIN_VALUE

int need_cali_rc = 0;
int chan_rc_value[8]={ 0,0,0,0, CHAN_RC_MODE_1_VALUE ,0,0,0 };
uint8_t chan_rc_pin_type[8]={ CHAN_ANALOG_TYPE ,CHAN_ANALOG_TYPE ,CHAN_ANALOG_TYPE ,CHAN_ANALOG_TYPE, CHAN_MODE_TYPE,  CHAN_ANALOG_TYPE ,CHAN_ANALOG_TYPE, CHAN_ANALOG_TYPE };//CHAN_GPIO_TYPE ,CHAN_GPIO_TYPE, CHAN_GPIO_TYPE };
int chan_rc_pin[8]= { A0,A1,A2,A3, 8 ,A4, A5,A6}; // roll pitch,thr,roll, adc key 1, adc key 2, gpio 4, gpio 5
int chan_rc_sensor_max_min_value[8][3]={
  {0, 0 ,1023},
  {0, 0 ,1023},
  {0, 0 ,1023},
  {0, 0 ,1023},
  {0, 0 ,1023}, // mode type , no sense
  {0, 0 ,1023},//step key
  {0, 0 ,1023},//mount pitch
  {0, 0 ,1023} //mount roll
  };
int chan_rc_min_max_tmp[8][2]={
  {0,1},
  {0,1},
  {0,1},
  {0,1},//1016},
  {0,1}, // mode type , no sense
  {0,1},//gpio
  {0,1},//gpio
  {0,1} //gpio
  };
int revert_rc_mask = 0x0 | (1 << PITCH_ID) ;//set mask  1<<ROLL_ID , will revert roll chan
void setup_chan_pin_type()
{
  int i;
  for( i = 0; i< CHAN_COUNT; i++){
    if( CHAN_GPIO_TYPE == chan_rc_pin_type[i] )
    //if( CHAN_MODE_TYPE != chan_rc_pin_type[i] )
      pinMode( chan_rc_pin[i] , INPUT);
  }
}

//********************************** key status
//#define KEY_MAX_COUNT 2
typedef enum KEY_FUNCTION_ID_TT
{
  FUNC_ARM = 0 , /* arm = 1, disarm = 0*/
  FUNC_RTL , /* do land = 1*/
  FUNC_SWITCH_MODE,
  KEY_MAX_COUNT
} KEY_FUNCTION_ID;
int key_pin[KEY_MAX_COUNT] = {30, 31, 32};
uint8_t key_value[KEY_MAX_COUNT]={0,0,0};
uint8_t key_function_status[KEY_MAX_COUNT]={0 , 0, 0};//the status changed after triggle happen

void setup_key_pin_mode()
{
  int i , ret ,val;
  for ( i = 0; i< KEY_MAX_COUNT ; i++ )
  {
    pinMode( key_pin[i] , INPUT);
  }
}

//******************************************* led

typedef enum LED_FUNCTION_ID_TT
{
  LED_GCS_CONNECTED = 0 , /* arm = 1, disarm = 0*/
  LED_WIFI_CONNECTED , /* arm = 1, disarm = 0*/
  LED_TELEM_CONNECTED , /* arm = 1, disarm = 0*/  
  LED_COUNT , /* do land = 1*/
} LED_FUNCTION_ID;

uint8_t led_pin[LED_COUNT]={40,41,42};
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
  if( DEBUG_RC == 1 )
      return val;
  else{
      return map( val, chan_rc_sensor_max_min_value[id][0], chan_rc_sensor_max_min_value[id][2], CHAN_RC_MIN_VALUE, CHAN_RC_MAX_VALUE);
  }
}
void update_rc_trim_value()
{
   int t,i,j,id ,val;
   int times = 30;
   int chan_count  = 3;
   int ids[3]={ROLL_ID, PITCH_ID, YAW_ID};
   int sum[3]={0,0,0};
   int sum_val;
 
   for ( i = 0; i< times; i++ )
   {
       for( j = 0; j< chan_count; j++)
       {
           id = ids[j];
           val = analogRead( chan_rc_pin[ id ] );
           sum[j] += val;
       }
       delay(50);
   }

   for ( i = 0 ; i< chan_count; i++)
   {
          sum[i] = sum[i]/times;
          id = ids[i];
          chan_rc_sensor_max_min_value[id][1] = sum[i];
          chan_rc_min_max_tmp[id][0] = chan_rc_sensor_max_min_value[id][1]- 10;
          chan_rc_min_max_tmp[id][1] = chan_rc_sensor_max_min_value[id][1]+ 10;
   }
   
   need_cali_rc =1;
}
void update_rc_min_max_vaule(int id)
{
    int max_to_trim ;
    int min_to_trim;
    int rang;
    
    max_to_trim = chan_rc_min_max_tmp[id][1] - chan_rc_sensor_max_min_value[id][1];
    min_to_trim = chan_rc_sensor_max_min_value[id][1] - chan_rc_min_max_tmp[id][0];
    
    if( max_to_trim < min_to_trim ){
      chan_rc_sensor_max_min_value[id][0] = chan_rc_sensor_max_min_value[id][1] - max_to_trim;
      chan_rc_sensor_max_min_value[id][2] = chan_rc_sensor_max_min_value[id][1] + max_to_trim;
    }else{
      chan_rc_sensor_max_min_value[id][0] = chan_rc_sensor_max_min_value[id][1] - min_to_trim;
      chan_rc_sensor_max_min_value[id][2] = chan_rc_sensor_max_min_value[id][1] + min_to_trim;
    }
    /*
    Serial.print("id=");
    Serial.print(id);
    Serial.print("trim=");
    Serial.print(chan_rc_sensor_max_min_value[id][1]);
    Serial.print(", max=");
    Serial.print(chan_rc_sensor_max_min_value[id][2]);
        Serial.print(", min=");
        Serial.print(chan_rc_sensor_max_min_value[id][0]);
    Serial.println(" ");
     */
}
void adc_sensor_cali(int id, int val)
{
  int i;
  //recovery the min or max value
  if( val < chan_rc_min_max_tmp[id][0] ) {
    chan_rc_min_max_tmp[id][0] = val;
    update_rc_min_max_vaule(id);
  }else if ( val > chan_rc_min_max_tmp[id][1] ){
    chan_rc_min_max_tmp[id][1] = val;
    update_rc_min_max_vaule(id);
  }
}
int check_and_revert_rc_value(int id, int val)
{
  if(0 !=  (revert_rc_mask & (1<<id)) )
    val = CHAN_RC_MAX_VALUE - (val-CHAN_RC_MIN_VALUE);
  return val;
}
void debug_rc_channel_val(int id)
{
      int val;
    if( CHAN_GPIO_TYPE == chan_rc_pin_type[id] ){
      val = digitalRead(chan_rc_pin[id]);
    }else if( CHAN_MODE_TYPE == chan_rc_pin_type[id] ){
      val = chan_rc_value[id];
    }else{
       val = analogRead(chan_rc_pin[id]);
    }
    chan_rc_value[id] = val;
      Serial.print("chan ");
      Serial.print(id);
      Serial.print("= ");
      Serial.println(val);
    
}
int get_rc_pin_value(int id)
{
    int val;
    if( CHAN_GPIO_TYPE == chan_rc_pin_type[id] ){
      val = digitalRead(chan_rc_pin[id]);
      if( val == 1 ) val = 1000;
    }else if( CHAN_MODE_TYPE == chan_rc_pin_type[id] ){
      val = chan_rc_value[id];
    }else{
       val = analogRead(chan_rc_pin[id]);
         if( 1 == need_cali_rc && id != THR_ID && id <= YAW_ID)
             adc_sensor_cali(id,val);
    }
/*
      Serial.print("chan ");
      Serial.print(id);
      Serial.print("= ");
      Serial.println(val);
*/    
    if( CHAN_MODE_TYPE == chan_rc_pin_type[id] ){
        return val;
    }else{
         //return CHAN_RC_SCALE(val);
        val = scale_rc_value(id,val); 
        return check_and_revert_rc_value(id,val);
    }
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
    if( DEBUG_RC != 1 &&   chan_rc_value[i] < CHAN_RC_MIN_VALUE &&    CHAN_MODE_TYPE != chan_rc_pin_type[i] )
      chan_rc_value[i] = CHAN_RC_MIN_VALUE;
    if( DEBUG_RC != 1 &&   chan_rc_value[i] > CHAN_RC_MAX_VALUE &&    CHAN_MODE_TYPE != chan_rc_pin_type[i] )
      chan_rc_value[i] = CHAN_RC_MAX_VALUE;
#if DEBUG_RC
  Serial.print(chan_rc_value[i]);
  Serial.print(",");
#endif
  }
#if DEBUG_RC
  Serial.println(";");
#endif
}

unsigned long rc_loop_ms = 0;
void update_rc_loop(int delay_ms)
{
  unsigned long ms = millis();
  if(  (ms - rc_loop_ms) >= delay_ms){
    update_chan_rc_value();
    rc_loop_ms = ms;
  }
}

int get_rc(int id)
{
  return chan_rc_value[id];
}
void set_rc(int id, int val)
{
  chan_rc_value[id] = val;
}
void set_revert(int id)
{
  revert_rc_mask |= (id <<1);
}
void clear_revert(int id)
{
  revert_rc_mask &= ~(id<<1);
}

//************************************************************* mavlink Function
int is_copter_connected()
{
  if( last_time_recived < MAVLINK_CONNECT_TIMEOUT ){
    return 1;
  }else{
    last_time_recived = MAVLINK_CONNECT_TIMEOUT;
    //switch_led(LED_CONNECTED, 0);
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
    //check_copter_mode();
  }else{
    ;
  }
  
}
int MAVLINK_UART = 0; //GCS, wifi, telem
int do_write_uart(uint8_t *buf, int len)
{
  int ret;
  if( MAVLINK_UART == GCS_ID ){
    //Serial.println("wirte to gcs");
    ret = gcs_write(buf,len);
  }else if( MAVLINK_UART == WIFI_ID ){
    //Serial.println("write to wifi");
    ret = wifiSerial.write(buf,len);
  }else if( MAVLINK_UART == TELEM_ID ){
    ret = teleSerial.write(buf,len);
  }else{
    ;//Serial.println("none way to send");
  }
   return ret;
}
uint8_t do_read_uart()
{
  uint8_t ret;
   if( MY_BOARD_TYPE == BOARD_PRO_MICRO ){
      ret = Serial1.read();
   }else{
      ret = Serial.read();
   } 
   return ret;
}
int is_uart_available()
{
  int ret;
   if( MY_BOARD_TYPE == BOARD_PRO_MICRO ){
      ret = Serial1.available();
   }else{
      ret = Serial.available();
   } 
   return ret;
}
int send_mavlink_message(mavlink_message_t *message)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, message);
  int ret;

  ret = do_write_uart(buf, len);
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
  while(is_uart_available() > 0) { 
    uint8_t c = do_read_uart();
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
                //triggle_led(LED_CONNECTED);
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
    send_rc_override_messages();
  //receive_and_handleMessage();
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
     // return; // if need armed , but mode is not stablize , do failed
    send_arm_disarm_message(arm);
  }
}
void do_key_func_land()
{
  if( is_copter_connected() && is_copter_armed())
    send_setmode_message(LAND);
}
void do_key_func_rtl()
{
    if( is_copter_connected() && is_copter_armed())
      send_setmode_message(RTL);
}
void do_key_func_switch_althold_stabilize()
{
   int mode = get_copter_mode();
   if( mode != ALT_HOLD )
      send_setmode_message(ALT_HOLD);
   else
      send_setmode_message(STABILIZE);
}
void do_key_func_switch_mode()
{
   int mode_rc = get_rc(CHAN_RC_MODE_ID);
   if( mode_rc != CHAN_RC_MODE_1_VALUE )
      set_rc( CHAN_RC_MODE_ID , CHAN_RC_MODE_1_VALUE );
   else
      set_rc( CHAN_RC_MODE_ID,CHAN_RC_MODE_6_VALUE );
}
void do_key_event(int id)
{
  if( id == FUNC_ARM ){
    do_key_func_arm_disarm();
  }else if( id == FUNC_RTL ){
    do_key_func_rtl();
  }else if( id == FUNC_SWITCH_MODE){
    do_key_func_switch_mode();
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

//####################################### jostick to pac cmd function
void do_set_param(char *param_id, float param_val)
{
  Serial.print("get cmd: ");
  Serial.print(param_id);
  Serial.println(param_val);
}
void deal_setting_cmd_loop()
{
  //MAV_CMD_DO_SET_PARAMETER
  mavlink_message_t msg; 
  mavlink_status_t status;
  int recived = 0;
  char param_id[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
  float param_val;
  
  //receive data over serial 
  while(is_uart_available() > 0) { 
    uint8_t c = do_read_uart();
    if( mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status) ) { 
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
              case MAVLINK_MSG_ID_PARAM_SET: {
                mavlink_msg_param_set_get_param_id(&msg , param_id);
                param_val = mavlink_msg_param_set_get_param_value(&msg);
                do_set_param(param_id,param_val);
                break;
              }
              default:
                //Do nothing
                break;
      }
    }
  }
}




//######################################################## mega funciton

inline int is_use_wifi_connect()
{
  return (connectStatus & (1<<WIFI_ID) ) != 0  ?  1:0;
}
inline int is_use_telem_connect()
{
  return (connectStatus & (1<<TELEM_ID) ) != 0  ?  1:0;
}
inline int is_use_4g_connect()
{
  return (connectStatus & (1<<GCS_ID) ) != 0  ?  1:0;
}
inline boolean set_connect_type(short type)
{
  connectStatus = type;
  return true;
}
void mega2560_uart_setup()
{
  gcsSerial.begin(57600);
  teleSerial.begin(57600);
  wifiSerial.begin(115200);
  //lcdSerial.begin(57600);
  gcsConfigSerial.begin(57600);
   
  copter_last_send_rc_ms=millis();
  gcs_last_send_rc_ms=millis();
  last_report_gcs_ms = millis();
}

void mega2560_uart_loop()
{
  mega2560_listen_telem();
  mega2560_listen_wifi();
  mega2560_listen_gcs();
}

//sync with tower ,esp8266 and mega2560{
#define MEGA2560_SYS_ID 254
enum _GCS_CMDS { 
    MEGA2560_BOARD_CMD_WIFI_CONNECT_TCP =  1,
    MEGA2560_BOARD_CMD_WIFI_SET_CONNECT_IP,
    MEGA2560_BOARD_CMD_WIFI_DISCONNECT_TCP,
    MEGA2560_BOARD_CMD_WIFI_MAX_ID,
    
    //MEGA2560_BOARD_CMD_CHANGE_PATH
    //....
    MEGA2560_BOARD_CMD_SWITCH_CONNECT,
    MEGA2560_BOARD_CMD_MAX_ID,
    GCS_CMD_REPORT_STATUS,
    GCS_CMD_MAX_ID
};
struct param_ip_data{
    uint8_t ip[4];
    uint8_t port[2] ;
};
//}
void mega2560_send_wifi_cmd(int cmd, struct param_ip_data *data)
{
  mavlink_param_set_t ps;
  mavlink_message_t msg;
  
  boolean need_send = true;
  if( MEGA2560_BOARD_CMD_WIFI_SET_CONNECT_IP == cmd ){
    ps.param_type= (byte) MEGA2560_SYS_ID;//0~10, i use 254:gcs
    ps.param_value= cmd ; //cmd
    ps.param_id[0] = (byte) data->ip[0];
    ps.param_id[1] = (byte) data->ip[1];
    ps.param_id[2] = (byte) data->ip[2];
    ps.param_id[3] = (byte) data->ip[3];
    ps.param_id[4] = (byte) data->port[0];
    ps.param_id[5] = (byte) data->port[1];
  }else if( MEGA2560_BOARD_CMD_WIFI_CONNECT_TCP == cmd ){
    ps.param_type= (byte) MEGA2560_SYS_ID;
    ps.param_value= cmd ;
  }else if( MEGA2560_BOARD_CMD_WIFI_DISCONNECT_TCP == cmd ){
    ps.param_type= (byte) MEGA2560_SYS_ID;
    ps.param_value= cmd ;
  }else{
    //false package
    need_send = false;
  }
  if( need_send ){
    mavlink_msg_param_set_encode(255,109,&msg,&ps);
    mega2560_send_mavlink(1<<WIFI_ID,&msg);
  }
}
void mega2560_handle_cmd( mavlink_message_t *msg )
{//msg from gcs 
  int p_len,cmd,value;
  /*
  mavlink_param_set_t packet;
  mavlink_msg_param_set_decode(msg,&packet);
  cmd = packet.param_value;
  
  Serial.println("I get a cmd package");
  if( cmd < MEGA2560_BOARD_CMD_WIFI_MAX_ID ){
    //the wifi board msg will send to wifi board directly
    mega2560_send_mavlink(1<<WIFI_ID,msg);
  }
  */
  mavlink_rc_channels_override_t packet;
  mavlink_msg_rc_channels_override_decode(msg,  &packet);
  cmd = packet.chan2_raw;
  switch(cmd){
    case MEGA2560_BOARD_CMD_WIFI_SET_CONNECT_IP:{
       struct param_ip_data data;
       data.ip[0]=packet.chan3_raw;data.ip[1]=packet.chan4_raw;data.ip[2]=packet.chan5_raw;data.ip[3]=packet.chan6_raw;
       data.port[0]=packet.chan7_raw & 0xff;
       data.port[1]=packet.chan7_raw>>8 & 0xff;
       mega2560_send_wifi_cmd(MEGA2560_BOARD_CMD_WIFI_SET_CONNECT_IP,&data);
       break; 
    }
    case MEGA2560_BOARD_CMD_WIFI_CONNECT_TCP:{
      mega2560_send_wifi_cmd(MEGA2560_BOARD_CMD_WIFI_CONNECT_TCP,NULL);
      break;
    }
    case MEGA2560_BOARD_CMD_WIFI_DISCONNECT_TCP:{
      mega2560_send_wifi_cmd(MEGA2560_BOARD_CMD_WIFI_DISCONNECT_TCP,NULL);
      if( is_use_wifi_connect() == 1 ){ 
        set_connect_type(1<<TELEM_ID);
      }
      break;
    }
    case MEGA2560_BOARD_CMD_SWITCH_CONNECT:{
      short connect_type= packet.chan3_raw;
      boolean ok = set_connect_type(connect_type);
    }
    default:{
      break; 
    }
  }
}

inline int gcs_write(uint8_t *buff,int len)
{
#if GCS_USE_BLE
  if( bleConnected == 1 )
#endif
  return gcsSerial.write(buff,len);
}
inline int gcsConfig_write(uint8_t *buff,int len)
{
#if GCS_USE_BLE
  //if( bleConnected == 1 )
#else
  if( bleConnected == 1 )
#endif
  return gcsConfigSerial.write(buff,len);
}
void mega2560_listen_gcs(){
    int len, i;
    mavlink_message_t msg; 
    mavlink_status_t status;
    uint16_t p_len ;
    uint8_t ids;
    
    len = gcsSerial.available();
    len = len>UART_BUFFER_LEN ? UART_BUFFER_LEN:len;
    len = gcsSerial.readBytes(uartBuffer,len);
    for( i=0; i< len; i++){
      //char c =gcsSerial.read();
      if( mavlink_parse_char(gcsCOM,  uartBuffer[i] , &msg, &status) ) { 

        if(  msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE  && mavlink_msg_rc_channels_override_get_chan1_raw(&msg)== MEGA2560_SYS_ID ) {
              mega2560_handle_cmd(&msg);
        }else{
            ids=0;
            if( is_use_telem_connect() == 1 ){
                  ids |= 1<<TELEM_ID;//teleSerial.write(mavlinkBuffer,p_len);
            }
            if( is_use_wifi_connect() == 1 ){
                ids |= 1<<WIFI_ID;//wifiSerial.write(mavlinkBuffer, p_len);
            }
            mega2560_send_mavlink(ids,&msg);
        }
      }
    }
    if( len > 0 ){
       triggle_led(LED_GCS_CONNECTED);
    #if GCS_USE_BLE
       bleConnected = 1;
    #endif
      lastPackageTime[GCS_ID] = millis();
    }
}

void mega2560_listen_telem(){
    int len, i;
    uint16_t p_len ;
    
    len = teleSerial.available();
    len = len>UART_BUFFER_LEN ? UART_BUFFER_LEN:len;
    if( len > 0 ) {
      if( is_use_telem_connect() == 1 ){
        len = teleSerial.readBytes(uartBuffer,len);
        gcs_write(uartBuffer,len);
      }
      lastPackageTime[TELEM_ID] = millis();
      triggle_led(LED_TELEM_CONNECTED);
    }
}
void mega2560_listen_wifi(){
    int len, i;
    uint16_t p_len ;
    
    len = wifiSerial.available();
    len = len>UART_BUFFER_LEN ? UART_BUFFER_LEN:len;

    if( len > 0 ) {
      if( is_use_wifi_connect() == 1 ){
        len = wifiSerial.readBytes(uartBuffer,len);
        gcs_write(uartBuffer,len);
      }
      lastPackageTime[WIFI_ID] = millis();
      triggle_led(LED_WIFI_CONNECTED);
    }
}
void mega2560_listen_gcsConfig()
{
    int len, i;
    mavlink_message_t msg; 
    mavlink_status_t status;
    uint16_t p_len ;
    uint8_t ids;
    
    len = gcsConfigSerial.available();
    len = len>UART_BUFFER_LEN ? UART_BUFFER_LEN:len;
    len = gcsConfigSerial.readBytes(uartBuffer,len);
    for( i=0; i< len; i++){
      if( mavlink_parse_char(gcsConfigCOM,  uartBuffer[i] , &msg, &status) ) { 
        if(  msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE  && mavlink_msg_rc_channels_override_get_chan1_raw(&msg)== MEGA2560_SYS_ID ) {
#if  GCS_USE_BLE
          //bleConnected = 1;
#else
          bleConnected = 1;
#endif
          mega2560_handle_cmd(&msg);
        }else{
            //unkonw cmd
        }
      }
      //lastPackageTime[GCS_ID] = millis();
    }
}

inline void mega2560_send_mavlink(uint8_t ids,mavlink_message_t *message)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, message);
  if( 0!= (ids & 1<<GCS_ID) )
    gcs_write(buf,len);
  if( 0!= (ids & 1<<TELEM_ID)  )
    teleSerial.write(buf,len);
  if( 0!= (ids & 1<<WIFI_ID)  )
    wifiSerial.write(buf,len);
  if( 0!= (ids & 1<<GCS_CONFIG_ID))
    gcsConfig_write(buf,len);
}
void mega2560_send_rc(uint8_t ids)
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
  mega2560_send_mavlink(ids,&message);
}
void mega2560_send_rc_loop(int copter_each_ms, int gcs_each_ms)
{//xx_each_ms < 0 , never send
  uint8_t ids=0;
  unsigned long ms = millis();
  if( gcs_each_ms>0 && ((ms - gcs_last_send_rc_ms) >= gcs_each_ms) ){
    ids |= 1<<GCS_CONFIG_ID;
    gcs_last_send_rc_ms = ms;
  }
  if( copter_each_ms>0 && ((ms-copter_last_send_rc_ms) >= copter_each_ms) ){
    if( is_use_wifi_connect() == 1 )
      ids |= WIFI_ID;
    if( is_use_telem_connect() == 1 )
      ids |= TELEM_ID;
    copter_last_send_rc_ms = ms;
  }  
  if( ids != 0 ) mega2560_send_rc(ids);
}


void mega2560_report_status_to_gcs(int each_ms)
{
  int ret = 0;
  mavlink_rc_channels_override_t sp;
  mavlink_message_t message;
  unsigned long ms = millis();
  
  if( (ms-last_report_gcs_ms)< each_ms )
    return;
  else
    last_report_gcs_ms = ms;
    
// fill with the sp 
  sp.chan1_raw = MEGA2560_SYS_ID; // my package tag
  sp.chan2_raw = GCS_CMD_REPORT_STATUS; // the connecting way
  sp.chan3_raw = connectStatus; // the connecting way
  sp.chan4_raw = isActivityPath[GCS_ID]<<GCS_ID | (isActivityPath[WIFI_ID]<<WIFI_ID) | (isActivityPath[TELEM_ID]<<TELEM_ID);//each way 's status
  //sp.chan4_raw = get_rc(YAW_ID);
  //sp.chan5_raw = get_rc(CHAN_5_ID);
  //sp.chan6_raw = get_rc(CHAN_6_ID);
  //sp.chan7_raw = get_rc(CHAN_7_ID);
  //sp.chan8_raw = get_rc(CHAN_8_ID);
  sp.target_component = 1;
  sp.target_system = 1;
  
  mavlink_msg_rc_channels_override_encode(MAVLINK_SYSID, MAVLINK_COMPID ,&message, &sp);
  mega2560_send_mavlink(1<<GCS_CONFIG_ID,&message);
}

void mega2560_update_status()
{
  unsigned long ms = millis();
  int i;
  for( i = 0; i< 3; i++){
    if( (ms - lastPackageTime[i]) > 5000 ){//5s
      switch_led(i,0); 
      isActivityPath[i] = 0;
    }else{
      isActivityPath[i] = 1;
    }
  }
  mega2560_report_status_to_gcs(REPORT_STATUS_MS);
}


//########################################## i2c

#include <Wire.h>
#define I2C_BUFFER_LEN 1024
uint8_t i2cBuffer[I2C_BUFFER_LEN];
void send_msg_to_lcd(uint8_t *buff,int len)
{
   Wire.beginTransmission(8); // transmit to device #8
   for( int i=0 ; i< len; i++){
     Wire.write(buff[i]);
   }
  Wire.endTransmission();    // stop transmitting
  delay(100);
}
void listen_i2c_lcd(uint8_t *buff,int len)
{
   Wire.requestFrom(8, len);    // request 6 bytes from slave device #2
  int d_len = Wire.available();   // slave may send less than requested
  d_len = d_len>len? len:d_len;
  if( d_len > 0 ){
    Wire.readBytes(buff,d_len);
    
    buff[d_len]=0;
    Serial.println((char*)buff);
  } 
}

//***************************************************** main function

void setup() {
  mega2560_uart_setup();
  setup_chan_pin_type();
  setup_key_pin_mode();
  setup_led_pin_mode();
  //update_rc_trim_value();
  
  Wire.begin();
}

void loop() {
  //for( int i=0 ; i < 8 ; i++)
   //debug_rc_channel_val(2);
   //debug_rc_channel_val(3);

  
  mega2560_uart_loop();
  mega2560_update_status();
  update_rc_loop(10);//echo 10ms update update
  //update_key_loop();
  mega2560_send_rc_loop( COPTER_SEND_RC_MS , GCS_SEND_RC_MS );//copter send each 20ms, gcs 50ms 
  //send_msg_to_lcd((uint8_t*)"mega2560",8);
  
  /*
  mega2560wrt_listen_gcs();
  mega2560wrt_listen_wifi();
  mega2560wrt_update_status();

  update_rc_loop(10);//alway update
  mega2560wrt_send_rc_loop(20);//echo 20 ms send a package
  //update_key_loop();
  */
  
  //delay(DELAY_TIME);
  

}


