#include <mavlink.h>
#include <ESP8266WiFi.h>

enum wifi_errors {
    AP_CONNECT_ERROR =     0,  
    TCP_CONNECT_ERROR =     1,
    UART_ERROR = 2   
};

#define DEBUG_ENABLE 1
inline void debugMsg(int msg)
{
  if( DEBUG_ENABLE==1 )
    Serial1.println(msg);
}
inline void debugMsg(char* msg)
{
  if( DEBUG_ENABLE == 1 )
    Serial1.println(msg);
}

//############################################## uart data
int call_connect_status = 0; // 0 no call connect, 1 call connect
#define MAX_BUFFER_SIZE 2048
uint8_t buffer[MAX_BUFFER_SIZE];
#define gcsSerial Serial
#define GCS_COM MAVLINK_COMM_0

//sync with tower ,esp8266 and mega2560{
#define MEGA2560_SYS_ID 254
enum _GCS_CMDS { 
    MEGA2560_BOARD_CMD_WIFI_CONNECT_TCP =  1,
    MEGA2560_BOARD_CMD_WIFI_SET_CONNECT_IP,
    MEGA2560_BOARD_CMD_WIFI_DISCONNECT_TCP,
    MEGA2560_BOARD_CMD_WIFI_MAX_ID,
    
    //MEGA2560_BOARD_CMD_CHANGE_PATH
    //....
    MEGA2560_BOARD_CMD_MAX_ID
};
struct param_ip_data{
    uint8_t ip[4];
    uint8_t port[2] ;
};
//}


//############################################  AP data 
unsigned long connect_ap_ms = 0;
 char* ssid = "RC_box";
//char* ssid = "car1";
 char* password = "88888888";
boolean has_start_connect_ap = false;



//################################################### tcp connect
WiFiClient client;
int mIp[4] = {192,168,1,100};
int mPort = 6666;
unsigned long connect_tcp_ms = 0;
boolean has_start_connect_tcp = false;











//############################################## uart func

void uart_handle_mavlink(mavlink_message_t *msg)
{
  mavlink_param_set_t packet;
  mavlink_message_t message;
  uint8_t type ;
  struct param_ip_data *data;

  
/*  if( msg->msgid == MAVLINK_MSG_ID_PARAM_SET ){
    mavlink_msg_param_set_decode(msg,&packet);
    if( packet.param_type == MEGA2560_WIFI_BOARD_CMD_CONNECT_TCP ){
      debugMsg("call wifi connect tcp");
      call_connect_status = 1;
    }else if( packet.param_type == MEGA2560_WIFI_BOARD_CMD_SET_CONNECT_IP){
      data = (struct param_ip_data *)packet.param_id;
      
      debugMsg("get ip from gcs :");
      debugMsg(data->port);debugMsg(data->ip[0]);debugMsg(data->ip[1]);debugMsg(data->ip[2]);debugMsg(data->ip[3]);
      
      for( int i=0 ;i <4 ;i++) mIp[i]= data->ip[i];
      mPort=data->port;
      
    }else if( packet.param_type == MEGA2560_WIFI_BOARD_CMD_DISCONNECT_TCP){
      debugMsg("call wifi disconnect tcp");
      call_connect_status = 0;
    }
  }*/
  mavlink_msg_param_set_decode(msg,&packet);
  if( packet.param_value == MEGA2560_BOARD_CMD_WIFI_CONNECT_TCP ){
      debugMsg("call wifi connect tcp");
      call_connect_status = 1;
  }else if( packet.param_value == MEGA2560_BOARD_CMD_WIFI_SET_CONNECT_IP){
      data = (struct param_ip_data *)packet.param_id;
      for( int i=0 ;i <4 ;i++) mIp[i]= data->ip[i];
      mPort=data->port[0]|data->port[1]<<8;
      debugMsg("get ip from gcs :");
      debugMsg(mPort);debugMsg(mIp[0]);debugMsg(mIp[1]);debugMsg(mIp[2]);debugMsg(mIp[3]);
      check_and_disconnect_tcp();
  }else if( packet.param_value == MEGA2560_BOARD_CMD_WIFI_DISCONNECT_TCP){
      debugMsg("call wifi disconnect tcp");
      call_connect_status = 0;
  }
  
}
void uart_listen_gcs(){
    size_t len;
    int i;
    mavlink_message_t msg; 
    mavlink_status_t status;
    uint16_t p_len ;
    uint8_t mavlinkBuffer[MAVLINK_MAX_PACKET_LEN];
    
    len = gcsSerial.available();
    len = len>MAX_BUFFER_SIZE ? MAX_BUFFER_SIZE:len;
    if( len > 0){
       gcsSerial.readBytes(buffer,len);
    
      for( i=0; i< len; i++){
        if( mavlink_parse_char(GCS_COM,  buffer[i] , &msg, &status) ) { 
          if( msg.msgid == MAVLINK_MSG_ID_PARAM_SET  && mavlink_msg_param_set_get_param_type(&msg)== MEGA2560_SYS_ID ){
            uart_handle_mavlink(&msg);
          }else{
            p_len = mavlink_msg_to_send_buffer(mavlinkBuffer,&msg);
            //debugMsg("write mavlinik to tcp"); 
            tcp_write(mavlinkBuffer,p_len);
          }
        }
      }
  }
}
//############################################  AP connect 
void startConnectAp(char * sid, char *pwd)
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(sid,pwd);
  has_start_connect_ap = true;
  connect_ap_ms = millis();
}
//WiFi.localIP()
//WiFi.printDiag(Serial) will print out some diagnostic info
boolean check_and_reconnect_ap(char * sid, char *pwd)
{
    if( WiFi.status() == WL_CONNECTED )
      return true;
    unsigned long now_ms = millis();
    if( !has_start_connect_ap || (now_ms - connect_ap_ms) > 10000){
       startConnectAp( sid,pwd );
    }
    return false;
}

//################################################### tcp connect
void startConnectTcp(int *ip,int port)
{
  IPAddress addr (ip[0],ip[1],ip[2],ip[3]);
  client.setNoDelay(true);
  client.connect(addr,port);
  connect_tcp_ms = millis();
  has_start_connect_tcp = true;
}
void check_and_disconnect_tcp()
{
  if( has_start_connect_tcp ) //client.connected() )
    client.stop();
  has_start_connect_tcp = false;
}
int tcp_read(uint8_t *buff,int max_len)
{
  size_t len=0;
  int ret;
  
  if( !client.connected() )
    return 0;
  /*
  len = client.available();
  len = len>MAX_BUFFER_SIZE? MAX_BUFFER_SIZE:len;
  if( len > 0 ){
   */
  len = client.read(buff,max_len);
  /*
  if( len > 0 ){
      Serial.print("tcp len=");
    Serial.print(len);
    Serial.print(", data:");
    buff[len]=0;
    Serial.println((char*)buff);
  }
*/
  return len;
}
void tcp_write(uint8_t *buff, size_t len)
{
  if( !client.connected() )
    return ;
    //debugMsg("write to tcp"); 
  client.write((const unsigned char*)buff,len);
  //for( int i=0 ; i< len; i++)
    //client.write(buff[i]);
}
boolean check_and_reconnect_tcp(int *ip,int port)
{
  if( client.connected())
    return true;
    
  unsigned long now_ms = millis();
  if( !has_start_connect_tcp || (now_ms-connect_tcp_ms) > 3000){
    client.stop();
    debugMsg("start reconnect tcp...");
    startConnectTcp(mIp,mPort);
  }
  return false;
}





//####################################################### main function
void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);
  Serial.begin(115200);
  startConnectAp( ssid,password );
}

void loop() {
  boolean ap_ret, tcp_ret;
  int len,ret;
 
  ap_ret = check_and_reconnect_ap(ssid,password);
  
  if( ap_ret ){
   if(call_connect_status ==1){
      tcp_ret = check_and_reconnect_tcp(mIp,mPort);
      if( tcp_ret ){
        len = tcp_read(buffer,MAX_BUFFER_SIZE);
        
        if( len > 0 ){
          debugMsg("get tcp data");
          gcsSerial.write(buffer,len);
        }
      }
   }else{
      check_and_disconnect_tcp();
   }
  }else{
      check_and_disconnect_tcp();
  }
  
  uart_listen_gcs();
  delay(10);
  
}










