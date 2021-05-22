#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <vector>
#include <NTPClient.h>
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

using namespace std;

#define SIZE_SERIAL_QUEUE 20
#define SIZE_NTP_QUEUE 1
#define TICKS_WAIT_SERIAL           ( TickType_t )1000

#define LED_BUILTIN 2

const char* mqtt_server = "broker.hivemq.com"; //mqtt server 
IPAddress ip_rasp = IPAddress( (uint8_t) 192, (uint8_t) 168,(uint8_t) 0,(uint8_t) 15);
const char* ssid = "brisa-1681333";
const char* password = "1twjuolc";



QueueHandle_t xQueue_Serial;
QueueHandle_t xQueue_Ntp;

/* NTP client variables*/
WiFiUDP udp;
NTPClient ntp(udp, "a.st1.ntp.br", -3 * 3600, 60000);//Cria um objeto "NTP" com as configurações.utilizada no Brasil 
String hora;            // Variável que armazena

/* Task functions prototypes*/
void task_wifi( void *pvParameters );
void task_mqtt( void *pvParameters );
void task_serial( void *pvParameters );

/*Utilities functions prototype*/
void mqtt_reconnect();

WiFiClient espClient;
PubSubClient client(espClient); //lib required for mqtt

void setup()
{
    Serial.begin(9600);
    delay(200);

    pinMode(LED_BUILTIN,OUTPUT);
    // digitalWrite(LED_BUILTIN,HIGH);
    // delay(2000);
    // digitalWrite(LED_BUILTIN,LOW);
    // delay(2000);
     WiFi.mode(WIFI_STA);


    xQueue_Serial = xQueueCreate(SIZE_SERIAL_QUEUE, sizeof(char[20]));
    xQueue_Ntp = xQueueCreate(SIZE_NTP_QUEUE, sizeof(char[11]));

    if (xQueue_Serial == NULL)
    {
        Serial.println("Failed on initializing queue, esp restarting");
        delay(2000);
        ESP.restart();
    }

    Serial.println("Setup finished");

    /* Configuração das tarefas */
    xTaskCreate(
    task_wifi                   
    , "wifi_control"                    
    ,  4096                             
    ,  NULL                             
    ,  6                               
    ,  NULL );                         
 
    xTaskCreate(
    task_mqtt             
    , "mqtt_control"        
    ,  4096    
    ,  NULL                      
    ,  6
    ,  NULL );   

    xTaskCreate(
    task_serial             
    , "serial_control"        
    ,  4096    
    ,  NULL                      
    ,  5
    ,  NULL );

    xTaskCreate(
    task_ntp_client             
    , "task_ntp_client"        
    ,  4096    
    ,  NULL                      
    ,  6
    ,  NULL );    
   
    /* O FreeRTOS está inicializado */
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Serial.print("Message arrived [");
  // Serial.print(topic);
  // Serial.print("] ");
  
  // for (int i = 0; i < length; i++)
  // {
  //   Serial.print((char)payload[i]);
  // }
  
  // Serial.println();
}

void loop()
{

}

void task_wifi( void *pvParameters )
{
  
  wl_status_t WiFi_status_prev = WL_DISCONNECTED;
  char msg_serial[20];
  
  while(1){
    
    if (WiFi.status() == WL_CONNECTED) {
     

      strcmp(msg_serial, "Wifi connected\0");
      // sprintf(msg_serial, "Connected to %s with IP: %d.%d.%d.%d\n",ssid,WiFi.localIP()[0],
      //                                                                 WiFi.localIP()[1],          
      //                                                                 WiFi.localIP()[2],
      //                                                                 WiFi.localIP()[3]                    
      //                                                                 );
      digitalWrite(LED_BUILTIN, HIGH);
      xQueueSend(xQueue_Serial, ( void * ) &msg_serial, TICKS_WAIT_SERIAL );
      
      digitalWrite(LED_BUILTIN,HIGH);
      vTaskDelay( 500 / portTICK_PERIOD_MS );
      digitalWrite(LED_BUILTIN, LOW);
      vTaskDelay( 500 / portTICK_PERIOD_MS );
    }
    
    else{
    
    
      WiFi.begin(ssid, password);
     
      while (WiFi.status() != WL_CONNECTED) 
      {        
        vTaskDelay( 100 / portTICK_PERIOD_MS );
        strcmp(msg_serial, "Wifi connecting\0");
        xQueueSend(xQueue_Serial, ( void * ) &msg_serial, TICKS_WAIT_SERIAL );
  
      }

    } 
  
    WiFi_status_prev = WiFi.status();
  }
  
}

void task_mqtt( void *pvParameters )
{
  char msg_serial[11] = "test mqtt\n";
  char msg_mqtt[20];
  char time[11];
  
  client.setServer(ip_rasp, 1883);//connecting to mqtt server
  client.setCallback(callback);
  client.connect("ESP32_Freitas192");
  vTaskDelay( 300 / portTICK_PERIOD_MS );
  
  while(1){
    if(WiFi.status() == WL_CONNECTED){
      
      while (1){
        if (!client.connected()){
          mqtt_reconnect();
        }
        else{
          //Consume queue containing messages and topics
          if( xQueuePeek( xQueue_Ntp, &(time ), portMAX_DELAY) )
          {

            //sprintf(msg_mqtt,"%s esp32 visual code Joao pessoa connected",time);
             //client.publish("freitasTopic", time);

             strcpy(msg_serial, time ); 
             client.publish("freitasTopic", msg_serial);
          
             xQueueSend(xQueue_Serial, ( void * ) &msg_serial, TICKS_WAIT_SERIAL );  
          }
          
          //sprintf(msg_serial, "Connected to Hive MQTT broker\n");
          

          vTaskDelay( 1000 / portTICK_PERIOD_MS );

        }
        
        client.loop(); 

    }

  }
    else{
    //Means internet is not connected, so wait for it to work with mqtt
    }

  }
   
}

void mqtt_reconnect() {
   char msg_serial[30] = "trying to connect\n";
  
  while (!client.connected()) {

    //sprintf(msg_serial, "Attempting MQTT connection...\n\0");
    xQueueSend(xQueue_Serial, ( void * ) &msg_serial, TICKS_WAIT_SERIAL );
    
    if (client.connect("ESP32_Freitas192")) {
      strcpy(msg_serial,"Client Connected");
      xQueueSend(xQueue_Serial, ( void * ) &msg_serial, TICKS_WAIT_SERIAL );
      //Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic", "Nodemcu connected to MQTT");
      // ... and resubscribe
      //client.subscribe("inTopic");

    } else {
      //Serial.print("failed, rc=");
      //Serial.print(client.state());
      //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay( 5000 / portTICK_PERIOD_MS );
    }
  }
}

void task_serial( void *pvParameters )
{
   char msg_serial[30];
   
   while(1){
     if( xQueueReceive( xQueue_Serial, &( msg_serial ), TICKS_WAIT_SERIAL) ){
       //Serial.println("Received Message from serial");
       //Serial.println("%s", hora.c_str());
      
       vTaskDelay( 1000 / portTICK_PERIOD_MS );

   }

   }
   
   
}

void task_ntp_client( void *pvParameters )
{
  char msg_serial[11];
  Serial.println("NTP begun");
  while(1){
    
    if (WiFi.status() == WL_CONNECTED) {
      
      ntp.begin();            
      Serial.println("NTP begun");
      while(1){
        
        if (WiFi.status() == WL_CONNECTED) {
          ntp.update();
          hora = ntp.getFormattedTime(); 
          Serial.print("Hora: ");
          Serial.println(hora);
          strcpy(msg_serial,hora.c_str());
          //xQueueSend(xQueue_Serial, ( void * ) &msg_serial, TICKS_WAIT_SERIAL ); 
          xQueueOverwrite(xQueue_Ntp, &msg_serial);
          vTaskDelay( 1000 / portTICK_PERIOD_MS );
        }
        else{
          break;
        }

      }
    
    }
    vTaskDelay( 100/ portTICK_PERIOD_MS );
    
  }
  
}

