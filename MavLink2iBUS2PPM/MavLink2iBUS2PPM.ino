// iBus2PPM v2 version 1.01
// Arduino Nano/Pro code to read FlySky iBus and output
// x channels of CPPM
// BaseFlight supports iBus, CleanFlight got it, and even betaflight.
// But since iBus is 115k2 serial, it requires an UART, sacrificing the ability to
// use GPS on the NAZE32.
// Another use for this is as a link to a 433MHz radio Tx
//          FlySky i6 (with 10 channel patch) -> iA6B -> iBus2PPM -> 433MHz LRS
// Nobody wants to use trainer cable if they can run wireless.

// I use Turnigy TGY-i6 aka Flysky FS-i6 transmitter and iA6B Rx. I am running hacked firmware from
// this thread to get 10 channels over iBus: http://www.rcgroups.com/forums/showthread.php?t=2486545
// the i6 has the 4 channels for the sticks, 4 switches, and 2 analog potentiometers. So can generate
// 10 independent channels.
// Latest hacked firmware allow to combine 2 switches to one 4 or 6 channel switch, which I use for flight modes,
// thus I get only 9 channels (could make channel 10 a derived channel).
// As a result, I chose to send  9 channels over PPM
// Unfortunately, there is no way to input high channels in trainer mode yet. Would be nice for head-tracker.
#include <PulsePosition.h>
PulsePositionOutput ppmOutput;
#define mavlinkSerial Serial1
#define ibusSerial Serial2

//SoftwareSerial mavlinkSerial(8, 9); // RX, TX
#include <GCS_MAVLink.h>
#include <string.h>
// NEW PPM CONFIGURATION
#define TX_DEFAULT_SIGNAL 1500.0
#define RX_MINIMUM_SPACE 3500.0
#define IBUS_MAX_CHANNEL 10
#define IBUS_BUFFSIZE 32
#define IBUS_SYNCBYTE 0x20
#define IBUS_BAUDRATE 115200
#define MAVLINK_BAUDRATE 57600
#define PPM_PIN 6
#define LED_PIN 11


#define PPM_CHANS 9   // The number of iBus channels to send as PPM. 14 is supported. 10 from FS-i6
           // No reason to send more than the FC will need.
          // If going above 10 channels, you need to add more channels to the unrolled loop in readRX()

// If you want to use PWM, you can use channel 5+6 on Rx directly. Then swap with other channels in PPM output
// To use channel 9 as PPM channel 5, det SWAPCH5 to 9. Channel 5 will be on channel 9 in PPM stream
#define SWAPCH5 5   // channel 9 mapped to channel 5 and the other way around
#define SWAPCH6 6  // channel 10 mapped to channel 6 and t1222he other way around

#define IBUS_MAXCHANNELS 14
// Failsafe values for the channels. FS-i6 does not support throttle failsafe < 1000, so we must fake it
// Setup failsafe on the FS-i6 to have all 6 channels to to -100 or thereabout to allow us to reliable detect it.
// This array must be minimum PPM_CHANS long
// In the example, below, I set channel 3 to 950 (Throttle) and channel 5 to 2000 (flight mode). Then just
// ensure RTH is on highest value on the flightmode switch, so even if FC does not see failsafe, it will enter RTH mode.
// We do no swapping in this array, so it goes directly to PPM
static uint16_t rcFailsafe[IBUS_MAXCHANNELS] = {  1500, 1500, 950, 1500, 2000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };
#define FAILSAFELIMIT 1020    // When all the 6 channels below this value assume failsafe


//////////////////////PPM CONFIGURATION///////////////////////////////
///// PPM_FrLen might be lowered a bit for higher refresh rates, as all channels
///// will rarely be at max at the same time. For 8 channel normal is 22.5ms.
//#define default_servo_value 1500  //set the default servo value 1
#define PPM_PulseLen 400  //set the pulse length
//#define PPM_Pause 3500    // Pause between PPM frames in microseconds (1ms = 1000µs) - Standard is 6500 1
#define PPM_FrLen (((1700+PPM_PulseLen) * PPM_CHANS)  + PPM_Pause)  //set the PPM frame length in microseconds 
    // PPM_FrLen can be adjusted down for faster refresh. Must be tested with PPM consumer (Flight Controller)
    // PPM_VariableFrames uses variable frame length. I.e. after writing channels, wait for PPM_Pause, and then next packet.
    // Would work as long as PPM consumer uses level shift as trigger, rather than timer (standard today).
    // 8 channels could go from 22500 us to an average of 1500 (center) * 8 + 3500 = 15500 us. That is
    // cutting 1/3rd off the latency.
    // For fastest response, make sure as many values as possible are low. I.e. fast response flight mode has lower value.
    // Make sure unused channels are assigned a switch set to value 1000. Or configure to fewer channels PPM .
#define PPM_VariableFrames 0  // Experimental. Cut down PPM latency. Should work on most Flight Controllers using edge trigger.
#define PPM_offset -7 // How much are the channels offset  ? Compensate for timer difference, CPU spent elsewhere
            // Use this to ensure center is 1500. Then use end-point adjustments on Tx to hit endpoints.
#define onState 0  //set polarity: 1 is positive, 0 is negative
#define sigPin 2  //set PPM signal                                                    digital pin on the arduino
//////////////////////////////////////////////////////////////////

#define IBUS_BUFFSIZE 32    // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
                                                                                                                                     
static uint16_t rcValue[IBUS_MAXCHANNELS];
static uint16_t rcValueSafe[IBUS_MAXCHANNELS]; // read by interrupt handler. Data copied here in cli/sei block
static boolean rxFrameDone;
static boolean failsafe = 0;
uint16_t dummy;
#define START    1
#define MSG_RATE    10 // Hertz

#define SYSID 200
#define COMPID 190  //190 MissionPlanner
#define TARSYS 1
#define TARCOMP 0 //0 all
// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0 3.1 3.2
#define LED_PIN 13

// Message #0  HEARTHBEAT 
uint8_t  ap_type = 0, ap_autopilot = 0, ap_base_mode = 0, ap_system_status = 0, ap_mavlink_version = 0;
uint32_t  ap_custom_mode = 0;

// Message # 1  SYS_STATUS 
uint16_t  ap_voltage_battery = 0;  //mV
int16_t  ap_current_battery = 0;  //dA

// Message #24  GPS_RAW_INT 
uint8_t  ap_sat_visible = 0, ap_fixtype = 1;  //0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
int32_t  ap_latitude = 0, ap_longitude = 0, ap_gps_altitude = 0;

// Message #74 VFR_HUD 
int32_t    ap_airspeed = 0;
uint32_t  ap_groundspeed = 0, ap_heading = 0;
uint16_t  ap_throttle = 0;
int32_t    ap_bar_altitude = 0, ap_climb_rate=0;
uint16_t load = 0;
// Message #27 RAW IMU 
int32_t   ap_accX = 0, ap_accY = 0, ap_accZ = 0;

uint8_t     MavLink_Connected, buf[MAVLINK_MAX_PACKET_LEN];
uint16_t  hb_count, len;
unsigned long MavLink_Connected_timer, hb_timer, acc_timer, deb_timer;

mavlink_message_t msg;
// Prototypes
void setupRx();
void readRx();
void hb_control();
void mavlink_receive();

void setup() {
//  setupRx();
//  setupPpm();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Checksum error - turn on error LED
  Serial.println("Init complete");
  mavlinkSerial.begin(MAVLINK_BAUDRATE);
//  ibusSerial.begin(57600);

}

void loop() {
    hb_control();
    mavlink_receive(); 
//  readRx();  
  // PPM output is run as timer events, so all we do here is read input, populating global array
}

static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};

void setupRx()
{
  uint8_t i;
  for (i = 0; i < PPM_CHANS; i++) { rcValue[i] = 1127; }
  ibusSerial.begin(IBUS_BAUDRATE);
}

void readRx()
{
  uint8_t i;
  uint16_t chksum, rxsum;

  rxFrameDone = false;

  uint8_t avail = ibusSerial.available();
  
  if (avail)
  {
    digitalWrite(4, LOW);
    uint8_t val = ibusSerial.read();
    // Look for 0x2040 as start of packet
    if (ibusIndex == 0 && val != 0x20) {
      return;
    }
    if (ibusIndex == 1 && val != 0x40) {
      ibusIndex = 0;
      return;
    }
 
    if (ibusIndex < IBUS_BUFFSIZE) ibus[ibusIndex] = val;
    ibusIndex++;

    if (ibusIndex == IBUS_BUFFSIZE)
    {
      ibusIndex = 0;
      chksum = 0xFFFF;
      for (i = 0; i < 30; i++)
        chksum -= ibus[i];

      rxsum = ibus[30] + (ibus[31] << 8);
      if (chksum == rxsum)
      {
        //Unrolled loop  for 10 channels - no need to copy more than needed.
        // MODIFY IF MORE CHANNELS NEEDED
        rcValue[0] = (ibus[ 3] << 8) + ibus[ 2];
        rcValue[1] = (ibus[ 5] << 8) + ibus[ 4];
        rcValue[2] = (ibus[ 7] << 8) + ibus[ 6];
        rcValue[3] = (ibus[ 9] << 8) + ibus[ 8];
        rcValue[4] = (ibus[11] << 8) + ibus[10];
        rcValue[5] = (ibus[13] << 8) + ibus[12];
        rcValue[6] = (ibus[15] << 8) + ibus[14];
        rcValue[7] = (ibus[17] << 8) + ibus[16];
        rcValue[8] = (ibus[19] << 8) + ibus[18];
        rcValue[9] = (ibus[21] << 8) + ibus[20];
        rxFrameDone = true;
        if (rcValue[0] < FAILSAFELIMIT && rcValue[1] < FAILSAFELIMIT &&
            rcValue[2] < FAILSAFELIMIT && rcValue[3] < FAILSAFELIMIT &&
            rcValue[4] < FAILSAFELIMIT && rcValue[5] < FAILSAFELIMIT ) 
        {
          failsafe = 1;
          cli(); // disable interrupts
          memcpy(rcValueSafe, rcFailsafe, IBUS_MAXCHANNELS * sizeof(uint16_t));
          sei();
          digitalWrite(LED_PIN, HIGH);  //  Error - turn on error LED
        }
        else
        {
          // Now we need to disable interrupts to copy 16-bit values atomicly
          // Only copy needed signals (10 channels default)
          // MODIFY IF MORE CHANNELS NEEDED
          cli(); // disable interrupts.
          rcValueSafe[0] = rcValue[0];
          rcValueSafe[1] = rcValue[1];
          rcValueSafe[2] = rcValue[2];
          rcValueSafe[3] = rcValue[3];
          rcValueSafe[4] = rcValue[SWAPCH5-1];
          rcValueSafe[5] = rcValue[SWAPCH6-1];
          rcValueSafe[6] = rcValue[6];
          rcValueSafe[7] = rcValue[7];
          rcValueSafe[8] = rcValue[8];
          rcValueSafe[9] = rcValue[9];
#if (SWAPCH5 != 5)
          rcValueSafe[SWAPCH5-1] = rcValue[4];
#endif
#if (SWAPCH6 != 6)
          rcValueSafe[SWAPCH6-1] = rcValue[5];
#endif          
          sei();
          digitalWrite(LED_PIN, LOW); // OK packet - Clear error LED
        }
      } else {
        digitalWrite(LED_PIN, HIGH);  // Checksum error - turn on error LED
      }
      return;
    }
  }
}

static byte cur_chan_numb;


// PPM sum is a signal consisting of a pulse of PPM_PulseLen, followed by a non-pulse 
// defining the signal value. So a 300us PPM-pulse and 1200us pause means the value 
// of the channel is 1500. You might get away with pulses down to 100us depending on
// receiving end.
// After the channels, a long channel is sent to fill out the frame.
// Normal frame length is 22500 us.
// 1 channel at full length is 2000us. 10 channels = 20000 us
// So even at 10 full channels, we have 2500us left for pause.
//
// Most Flight Controller trigger on level shift rather than timing.
// They measure from pulse start to pulse start, so PPM_PulseLen not critical
// If you want more channels, increase the framelen, to say 12*2000 + 3000 = 27000.


 


void mavlink_receive()  //to run when you want to receive updated informations
{ 
  mavlink_message_t msg;
  mavlink_status_t status;
  while(mavlinkSerial.available()) 
  { 
//Serial.println("voll da");
uint8_t c = mavlinkSerial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) 
    {
      switch(msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT:  // 0
        Serial.println("HEARTBEAT");
          ap_base_mode = (mavlink_msg_heartbeat_get_base_mode(&msg) & 0x80) > 7;
          ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
          MavLink_Connected_timer=millis(); 
          if(!MavLink_Connected);
          {
            hb_count++;   
            Serial.println(hb_count);
            if((hb_count++) > 10)
            {        // If  received > 10 heartbeats from MavLink then we are connected
              MavLink_Connected=1;
              hb_count=0;
              digitalWrite(LED_PIN,HIGH);      // LED_PIN will be ON when connected to MavLink, else it will slowly blink
            }
          }
          break;
        
        case MAVLINK_MSG_ID_SYS_STATUS :
        Serial.println("STATUS");
        // 1
          ap_voltage_battery = mavlink_msg_sys_status_get_voltage_battery(&msg);  // 1 = 1mV
          load = mavlink_msg_sys_status_get_load(&msg);
          //ap_current_battery = Get_Current_Average(mavlink_msg_sys_status_get_current_battery(&msg));     // 1=10mA
         Serial.print("voltage : ");
         Serial.println(ap_voltage_battery);
         Serial.print("load : ");
         Serial.println(load);
          break;
          
        case MAVLINK_MSG_ID_GPS_RAW_INT:   // 24
          ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                               // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
          ap_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);          // numbers of visible satelites
          if(ap_fixtype == 3)
          {
            ap_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
            ap_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
            ap_gps_altitude = mavlink_msg_gps_raw_int_get_alt(&msg);    // 1m =1000
          }
          break;
        
        case MAVLINK_MSG_ID_RAW_IMU:   // 27
          ap_accX = mavlink_msg_raw_imu_get_xacc(&msg) / 10;                // 
          ap_accY = mavlink_msg_raw_imu_get_yacc(&msg) / 10;
          ap_accZ = mavlink_msg_raw_imu_get_zacc(&msg) / 10;
          //Serial.print(ap_accX);
          break;
        
        case MAVLINK_MSG_ID_VFR_HUD:   //  74
          ap_airspeed = 0;
          ap_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);      // 100 = 1m/s
          ap_heading = mavlink_msg_vfr_hud_get_heading(&msg);     // 100 = 100 deg
          ap_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);        //  100 = 100%
          ap_bar_altitude = mavlink_msg_vfr_hud_get_alt(&msg) * 100;        //  m
          ap_climb_rate=mavlink_msg_vfr_hud_get_climb(&msg) * 100;        //  m/s
          break;
        
        default:
          break;
      }
    }
  }
}

void hb_control()   //To run every loop
{
  
  if(millis()-hb_timer > 1500)
  {
    hb_timer=millis();
    if(!MavLink_Connected)  // Start requesting data streams from MavLink
    {
      Serial.println("hb_control");
      digitalWrite(LED_PIN,HIGH);
      
      mavlink_msg_request_data_stream_pack(SYSID, COMPID, &msg, TARSYS, TARCOMP, MAV_DATA_STREAM_EXTENDED_STATUS, MSG_RATE, START);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      mavlinkSerial.write(buf,len);
      
      delay(10);
      
      mavlink_msg_request_data_stream_pack(SYSID, COMPID, &msg, TARSYS, TARCOMP, MAV_DATA_STREAM_EXTRA2, MSG_RATE, START);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      mavlinkSerial.write(buf,len);
      
      delay(10);
      
      mavlink_msg_request_data_stream_pack(SYSID, COMPID, &msg, TARSYS, TARCOMP ,MAV_DATA_STREAM_RAW_SENSORS, MSG_RATE, START);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      mavlinkSerial.write(buf,len);
      
      digitalWrite(LED_PIN,LOW);
    }
  }
  
  if((millis() - MavLink_Connected_timer) > 1500)
  {
    MavLink_Connected=0;
    hb_count = 0;
  } 
}

