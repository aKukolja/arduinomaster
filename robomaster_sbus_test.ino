#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010

#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 16 //ms 11 for futaba, unstable, 20 works great

/* Channel values range from 1000 to 2000 where 1500 is reference 0. Note: this may not be correct
 * channel        gimbal+chassis         free(gimbal is not controlable)
 * 0              ? < 1500 < ?
 * 1              back < 1500 < forward   same
 * 2              up < 1500 < down        nothing
 * 3              rot.l < 1500 < rot.r    same but chassis
 * 4              chassis speed: slow(-100) < 1500 (medium) < fast(+100)
 * 5              free < 1500 <gimbal+chassis
 * 6              free chassis < 1500 < control chassis
 * 
 * This example has been tested with arduino mkr1000.
 * The serial (Serial1) port on pin 14 has been configured to transmit using sbus protocol.
 * Configuration and code have been taken from:
 *  https://quadmeup.com/generate-s-bus-with-arduino-in-a-simple-way/
 * Because sbus requires the signal to be inverted the output of pin 14 is connected to
 * SN54HCT04N. This inverted output is connected to robomaster sbus connector as explained here:
 *  https://forum.dji.com/forum.php?mod=viewthread&tid=204929&extra=page%3D1%26filter%3Dtypeid%26typeid%3D704%26typeid%3D704
 * The SBUS_UPDATE_RATE constant determines the time between two packets sent on the bus.
 * The value of 11ms is used in the Futaba implementation of the protocol (DJI forum).
 * This value caused robomaster to become unresponsive at random and after this brief
 * period it would again start executing commands sent over sbus.
 * A higer value of 16ms was chosen because it greatly reduces the chance of robomaster
 * becoming unresponsive. Robomaster ignores all packets for any value < 11ms.
 * 
 */

void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe){

    static int output[SBUS_CHANNEL_NUMBER] = {0};

    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
        //output[i] = channels[i];
    }

    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }
    packet[0] = SBUS_FRAME_HEADER; //Header

    packet[1] = (uint8_t) (output[0] & 0x07FF);
    packet[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);

    packet[23] = stateByte; //Flags byte
    packet[24] = SBUS_FRAME_FOOTER; //Footer
}

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 5 * 1000;
uint8_t echo[SBUS_CHANNEL_NUMBER];

void go_forward() {
  for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
      //rcChannels[i] = i < 8 ? 1000 : 1500;
      rcChannels[i] = 1500;
  }
  rcChannels[1] = 1700; // moves the chassis forward
  rcChannels[4] = 1000; // slow your roll
  rcChannels[5] = 1000; // gimbal + chassis
  //magic that makes it all work
  rcChannels[6] = 2000; // control chassis
}

void setup() {
    Serial1.begin(100000, SERIAL_8E2);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    go_forward();
}

void flush_input() {
  while(Serial1.available()) Serial.read();
}

int read_echo() {
  int num_read = 0;
  while(num_read<SBUS_PACKET_LENGTH) {
    int b;
    if (Serial1.available() <= 0) break;
    b = Serial1.read();
    if (b < 0) break;
    echo[num_read++] = (uint8_t) b;
  }
  flush_input();
  return num_read;
}

#define BUMP_TIME 2000 //ms
#define BUMP_STEP 200

uint32_t last_millis = 0;

int pitch_step = BUMP_STEP;
void up_down(int *channel) {
  uint32_t now = millis();
  if(now-last_millis > BUMP_TIME) {
    *channel += pitch_step;
    last_millis = now;
  }
  if(*channel > 1700 || *channel < 1300) {
    pitch_step = -pitch_step;
    *channel += 2* pitch_step;
  }
  /*
  if(pitch_step - BUMP_STEP) digitalWrite(LED_BUILTIN, HIGH);
  else digitalWrite(LED_BUILTIN, LOW);
  */
}

uint32_t last_sent = 0;

void loop() {
    uint32_t currentMillis = millis();
    
    if (currentMillis >= sbusTime) {
        Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);
        sbusPreparePacket(sbusPacket, rcChannels, false, true);
        // go back forth
        up_down(&rcChannels[1]);
        sbusTime = currentMillis + SBUS_UPDATE_RATE;
        
 /*
        if (read_echo() != SBUS_PACKET_LENGTH) {
          digitalWrite(LED_BUILTIN, LOW);
        } else {
          int problem = 0;
          for(int i=0; i<SBUS_PACKET_LENGTH; ++i) {
            if (sbusPacket[i] != echo[i]) {
              problem++;
              break;
            }
          }
          if (problem) {
            digitalWrite(LED_BUILTIN, LOW);
          }else{
            digitalWrite(LED_BUILTIN, HIGH);
          }
        }
    */
    }
}
