/// Woodpecker throttle settings: 0.9V - idle  1.3-1.4V - start moving
// CH1 value 1177-1210  and DAC value 1044-1159 - start moving
// CH1 value 1792-1808  and DAC value 3253-3310 - max value

#include "SPI.h"
#include "mcp_can.h"
#include "can_frame.h"
#include "DAC_MCP49xx.h"
#include "common.h"

// PIN 2 Relay
// PIN 3 Radio reciever Throttle - CH1 Values - 983 - 2001
// PIN 4 Radio reciever SF switch - CH2 983 - 2001
// PIN 5 Radio reciever SG switch - CH3 983- 1495 - 2001
// PIN 7 Relay
// PIN 8 Relay
// PIN 9 DAC
// PIN 10 CAN
// PIN 10 Relay (disabled)

byte relayPin[4] = {
  2,7,8,11};
//D2 -> RELAY1
//D7 -> RELAY2
//D8 -> RELAY3
//D11 -> RELAY
int val;
#define SIGNAL_INPUT_A A0     // input pin for sensing sensor 1 output
#define SIGNAL_INPUT_B A1     // input pin for sensing sensor 2 output
#define SPOOF_SIGNAL_A A2     // input pin for sensing DAC 1 output
#define SPOOF_SIGNAL_B A3     // input pin for sensing DAC 2 output
#define SPOOF_ENGAGE 6        // signal interrupt (relay) for spoofed signals
#define DAC_CS 9              // chip select pin for DAC
#define CAN_CS 10             // chip select pin for CAN

DAC_MCP49xx dac( DAC_MCP49xx::MCP4922, 9 ); // DAC model, SS pin, LDAC pin

// Construct the CAN shield object
MCP_CAN CAN( CAN_CS );

    uint16_t dac_value = 0;
    uint16_t dac_val_a = 0;
    uint16_t dac_val_b = 0;

    uint16_t spoof_a_adc_signal = 0;
    uint16_t spoof_b_adc_signal = 0;

    float spoof_a_adc_volts = 0.0;
    float spoof_b_adc_volts = 0.0;

    float dac_expected_output_a = 0.0;
    float dac_expected_output_b = 0.0;
    
    int dac_value1 = 0;

int ch1; // Throttle
int ch2; // Power ON Switch
int ch3; // Reverse-Forwrad Switch
unsigned char Distance1m[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09};

void setup( )
{
    init_serial();
    init_can();
 //   test_interrupt_relay();
  for(int i = 0; i < 4; i++)  pinMode(relayPin[i],OUTPUT);
  for(int j = 0; j < 4; j++)  digitalWrite(relayPin[j],LOW);
  pinMode(SPOOF_ENGAGE,OUTPUT);
  digitalWrite( SPOOF_ENGAGE, LOW );
//  test_CAN_recieve( ); 

}


void loop()
{
 CAN.sendMsgBuf(0x301, 0, 8, Distance1m);     
  ch1 = pulseIn(3, HIGH, 25000); // Read the pulse width Throttle
  ch2 = pulseIn(4, HIGH, 25000); // Read the pulse width SG switch
  ch3 = pulseIn(5, HIGH, 25000); // Read the pulse width SF switch 
  
  //      Serial.print("CH1 Value = ");
  //      Serial.print(ch1);
  
  //      Serial.print("\tCH2 Value = ");
  //      Serial.print(ch2);

  //      Serial.print("\tCH3 Value = ");
   //     Serial.print(ch3);
                
        
  int voltage = map(ch1, 900, 2000, 50, 3300);  //map(value, fromLow, fromHigh, toLow, toHigh)
  DACS_command (voltage);
        if ((ch2 >=1950) && (ch2<= 2050)){
          digitalWrite(relayPin[2],HIGH);
               } 
          else {
            digitalWrite(relayPin[2],LOW);
               }
          
          if ((ch3 >=950) && (ch3<= 1000)){
          digitalWrite(relayPin[0],HIGH);
               } 
          else if ((ch3 >=1950) && (ch3<= 2050)){
          digitalWrite(relayPin[1],HIGH);
               }     
          else {
            digitalWrite(relayPin[0],LOW);
            digitalWrite(relayPin[1],LOW);
               }

          if ((ch1 >=1050) && (ch1<= 2050)){
          digitalWrite( SPOOF_ENGAGE, HIGH );
               } 
          else {
          digitalWrite( SPOOF_ENGAGE, LOW );
               }
      
    if (Serial.available() > 0) {
    int inByte = Serial.read();

     switch (inByte) {
       case '1':
        Serial.println("Switch Relay OFF");
        digitalWrite( SPOOF_ENGAGE, LOW );       // start off with the relay in it's depowered state
       break;
       case '2':
        Serial.println("Switch Relay ON");
        digitalWrite( SPOOF_ENGAGE, HIGH );       // start off with the relay in it's depowered state
       break;
       case '3':
        Serial.println("DACS Command 0");
        DACS_command (0);  
       break;
      case '4':
        Serial.println("DACS Command Init Value 750  0.9V");
        DACS_command (750);  
       break;
        case '5':
        Serial.println("DACS Command Start Moving 1200 1.4V");
        DACS_command (1200);  
       break;
       case '6':
        Serial.println("DACS Command Start Moving 1400 1.7V");
        DACS_command (1400);  
       break;
       case '7':
        Serial.println("CAN Send");  
        test_CAN_send(); 
        break;
       case '8':
        Serial.println("Relay Test");
         digitalWrite(relayPin[2],HIGH);
         delay (500);       
         digitalWrite(relayPin[2],LOW);                                                                    
         break;
       case 'a':                         
        Serial.println("Relay1");
        val=digitalRead(relayPin[0]);
        val=!val;
        digitalWrite(relayPin[0],val);
        break;
      case 's':
        Serial.println("Relay2");
        val=digitalRead(relayPin[1]);
        val=!val;
        digitalWrite(relayPin[1],val);
        break;
      case 'd':
        Serial.println("Relay3");
        val=digitalRead(relayPin[2]);
        val=!val;
        digitalWrite(relayPin[2],val);
        break;
      case 'f':
        Serial.println("Relay4");
        val=digitalRead(relayPin[3]);
        val=!val;
        digitalWrite(relayPin[3],val);
        break;  
     }
    }       
  
    //test_DACS();
    //test_interrupt_relay();
    //test_CAN_send();
    // test_CAN_recieve();
    //test_signal_sense();

}


static void init_serial( void )
{
    Serial.begin( SERIAL_BAUD );
}


static void init_can ( void )
{
    // wait until we have initialized
    while( CAN.begin(CAN_BAUD) != CAN_OK )
    {
        // wait a little
        delay( CAN_INIT_RETRY_DELAY );
        Serial.println( "init_can: retrying" );
    }

    // debug log
    Serial.println( "init_can: pass" );
}

void test_DACS( )                // the two DACS have circuitry for measuring the output. Create a signal and
                                // measure that the output is what is expected.
{
    uint16_t dac_value;
    uint16_t dac_val_a;
    uint16_t dac_val_b;

    uint16_t spoof_a_adc_signal = 0;
    uint16_t spoof_b_adc_signal = 0;

    float spoof_a_adc_volts = 0.0;
    float spoof_b_adc_volts = 0.0;

    float dac_expected_output_a = 0.0;
    float dac_expected_output_b = 0.0;

    // energize the relay so we can read the values at the terminal
    digitalWrite( SPOOF_ENGAGE, HIGH );

    for ( dac_value = 0; dac_value < 4095; dac_value = dac_value + 15 )
    {
        dac_val_a = dac_value;
        dac_val_b = 4095.0 - dac_value;

        // Convert 12-bit DAC output integer value to volts ( 5V / 4096steps )
        // Maximum voltage is 5 Volts.
        dac_expected_output_a = ( 5.0 / 4095.0 ) * dac_val_a;
        dac_expected_output_b = ( 5.0 / 4095.0 ) * dac_val_b;

        dac.outputA( dac_val_a );
        dac.outputB( dac_val_b );

        delay( 2000 );

        spoof_a_adc_signal = analogRead( SPOOF_SIGNAL_A );
        spoof_b_adc_signal = analogRead( SPOOF_SIGNAL_B );

        // Convert 10-bit ADC input integer value to volts ( 5V / 1024steps )
        // Maximum voltage is 5 Volts.
        spoof_a_adc_volts = ( spoof_a_adc_signal * 5.0 ) / 1023.0;
        spoof_b_adc_volts = ( spoof_b_adc_signal * 5.0 ) / 1023.0;

        Serial.print( "\tDAC Value (A): " );
        Serial.print( dac_val_a );

        Serial.print( "\tDAC Value (B): " );
        Serial.print( dac_val_b );

        Serial.print( "\tOutput A Voltage: " );
        Serial.print( dac_expected_output_a, 3 );

        Serial.print( "\tOutput B Voltage: " );
        Serial.println( dac_expected_output_b, 3 );

        Serial.print( "Spoof A Value: " );
        Serial.print( spoof_a_adc_signal );

        Serial.print( "\tSpoof B Value: " );
        Serial.print( spoof_b_adc_signal );

        Serial.print( "\tSpoof A Voltage: " );
        Serial.print( spoof_a_adc_volts, 3 );

        Serial.print( "\tSpoof B Voltage: " );
        Serial.println( spoof_b_adc_volts, 3 );

        Serial.println( "" );
    }
}

void test_interrupt_relay( ) 
{
        digitalWrite( SPOOF_ENGAGE, LOW );       // start off with the relay in it's depowered state
        delay( 500 );                            // wait half a second

        digitalWrite( SPOOF_ENGAGE, HIGH );      // energize the relay
        delay( 500 );                            // wait half a second

        digitalWrite( SPOOF_ENGAGE, LOW );       // start off with the relay in it's depowered state
        delay( 500 );                            // wait half a second

        digitalWrite( SPOOF_ENGAGE, HIGH );      // energize the relay
        delay( 500 );                            // wait half a second
}

// send a CAN frame, to be recieved by some module on a CAN bus.
void test_CAN_send( ) 
{
    int cantxValue = 60;

    Serial.print( "cantxValue: " );
    Serial.println( cantxValue );

    //Create data packet for CAN message
    unsigned char canMsg[ 8 ] = {   cantxValue, 
                                    0x01, 
                                    0x02, 
                                    0x03, 
                                    0x04, 
                                    0x05, 
                                    0x06, 
                                    0x07 };

    // send data:  id = 0x123, standrad frame, data len = 8, stmp: data buf
    CAN.sendMsgBuf( 0x07B, 0, 8, canMsg ); 
    delay( 250 );
}


// recieve a CAN frame sent from some module on a CAN bus.
void test_CAN_recieve( ) 
{
    // local vars
    can_frame_s rx_frame;

    if( CAN.checkReceive() == CAN_MSGAVAIL )
    {
        memset( &rx_frame, 0, sizeof(rx_frame) );

        // read frame
        CAN.readMsgBufID(
                (INT32U*) &rx_frame.id,
                (INT8U*) &rx_frame.dlc,
                (INT8U*) rx_frame.data );

        Serial.print( "canRxValue:" );
        Serial.println( rx_frame.id );
        delay(250);
    }
}

void DACS_command(int dac_value1)               
                                
{       dac_val_a = dac_value1;
   //     dac_val_b = 4095.0 - dac_value1;

        // Convert 12-bit DAC output integer value to volts ( 5V / 4096steps )
        // Maximum voltage is 5 Volts.
        dac_expected_output_a = ( 5.0 / 4095.0 ) * dac_val_a;
//        dac_expected_output_b = ( 5.0 / 4095.0 ) * dac_val_b;

        dac.outputA( dac_val_a );
  //      dac.outputB( dac_val_b );

   //     delay( 500 );

        spoof_a_adc_signal = analogRead( SPOOF_SIGNAL_A );
   //     spoof_b_adc_signal = analogRead( SPOOF_SIGNAL_B );

        // Convert 10-bit ADC input integer value to volts ( 5V / 1024steps )
        // Maximum voltage is 5 Volts.
        spoof_a_adc_volts = ( spoof_a_adc_signal * 5.0 ) / 1023.0;
//        spoof_b_adc_volts = ( spoof_b_adc_signal * 5.0 ) / 1023.0;

  //      Serial.print( "\tDAC Value (A): " );
  //      Serial.print( dac_val_a );

   //     Serial.print( "\tOutput A Voltage: " );
   //     Serial.print( dac_expected_output_a, 3 );

   //     Serial.print( "\tSpoof A Value: " );
   //     Serial.print( spoof_a_adc_signal );

   //     Serial.print( "\tSpoof A Voltage: " );
   //    Serial.print( spoof_a_adc_volts, 3 );

   //     Serial.println( "" );
    
}
