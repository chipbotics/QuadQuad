/**************************/
/* QuadQuad Demo          */
/* Author : Christie Nel  */
/* Date   : 22/05/2019    */
/**************************/

/* How to get started:
 *
 * Note: This demo was tested on an Arduino Nano v3.
 *
 * (1) Connect the Arduino to the QuadQuad PCB:
 *     - GND to GND
 *     - 5V to V+
 *     - MISO to SDO
 *     - MOSI to SDI
 *     - SCK to SCK
 *     - qq_pin_chip_select to SS
 *     - qq_pin_backoff to SBO
 *     - qq_pin_reset to RESET (optional)
 *     - Quadrature decoder A/B to Q1A/Q1B in any order.
 * 
 * (2) Open the Arduino Serial Monitor and set it to 115200 baud.
 * (3) Download this program to your Arduino.  If all is well, you should see
 *     serial output.
 * (4) You may touch/connect Q1in to GND to test the HOME/INDEX input.
 * 
 * Please read the QuadQuad Datasheet to better understand the interface and
 * read through the QuadQuad_Lib .h file for a list of all functions, errors
 * and typedefs.
 */

#include <SPI.h>
#include "QuadQuad_Lib.h"

/* Local defines */
#define FALSE 0
#define TRUE 1
  
/* ---------------------------------------------- */

/* Arduino pins */
const uint8_t qq_pin_chip_select = 10;
const uint8_t qq_pin_backoff = 2;
const uint8_t qq_pin_reset = 3;

/* Instantiate QuadQuad class with configured pins */
QuadQuad qq( qq_pin_chip_select, qq_pin_backoff, qq_pin_reset );

// ----------------------------------------------------------

void setup()
{
  Serial.begin( 115200 );
  Serial.println();
}

void print_result( err rc, err packet_rc )
{
  if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
    Serial.println( "Success" );
  else
  {
    Serial.print( "Error " );
    Serial.print( rc );
    Serial.print( ", Packet Error " );
    Serial.println( packet_rc );
  }
}

void loop()
{
  err rc;
  err packet_rc;
  byte buf[64];
  uint8_t chan;
  uint8_t i;
  uint8_t samples;
  int16_t payload_size;
  uint8_t fw_ver_major;
  uint8_t fw_ver_minor;
  uint8_t protocol_ver;
  chan_data_masks_t chan_data_masks[4];
  data_masks_t data_masks;
  motion_data_t motion_data;
  input_params_t input_params[4];
  uint8_t history_len;
  uint8_t history_time_bits;
  stream_config_t stream_config;
  stream_data_t stream_data;
  uint16_t stream_period;
  
  Serial.println( "QuadQuad Demo" );
  Serial.println( "--------------------------------------------" );
  
  /* --------------------------------------------------------- */
  /* Report configured Arduino pins                            */
  
  Serial.println( "Arduino pin connections..." );
  Serial.print( "QuadQuad Chip Select Pin : " );
  Serial.println( qq_pin_chip_select );
  Serial.print( "QuadQuad Backoff Pin     : " );
  Serial.println( qq_pin_backoff );
  Serial.print( "QuadQuad Reset Pin       : " );
  Serial.println( qq_pin_reset );
  Serial.println( "Standard Arduino SPI pins are used." );
  
  /* --------------------------------------------------------- */
  /* Reset QuadQuad chip by pulling low reset line             */
  
  Serial.println();
  Serial.println( "Resetting QuadQuad..." );
  qq.reset();
  
  /* --------------------------------------------------------- */
  /* Read QuadQuad version information                         */
  
  Serial.println();
  Serial.print( "Reading version..." );
  rc = qq.get_version( buf, sizeof(buf), &payload_size, &packet_rc, &fw_ver_major, &fw_ver_minor, &protocol_ver );
  print_result( rc, packet_rc );
  
  if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
  {
    Serial.print( "Firmware Version Major : " );
    Serial.println( fw_ver_major );
    Serial.print( "Firmware Version Minor : " );
    Serial.println( fw_ver_minor );
    Serial.print( "Protocol Version       : " );
    Serial.println( protocol_ver );
  }
  
  Serial.println( "--------------------------------------------" );
  
  /* --------------------------------------------------------- */
  /* Configure which motion data is reported and its format    */
  
  Serial.print( "Configure channel data fields..." );
  chan_data_masks[0].chan_mask = CHAN_ALL_MASK;               // Which channels to set
  chan_data_masks[0].data_mask.position_bitsize = BITSIZE_32; // How many bits in position
  chan_data_masks[0].data_mask.position_relative = FALSE;     // Return delta or absolute position
  chan_data_masks[0].data_mask.velocity_enable = TRUE;        // Return velocity
  chan_data_masks[0].data_mask.status_enable = TRUE;          // Return status
  rc = qq.set_data_mask( buf, sizeof(buf), &payload_size, &packet_rc, chan_data_masks, 1 );
  print_result( rc, packet_rc );
  
  Serial.println();
  Serial.print( "Read back channel data fields..." );
  rc = qq.get_data_mask( buf, sizeof(buf), &payload_size, &packet_rc, data_masks );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Read motion data using polling                            */
  
  Serial.println();
  Serial.print( "Read motion data..." );
  rc = qq.get_motion_data( buf, sizeof(buf), &payload_size, &packet_rc, data_masks, motion_data );
  print_result( rc, packet_rc );
  
  if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
  {
    for ( chan=0; chan<QQ_NUM_CHANS; chan++ )
    {
      Serial.print( "Channel " );
      Serial.print( chan + 1 );
      Serial.println( " : " );
      Serial.print( "  Position : " );
      Serial.println( motion_data[chan].position );
      Serial.print( "  Velocity : " );
      Serial.println( motion_data[chan].velocity );
      Serial.print( "  Glitch : " );
      Serial.println( motion_data[chan].glitch ? "TRUE" : "FALSE" );
      Serial.print( "  Overspeed : " );
      Serial.println( motion_data[chan].overspeed ? "TRUE" : "FALSE" );
    }
  }
  
  /* --------------------------------------------------------- */
  /* Read and write history dimensions                         */
  
  Serial.println();
  Serial.print( "Get history dimensions..." );
  rc = qq.get_history_dims( buf, sizeof(buf), &payload_size, &packet_rc, &history_len, &history_time_bits );
  print_result( rc, packet_rc );
  
  if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
  {
    Serial.print( "History Length : " );
    Serial.println( history_len );
    Serial.print( "History Time Bits : " );
    Serial.print( history_time_bits );
    Serial.print( " = " );
    Serial.print( qq.history_time_bits_to_ms( history_time_bits ) );
    Serial.println( "ms" );
  }
  
  Serial.println();
  Serial.print( "Set history dimensions..." );
  history_len = 16;
  history_time_bits = 15;
  rc = qq.set_history_dims( buf, sizeof(buf), &payload_size, &packet_rc, history_len, history_time_bits );
  print_result( rc, packet_rc );
  
  Serial.println();
  Serial.print( "Get history dimensions..." );
  rc = qq.get_history_dims( buf, sizeof(buf), &payload_size, &packet_rc, &history_len, &history_time_bits );
  print_result( rc, packet_rc );
  
  if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
  {
    Serial.print( "History Length : " );
    Serial.println( history_len );
    Serial.print( "History Time Bits : " );
    Serial.print( history_time_bits );
    Serial.print( " = " );
    Serial.print( qq.history_time_bits_to_ms( history_time_bits ) );
    Serial.println( "ms" );
  }
  
  /* --------------------------------------------------------- */
  /* Enable reporting of stream timing and periods elapsed     */
  
  Serial.println();
  Serial.print( "Set stream config..." );
  stream_config.period_timing_enable = TRUE;
  stream_config.periods_elapsed_enable = TRUE;
  rc = qq.set_stream_config( buf, sizeof(buf), &payload_size, &packet_rc, stream_config );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Enable input on channel 1 to set position when pulled low */
  
  Serial.println();
  Serial.print( "Set input mode..." );
  /* Set channel 1 position when input is low */
  input_params[0].chan = CHAN1_MASK;
  input_params[0].value_bitsize = BITSIZE_16;
  input_params[0].input_mode = INPUT_MODE_HOME;
  input_params[0].input_polarity = INPUT_ACTIVE_LOW;
  input_params[0].value = 1234;
  /* Disable inputs on other channels */
  input_params[1].chan = ~CHAN1_MASK;
  input_params[1].input_mode = INPUT_MODE_DISABLED;
  rc = qq.set_input_mode( buf, sizeof(buf), &payload_size, &packet_rc, input_params, 2 );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Read and write stream period                              */
  
  Serial.println();
  Serial.print( "Set stream period..." );
  rc = qq.set_stream_period( buf, sizeof(buf), &payload_size, &packet_rc, qq.ms_to_period( 1000 ) );
  print_result( rc, packet_rc );
  
  Serial.println();
  Serial.print( "Get stream period..." );
  rc = qq.get_stream_period( buf, sizeof(buf), &payload_size, &packet_rc, &stream_period );
  print_result( rc, packet_rc );
  
  if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
  {
    Serial.print( "Stream Period : " );
    Serial.print( stream_period );
    Serial.print( " = " );
    Serial.print( qq.period_to_ms( stream_period ) );
    Serial.println( "ms" );
  }
  
  /* --------------------------------------------------------- */
  /* Read stream and motion data from stream                   */
  
  Serial.println();
  Serial.println( "Reading stream..." );
  
  samples = 5;
  
  for ( i=0; i<samples; i++ )
  {
    rc = qq.read_stream( buf, sizeof(buf), &payload_size, stream_config, data_masks, &stream_data );
    
    if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
    {
      Serial.print( "Sample " );
      Serial.print( i + 1 );
      Serial.print( " of " );
      Serial.print( samples );
      
      Serial.print( ", Timing=" );
      Serial.print( stream_data.period_timing );
      Serial.print( "=" );
      Serial.print( qq.period_to_ms( stream_data.period_timing ) );
      Serial.print( "ms" );
      
      Serial.print( ", Elapsed=" );
      Serial.print( stream_data.periods_elapsed );
      
      for ( chan=0; chan<QQ_NUM_CHANS; chan++ )
      {
        Serial.println();
        Serial.print( "Channel " );
        Serial.print( chan + 1 );
        Serial.print( " Position=" );
        Serial.print( stream_data.motion_data[chan].position );
        Serial.print( ", Velocity=" );
        Serial.print( stream_data.motion_data[chan].velocity );
      }
      
      Serial.println();
    }
    else
    {
      print_result( rc, packet_rc );
      break;
    }
  }
  
  /* --------------------------------------------------------- */
  /* Turn off stream by setting stream period to 0             */
  
  Serial.println();
  Serial.print( "Turn off stream..." );
  rc = qq.set_stream_period( buf, sizeof(buf), &payload_size, &packet_rc, 0 );
  print_result( rc, packet_rc );
  
  Serial.println( "--------------------------------------------" );
  
  while (1);
}
