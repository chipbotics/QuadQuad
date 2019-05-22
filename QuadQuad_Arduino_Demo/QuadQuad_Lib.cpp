/**************************/
/* QuadQuad Library v1.00 */
/* Author : Christie Nel  */
/* Date   : 22/05/2019    */
/**************************/

#include <Arduino.h>
#include <SPI.h>
#include "QuadQuad_Lib.h"

/* Basic Definitions */
#define FALSE 0
#define TRUE  1

/* Protocol Constants */
#define BUFFER_SIZE                 64
#define ASCII_STX                   2
#define RX_TIMEOUT_MS               100
#define TX_TIMEOUT_MS               100
#define QUERY_TIMEOUT_MS            100
#define STREAM_PERIOD_DIVIDER       ( 15625 / 4 )
#define STREAM_PERIOD_MS( period )  ( ( ( (uint32_t)1000 * period ) + ( STREAM_PERIOD_DIVIDER >> 1 ) ) / STREAM_PERIOD_DIVIDER )
#define STREAM_TIMEOUT_MS( period ) ( STREAM_PERIOD_MS( period ) + RX_TIMEOUT_MS )
#define SPI_BACKOFF_ACTIVE          ( digitalRead( pin_back_off ) == HIGH )

/* Packet IDs */
#define CMD_GET_VERSION             1
#define CMD_GET_BIN_DATA            2
#define CMD_STREAM_BIN_DATA         3
#define CMD_SET_STREAM_PERIOD       4
#define CMD_GET_STREAM_PERIOD       5
#define CMD_SET_DATA_MASK           6
#define CMD_GET_DATA_MASK           7
#define CMD_SET_STREAM_CFG          8
#define CMD_GET_STREAM_CFG          9
#define CMD_SET_POSITION            10
#define CMD_GET_POSITION            11
#define CMD_SET_HISTORY_DIMS        12
#define CMD_GET_HISTORY_DIMS        13
#define CMD_SET_INPUT_MODE          14
#define CMD_GET_INPUT_MODE          15
#define CMD_GET_DEBUG               99

typedef struct
{
  uint8_t buf[BUFFER_SIZE];
  uint8_t len;
} buf_type;

buf_type rx_buf;

/* ------------------------------------------- */
/* QuadQuad initialization function            */

QuadQuad::QuadQuad( uint8_t pin_chip_select_init, uint8_t pin_back_off_init, uint8_t pin_reset_init )
{
  pin_chip_select = pin_chip_select_init;
  pin_back_off = pin_back_off_init;
  pin_reset = pin_reset_init;
  
  digitalWrite( pin_chip_select, HIGH );
  digitalWrite( pin_reset, HIGH );
  
  pinMode( pin_chip_select, OUTPUT );
  pinMode( pin_back_off, INPUT );
  pinMode( pin_reset, OUTPUT );
  
  SPI.begin();
  SPI.beginTransaction( SPISettings( 2000000, MSBFIRST, SPI_MODE1 ) );
  
  stream_timeout_ms = STREAM_TIMEOUT_MS( 0 );
}

/* ------------------------------------------- */
/* QuadQuad command functions                  */

void QuadQuad::reset( void )
{
  digitalWrite( pin_reset, LOW );
  digitalWrite( pin_reset, HIGH );
}

err QuadQuad::get_version( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, uint8_t *fw_ver_major, uint8_t *fw_ver_minor, uint8_t *protocol_ver )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  
  rc = spi_queryPacket( CMD_GET_VERSION, "", 0, buf, payload_size, buf_size );
  
  *packet_rc = buf[0];
  *fw_ver_major = buf[1];
  *fw_ver_minor = buf[2];
  *protocol_ver = buf[3];
  
  return rc;
}

err QuadQuad::get_motion_data( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, data_masks_t data_masks, motion_data_t motion_data )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  uint8_t chan;
  
  rc = spi_queryPacket( CMD_GET_BIN_DATA, "", 0, buf, payload_size, buf_size );
  
  if ( rc == QQ_ERR_OK )
  {
    *packet_rc = *buf++;
    parse_motion_data( &buf, buf_size, payload_size, data_masks, motion_data );
  }
  
  return rc;
}

err QuadQuad::read_stream( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, stream_config_t stream_config, data_masks_t data_masks, stream_data_t *stream_data )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  uint8_t chan;
  
  rc = spi_readStreamPacket( buf, payload_size, buf_size );
  
  if ( rc == QQ_ERR_OK )
  {
    if ( stream_config.period_timing_enable )
    {
      stream_data->period_timing = *(uint16_t *)buf;
      buf += sizeof(uint16_t);
    }
    if ( stream_config.periods_elapsed_enable )
    {
      stream_data->periods_elapsed = *(uint8_t *)buf;
      buf += sizeof(uint8_t);
    }
    
    parse_motion_data( &buf, buf_size, payload_size, data_masks, stream_data->motion_data );
  }
  
  return rc;
}

err QuadQuad::set_stream_period( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, uint16_t stream_period )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  
  rc = spi_queryPacket( CMD_SET_STREAM_PERIOD, (uint8_t *)&stream_period, 2, buf, payload_size, buf_size );

  *packet_rc = buf[0];
  
  if ( *packet_rc == QQ_ERR_OK )
    stream_timeout_ms = STREAM_TIMEOUT_MS( stream_period );
  
  return rc;
}

err QuadQuad::get_stream_period( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, uint16_t *stream_period )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  
  rc = spi_queryPacket( CMD_GET_STREAM_PERIOD, "", 0, buf, payload_size, buf_size );

  *packet_rc = buf[0];
  *stream_period = *(uint16_t *)&buf[1];
  
  if ( *packet_rc == QQ_ERR_OK )
    stream_timeout_ms = STREAM_TIMEOUT_MS( *stream_period );
  
  return rc;
}

err QuadQuad::set_data_mask( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, chan_data_masks_t *chan_data_masks, uint8_t chan_data_masks_num )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  
  rc = spi_queryPacket( CMD_SET_DATA_MASK, (uint8_t *)chan_data_masks, chan_data_masks_num << 1, buf, payload_size, buf_size );

  *packet_rc = buf[0];
  
  return rc;
}

err QuadQuad::get_data_mask( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, data_masks_t data_masks )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  uint8_t i;
  
  rc = spi_queryPacket( CMD_GET_DATA_MASK, "", 0, buf, payload_size, buf_size );
  
  *packet_rc = *buf++;
  
  if ( *payload_size != 5 )
    rc = QQ_ERR_CMD_INVALID_DATA;
  else
  {
    for ( i=0; i<QQ_NUM_CHANS; i++ )
      *(uint8_t *)data_masks++ = *buf++;
  }
  
  return rc;
}

err QuadQuad::set_stream_config( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, stream_config_t stream_config )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  
  rc = spi_queryPacket( CMD_SET_STREAM_CFG, (uint8_t *)&stream_config, 1, buf, payload_size, buf_size );

  *packet_rc = buf[0];
  
  return rc;
}

err QuadQuad::get_stream_config( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, stream_config_t *stream_config )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  
  rc = spi_queryPacket( CMD_GET_STREAM_CFG, "", 0, buf, payload_size, buf_size );
  
  if ( *payload_size != 2 )
    rc = QQ_ERR_CMD_INVALID_DATA;
  else
  {
    *packet_rc = buf[0];
    *(uint8_t *)stream_config = buf[1];
  }
  
  return rc;
}

err QuadQuad::set_position( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, position_params_t *position_params, uint8_t position_params_num )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  uint8_t *buf_p;
  uint8_t i;
  
  buf_p = buf;
  
  for ( i=0; i<position_params_num; i++ )
  {
    *buf_p++ = *(uint8_t *)&position_params[i];
    
    switch( position_params[i].value_bitsize )
    {
      case BITSIZE_ZERO :
        break;
      case BITSIZE_8 :
        *(int8_t *)buf_p = position_params[i].value;
        buf_p += sizeof(int8_t);
        break;
      case BITSIZE_16 :
        *(int16_t *)buf_p = position_params[i].value;
        buf_p += sizeof(int16_t);
        break;
      case BITSIZE_32 :
        *(int32_t *)buf_p = position_params[i].value;
        buf_p += sizeof(int32_t);
        break;
      default :;
    }
  }
  
  rc = spi_queryPacket( CMD_SET_POSITION, buf, buf_p-buf, buf, payload_size, sizeof(buf) );

  *packet_rc = buf[0];
  
  return rc;
}

err QuadQuad::get_position( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, E_GET_POS_MODE mode, E_CHAN_MASK chan_mask, data_masks_t data_masks, positions_t positions )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  uint8_t *buf_p;
  uint8_t i;
  int16_t size;
  
  switch( mode )
  {
    case GET_POS_AS_CONFIGURED :
      rc = spi_queryPacket( CMD_GET_POSITION, "", 0, buf, payload_size, buf_size );
      break;
    case GET_POS_BY_CHAN_MASK :
      rc = spi_queryPacket( CMD_GET_POSITION, (uint8_t *)&chan_mask, 1, buf, payload_size, buf_size );
      break;
    case GET_POS_BY_SELECTION :
      rc = spi_queryPacket( CMD_GET_POSITION, (uint8_t *)data_masks, 4, buf, payload_size, buf_size );
      break;
    default:
      *payload_size = 0;
  }
  
  buf_p = buf;
  
  if ( *payload_size >= 1 )
    *packet_rc = *buf_p++;
  
  size = *payload_size - 1; // -1 for packet_rc
  
  for ( i=0; i<QQ_NUM_CHANS; i++ )
  {
    E_BITSIZE value_bitsize;
    
    switch( mode )
    {
      case GET_POS_AS_CONFIGURED :
      case GET_POS_BY_SELECTION :
        value_bitsize = data_masks[i].position_bitsize;
        break;
      case GET_POS_BY_CHAN_MASK :
        if ( chan_mask & ( 1 << i ) )
          value_bitsize = data_masks[i].position_bitsize;
        else
          value_bitsize = BITSIZE_ZERO;
        break;
      default :;
    }
    
    switch( value_bitsize )
    {
      case BITSIZE_ZERO :
        break;
      case BITSIZE_8 :
        size -= sizeof(int8_t);
        if ( size < 0 )
          break;
        positions[i] = *(int8_t *)buf_p;
        buf_p += sizeof(int8_t);
        break;
      case BITSIZE_16 :
        size -= sizeof(int16_t);
        if ( size < 0 )
          break;
        positions[i] = *(int16_t *)buf_p;
        buf_p += sizeof(int16_t);
        break;
      case BITSIZE_32 :
        size -= sizeof(int32_t);
        if ( size < 0 )
          break;
        positions[i] = *(int32_t *)buf_p;
        buf_p += sizeof(int32_t);
        break;
      default :;
    }
  }
  
  if ( size < 0 )
    rc = QQ_ERR_CMD_INVALID_DATA;
  
  return rc;
}

err QuadQuad::set_history_dims( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, uint8_t history_len, uint8_t history_time_bits )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  
  buf[0] = history_len;
  buf[1] = history_time_bits;
  rc = spi_queryPacket( CMD_SET_HISTORY_DIMS, buf, 2, buf, payload_size, sizeof(buf) );

  *packet_rc = buf[0];
  
  return rc;
}

err QuadQuad::get_history_dims( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, uint8_t *history_len, uint8_t *history_time_bits )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  
  rc = spi_queryPacket( CMD_GET_HISTORY_DIMS, "", 0, buf, payload_size, buf_size );
  
  if ( *payload_size != 3 )
    rc = QQ_ERR_CMD_INVALID_DATA;
  else
  {
    *packet_rc = buf[0];
    *history_len = buf[1];
    *history_time_bits = buf[2];
  }
  
  return rc;
}

err QuadQuad::set_input_mode( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, input_params_t *input_params, uint8_t input_params_num )
{
  err rc = QQ_ERR_UNSPECIFIED_ERROR;
  uint8_t *buf_p;
  uint8_t i;
  
  buf_p = buf;
  
  for ( i=0; i<input_params_num; i++ )
  {
    *(uint16_t *)buf_p = *(uint16_t *)&input_params[i];
    buf_p += sizeof(uint16_t);
    
    if ( input_params[i].input_mode != INPUT_MODE_DISABLED )
    {
      switch( input_params[i].value_bitsize )
      {
        case BITSIZE_ZERO :
          break;
        case BITSIZE_8 :
          *(int8_t *)buf_p = input_params[i].value;
          buf_p += sizeof(int8_t);
          break;
        case BITSIZE_16 :
          *(int16_t *)buf_p = input_params[i].value;
          buf_p += sizeof(int16_t);
          break;
        case BITSIZE_32 :
          *(int32_t *)buf_p = input_params[i].value;
          buf_p += sizeof(int32_t);
          break;
        default :;
      }
    }
  }
  
  rc = spi_queryPacket( CMD_SET_INPUT_MODE, buf, buf_p-buf, buf, payload_size, sizeof(buf) );

  *packet_rc = buf[0];
  
  return rc;
}

/* ------------------------------------------- */
/* QuadQuad helper functions                   */

uint16_t QuadQuad::period_to_ms( uint16_t period_timing )
{
  return STREAM_PERIOD_MS( period_timing );
}

uint16_t QuadQuad::ms_to_period( uint16_t ms )
{
  return ( ( (uint32_t)ms * STREAM_PERIOD_DIVIDER ) + 500 ) / 1000;
}

uint16_t QuadQuad::history_time_bits_to_ms( uint8_t history_time_bits )
{
  return ( ( ( (uint32_t)1 << history_time_bits ) * 1000 ) + 31250 ) / 62500;
}

/* ------------------------------------------- */
/* Private functions                           */

uint8_t QuadQuad::spi_read_byte( uint8_t *byte_read )
{
  /*
     Returns: 1 if data available
              0 if no data available
  */
  
  uint32_t time = millis();
  bool timeout = 0;
  
  while ( SPI_BACKOFF_ACTIVE && ( !timeout ) )
    timeout = ( millis() - time > RX_TIMEOUT_MS );
  
  if ( !timeout )
  {
    SPDR = 0;
    while ( !( SPSR & ( 1 << SPIF ) ) );
    *byte_read = SPDR;
    return 1;
  }
  else
    return 0;
}

err QuadQuad::spi_write( uint8_t *data, uint8_t len )
{
  err rc = QQ_ERR_OK;
  uint8_t dummy;
  uint32_t time = millis();
  uint8_t timeout = 0;
  uint8_t i;
  
  for ( i=0; (i<len)&&(!timeout); i++ )
  {
    while ( SPI_BACKOFF_ACTIVE && ( !timeout ) )
      timeout = ( millis() - time > TX_TIMEOUT_MS );
    
    if ( ( !SPI_BACKOFF_ACTIVE ) && ( !timeout ) )
    {
      SPDR = *data++;
      while( !( SPSR & ( 1 << SPIF ) ) );
      dummy = SPDR;
    }
  }
  
  if ( timeout )
    rc = QQ_ERR_SPI_TX_TIMEOUT;
  
  return rc;
}

void QuadQuad::spi_sendBytes( uint8_t *data, uint8_t data_len, uint8_t *checksum, err *error )
{
  if ( *error == QQ_ERR_OK )
  {
    spi_write( data, data_len );

    while ( data_len-- )
      *checksum += *data++;
  }
}

err QuadQuad::spi_sendPacket( uint8_t packet_type, uint8_t *payload, uint8_t payload_size, err return_code )
{
  err rc = QQ_ERR_OK;
  uint8_t packet_size = payload_size + 4;
  uint8_t stx = ASCII_STX;
  uint8_t checksum = 0;
  
  if ( payload_size > ( 255 - 4 ) )
    rc = QQ_ERR_SPI_TX_PACKET_TOO_BIG;
  else
  {
    spi_sendBytes( &stx, 1, &checksum, &rc );
    spi_sendBytes( (unsigned char *)&packet_size, 1, &checksum, &rc );
    spi_sendBytes( (unsigned char *)&packet_type, 1, &checksum, &rc );
    spi_sendBytes( payload, payload_size, &checksum, &rc );
    checksum = ~checksum + 1;
    spi_sendBytes( &checksum, 1, &checksum, &rc );
  }

  return rc;
}

err QuadQuad::spi_receivePacket( uint8_t *packet_type, uint8_t *payload, uint8_t payload_buf_size, int16_t *payload_size )
{
  err rc = QQ_ERR_OK;
  bool corruption;
  uint8_t bytes;
  uint8_t packet_size;
  uint32_t time = millis();
  uint8_t i;

  do
  {
    corruption = FALSE;

    bytes = spi_read_byte( rx_buf.buf + rx_buf.len );

    if ( bytes > 0 )
    {
      rx_buf.len += bytes;
      
      if ( rx_buf.buf[0] != ASCII_STX )
      {
        /* Corrupted data, no STX at start of packet. */

        /* Look for start of packet (STX) */
        for ( i=0; rx_buf.buf[i]!=ASCII_STX && i<rx_buf.len; i++ );

        /* We found a potential STX -> remove broken data before */
        if ( i <= rx_buf.len )
          memcpy( rx_buf.buf, rx_buf.buf + i, rx_buf.len - i );

        /* Shorten the buffer length by number of bytes removed. */
        rx_buf.len -= i;
      }

      if ( rx_buf.len >= 2 )
      {
        /* Enough data to get STX + packet size */

        packet_size = *(uint8_t*)( (uint8_t*)&rx_buf.buf + 1 );

        if ( rx_buf.len >= packet_size )
        {
          /* We have the whole packet */
          uint8_t checksum = 0;

          for ( i=0; i<packet_size; i++ )
            checksum += rx_buf.buf[i];

          if ( checksum == 0 )
          {
            /* Packet data is valid */
            *packet_type = *(rx_buf.buf + 2);
            *payload_size = packet_size - 4;
            
            if ( *payload_size <= payload_buf_size )
            {
              memcpy( payload, rx_buf.buf + 3, *payload_size );

              /* Move any remaining to start of buffer */
              rx_buf.len -= packet_size;
              if ( rx_buf.len )
                memcpy( rx_buf.buf, rx_buf.buf + packet_size, rx_buf.len );
            }
            else
              rc = QQ_ERR_SPI_RX_PACKET_TOO_BIG;
          }
          else
          {
            /* Corrupted packet, we don't know where it starts.  Remove STX and retry. */
            rx_buf.buf[0] = 0;
            corruption = TRUE;
          }
        }
        else
          *payload_size = -1;
      }
      else
        *payload_size = -1;
    }
    else
      *payload_size = -1;

    if ( bytes < 0 )
      rc = QQ_ERR_SPI_RX_FAIL;
    
    if ( millis() - time > RX_TIMEOUT_MS )
      rc = QQ_ERR_SPI_RX_TIMEOUT;
  } while ( corruption && rc == QQ_ERR_OK );

  return rc;
}

err QuadQuad::spi_queryPacket( uint8_t packet_type, uint8_t *payload_tx, uint8_t payload_size_tx, uint8_t *payload_rx, int16_t *payload_size_rx, uint8_t payload_buf_size )
{
  err rc;
  uint32_t time;
  uint8_t packet_type_read;
  bool timeout = FALSE;
  
  digitalWrite( pin_chip_select, LOW );
  rc = spi_sendPacket( packet_type, payload_tx, payload_size_tx, 0 );
  digitalWrite( pin_chip_select, HIGH );
  
  if ( rc == QQ_ERR_OK )
  {
    time = millis();
    do {
      timeout = ( millis() - time > QUERY_TIMEOUT_MS );
    } while ( SPI_BACKOFF_ACTIVE && ( !timeout ) );
    
    if ( timeout )
      rc = QQ_ERR_SPI_QUERY_TIMEOUT;
    else
    {
      digitalWrite( pin_chip_select, LOW );
      
      *payload_size_rx = -1;
      rx_buf.len = 0;
      
      do {
        rc = spi_receivePacket( &packet_type_read, payload_rx, payload_buf_size, payload_size_rx );
      } while ( ( ( millis() - time <= QUERY_TIMEOUT_MS ) && ( rc == QQ_ERR_OK ) && ( ( packet_type_read != packet_type ) || ( *payload_size_rx < 0 ) ) ) );
      
      if ( ( ( *payload_size_rx < 0 ) || ( packet_type_read != packet_type ) ) && ( rc == QQ_ERR_OK ) )
        rc = QQ_ERR_SPI_QUERY_TIMEOUT;
    }
    
    digitalWrite( pin_chip_select, HIGH );
  }
  
  return rc;
}

err QuadQuad::spi_readStreamPacket( uint8_t *payload_rx, int16_t *payload_size_rx, uint8_t payload_buf_size )
{
  err rc;
  uint32_t time;
  bool timeout;
  uint8_t packet_type;
  
  digitalWrite( pin_chip_select, HIGH );
  
  time = millis();
  do {
    timeout = ( millis() - time > stream_timeout_ms );
  } while ( SPI_BACKOFF_ACTIVE && ( !timeout ) );
  
  if ( timeout )
    rc = QQ_ERR_SPI_STREAM_TIMEOUT;
  else
  {
    digitalWrite( pin_chip_select, LOW );
    
    *payload_size_rx = -1;
    rx_buf.len = 0;
    
    do {
      rc = spi_receivePacket( &packet_type, payload_rx, payload_buf_size, payload_size_rx );
    } while ( ( ( millis() - time <= stream_timeout_ms ) && ( rc == QQ_ERR_OK ) && ( ( packet_type != CMD_STREAM_BIN_DATA ) || ( *payload_size_rx < 0 ) ) ) );
    
    if ( ( ( *payload_size_rx < 0 ) || ( packet_type != CMD_STREAM_BIN_DATA ) ) && ( rc == QQ_ERR_OK ) )
      rc = QQ_ERR_SPI_STREAM_TIMEOUT;
  }
  
  digitalWrite( pin_chip_select, HIGH );
  
  return rc;
}

void QuadQuad::parse_motion_data( uint8_t **buf, uint8_t buf_size, int16_t *payload_size, data_masks_t data_masks, motion_data_t motion_data )
{
  uint8_t chan;
  
  for ( chan=0; chan<QQ_NUM_CHANS; chan++ )
  {
    data_mask_t data_mask = data_masks[chan];
    
    switch ( data_mask.position_bitsize )
    {
      case BITSIZE_8:
        motion_data[chan].position = **(int8_t **)buf;
        *buf += sizeof(int8_t);
        break;
      case BITSIZE_16:
        motion_data[chan].position = **(int16_t **)buf;
        *buf += sizeof(int16_t);
        break;
      case BITSIZE_32:
        motion_data[chan].position = **(int32_t **)buf;
        *buf += sizeof(int32_t);
        break;
      default:
        break;
    }
    if ( data_mask.velocity_enable )
    {
      motion_data[chan].velocity = **(int16_t **)buf;
      *buf += sizeof(int16_t);
    }
    if ( data_mask.status_enable )
    {
      uint8_t status = **(uint8_t **)buf;
      *buf += sizeof(uint8_t);
      
      motion_data[chan].overspeed = ( status & QQ_STATUS_MASK_OVERSPEED ) ? TRUE : FALSE;
      motion_data[chan].glitch = ( status & QQ_STATUS_MASK_INVALID ) ? TRUE : FALSE;
    }
  }
}
