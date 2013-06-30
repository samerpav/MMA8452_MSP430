#include <msp430g2452.h>
#include "i2c_usi_mst.h"

#define SLAVE_I2C_ADDR         (0x1C)

UINT8 dataZ;

BOOL write_reg(UINT8 reg, UINT8 value);
BOOL read_reg(UINT8 reg, UINT8 *value);

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog

  if (CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF)
  {
    while(1);                               // If calibration constants erased
                                            // do not load, trap CPU!!
  }
  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
  DCOCTL = CALDCO_1MHZ;

  // setup master node
  i2c_usi_mst_init();
  P1SEL |= BIT6 + BIT7;

  // P2.2 supplies VCC to CMA3000
  P2DIR |= BIT2;
  P2OUT &= ~BIT2;
  __delay_cycles(10000);
  P2OUT |= BIT2;
  __delay_cycles(10000);

  __enable_interrupt();

  // set CTRL register
  if(write_reg(0x02, 0x85) != TRUE)
  {
	while(1);
  }

  while(1)
  {
	// delay ~1s
    __delay_cycles(1000000);

    // read out DOUTZ register
    if(read_reg(0x08, &dataZ) != TRUE)
    {
      __no_operation(); // for debugger
    }
  }
}

BOOL write_reg(UINT8 reg, UINT8 value)
{
  BOOL ret = FALSE;

  // generate START
  i2c_usi_mst_gen_start();

  // send slave address with WRITE bit
  if(i2c_usi_mst_send_address(SLAVE_I2C_ADDR, FALSE) == TRUE)
  {
    // receive ACK for slave address, send register address to be written
    if(i2c_usi_mst_send_byte(reg) == TRUE)
    {
      // send register value
      if(i2c_usi_mst_send_byte(value) == TRUE)
      {
    	ret = TRUE;
      }
    }
  }

  // generate STOP
  i2c_usi_mst_gen_stop();

  return ret;
}

BOOL read_reg(UINT8 reg, UINT8 *value)
{
  BOOL ret = FALSE;

  // generate START
  i2c_usi_mst_gen_start();

  // send slave address with WRITE bit
  if(i2c_usi_mst_send_address(SLAVE_I2C_ADDR, FALSE) == TRUE)
  {
    // receive ACK for slave address, send register address to be read
    if(i2c_usi_mst_send_byte(reg) == TRUE)
    {
      // generate RESTART
      i2c_usi_mst_gen_repeated_start();

      // send slave address with READ bit
      if(i2c_usi_mst_send_address(SLAVE_I2C_ADDR, TRUE) == TRUE)
      {
        // send register value
        *value = i2c_usi_mst_read_byte();

        // send ACK
        i2c_usi_mst_send_n_ack(TRUE);

        ret = TRUE;
      }
    }
  }

  // generate STOP
  i2c_usi_mst_gen_stop();

  return ret;
}

