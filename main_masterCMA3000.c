#include <msp430f2013.h>
#include "i2c_usi_mst.h"

#define SLAVE_I2C_ADDR         (0x1D)

UINT8 dataZ;
unsigned char interrupt_triggered = 0;

BOOL write_reg(UINT8 reg, UINT8 value);
BOOL read_reg(UINT8 reg, UINT8 *value);

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD; 		// Stop watchdog timer

  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
  DCOCTL = CALDCO_1MHZ;

  i2c_usi_mst_init();

  // the only INPUT pin is P1.4 (INT1)
  P1DIR = 0xFF;
  P1DIR &= ~BIT4;
  P1OUT = 0;
  P1SEL = BIT6 + BIT7;

  P2DIR = 0xFF;
  P2OUT = 0;
  P2SEL = 0;

  // interrupt check on P1.4
  P1IE = BIT4;
  P1IES &= ~BIT4;	// rise edge
  P1IFG &= ~BIT4; 	// P1.4 IFG cleared

  __enable_interrupt();

  // WHO_AM_I
  read_reg(0x0D, &dataZ);
  if(dataZ == 0x2A)
	  P1OUT |= BIT0;

  write_reg(0x2A, 0x28); // 12.5 Hz Hz ODR, standby	0x28 = 101000
  write_reg(0x2B, 0x03); // MODS = 0x03 (low power)

  write_reg(0x2C, 0x02); // push-pull, active high
  write_reg(0x1D, 0x1E); // ELE,x,y,z enabled
  write_reg(0x1F, 0x01); // threshold
  write_reg(0x20, 0x02); // debounce counter
  write_reg(0x2D, 0x20); // interrupt enable register desc, int 1
  write_reg(0x2E, 0x20); // interrupt config register desc, int 1

  // set to ACTIVE mode
  write_reg(0x2A, 0x29);	// 12.5 Hz ODR 0x29 = 101001

  while (1)
  {
	  // enter low power mode
	  LPM3;

	  if (interrupt_triggered)
	  {
		  // clear transient event status register
		  read_reg(0x1E, &dataZ);

		  interrupt_triggered = 0;

		  // have to set it to ACTIVE mode again to resume i2c operations!! don't know why?!?!?
		  write_reg(0x2A, 0x29);	// 0x29 = 101001
	  }
  }
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
	// determine source of interrupt -- don't need this for now
	//read_reg(0x0C, &dataZ);

	int j;
	for (j=10; j>0; j--)
	{
		volatile unsigned int i;
		P1OUT ^= BIT0;
		i = 3000;
		do i--;
		while(i != 0);
	}

	P1IFG &= ~BIT4; // P1.4 IFG cleared
	P1OUT &= ~BIT0; // turn off LED

	interrupt_triggered = 1;

	// exit low power mode
	LPM3_EXIT;
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
