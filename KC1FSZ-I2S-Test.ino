// This is a very simple "hello world" test of the K20 (Teensy 3.2)
// I2S audio output.
//
// Uses 16 bit audio.  Plays a ramp through both the L and R channels.
//
// Bruce MacKinnon KC1FSZ
//
// Here is the K20 reference manual:
// https://www.nxp.com/docs/en/reference-manual/K20P64M72SF1RM.pdf
//
//
//
// Clocking calculation (AK4556)
//
//   96 MHz system clock
//   9/423 multiply/divide
//   2,042,533 Hz MCLK
//   DIV=1 -> divide by (1+1)*2=4
//   510,638 Hz BCLK
//   There are 24x2 bits in a frame (L+R)
//   Fs is 7,978 Hz 
//
#include <CircularBuffer.h>

#define MCLK_FRACT 9
#define MCLK_DIVIDE 423

void setup() {

  Serial.begin(9600);
  delay(1000);
  Serial.println("KC1FSZ");

  // Configure the Teensy 3.2 pins per the reference and wiring
  PORTC_PCR3 = PORT_PCR_MUX(6); // Alt 6 is BLCK - T3.2 pin 9
  PORTC_PCR6 = PORT_PCR_MUX(6); // Alt 6 is MCLK - T3.2 pin 11
  PORTC_PCR1 = PORT_PCR_MUX(6); // Alt 6 is TXD0 - T3.2 pin 22
  PORTC_PCR2 = PORT_PCR_MUX(6); // Alt 6 is LRCLK - T3.2 pin 23
  PORTC_PCR5 = PORT_PCR_MUX(4); // Alt 4 is RXD0 - T3.2 pin 13
  
  // System gating control register.  I2S clock gate control is enabled.
  SIM_SCGC6 |= SIM_SCGC6_I2S;
  // The MCLK is sourced from the system clock | The MCLK is enabled
  I2S0_MCR = I2S_MCR_MICS(0) | I2S_MCR_MOE;
  // Divide down the system clock to get the MCLK
  I2S0_MDR = I2S_MDR_FRACT(MCLK_FRACT) | I2S_MDR_DIVIDE(MCLK_DIVIDE);

  // CONFIGURE TRANSMIT
  // No mask - we are sending both channels of audio
  I2S0_TMR = 0;
  // Set the high water mark for the FIFO.  This determines when the interupt
  // needs to fire.
  I2S0_TCR1 = I2S_TCR1_TFW(4);
  // Asynchronous mode | Bit Clock Active Low | Master Clock 1 Selected |
  // Bit Clock generated internally (master) | Bit Clock Divide
  // Setting DIV=1 means (1+1)*2=4 division of MCLK->BCLK
  I2S0_TCR2 = I2S_TCR2_SYNC(0) | I2S_TCR2_BCP | I2S_TCR2_MSEL(1) |
     I2S_TCR2_BCD | I2S_TCR2_DIV(1);
  // Transmit channel 0 is enabled | Start of word flag = 0
  I2S0_TCR3 = I2S_TCR3_TCE | I2S_TCR3_WDFL(0);
  // Frame size = 2 | Sync width = 24 bit clocks | MSB first |
  // Frame sync asserts one bit before the first bit of the frame |
  // Frame sync is active low | Frame sync is generated internally
  I2S0_TCR4 = I2S_TCR4_FRSZ(1) | I2S_TCR4_SYWD(23) | I2S_TCR4_MF |
    I2S_TCR4_FSE | 
    I2S_TCR4_FSD;
  // Word N Width = 24 | Word 0 Width = 24 | First Bit Shifted = 24
  I2S0_TCR5 = I2S_TCR5_WNW(23) | I2S_TCR5_W0W(23) | I2S_TCR5_FBT(23);

  // CONFIGURE RECEIVE
  // No mask - we are sending both channels of audio
  I2S0_RMR = 0;
  // Set the high water mark for the FIFO.  This determines when the interupt
  // needs to fire.
  I2S0_RCR1 = I2S_RCR1_RFW(4);
  // Synchronous with transmitter | Bit Clock Active Low | Master Clock 1 Selected |
  // Bit Clock generated internally (master) | Bit Clock Divide
  // Setting DIV=1 means (1+1)*2=4 division of MCLK->BCLK
  I2S0_RCR2 = I2S_RCR2_SYNC(1) | I2S_RCR2_BCP | I2S_RCR2_MSEL(1) |
     I2S_RCR2_BCD | I2S_RCR2_DIV(1);
  // Receive channel 0 is enabled | Start of word flag = 0
  I2S0_RCR3 = I2S_RCR3_RCE | I2S_RCR3_WDFL(0);
  // Frame size = 2 | Sync width = 24 bit clocks | MSB first |
  // Frame sync asserts one bit before the first bit of the frame |
  // Frame sync is active low | Frame sync is generated internally
  I2S0_RCR4 = I2S_RCR4_FRSZ(1) | I2S_RCR4_SYWD(23) | I2S_RCR4_MF |
    I2S_RCR4_FSE | I2S_RCR4_FSD;
  // Word N Width = 24 | Word 0 Width = 24 | First Bit Shifted = 24
  I2S0_RCR5 = I2S_RCR5_WNW(23) | I2S_RCR5_W0W(23) | I2S_RCR5_FBT(23);

  // Enable the system-level interrupt for I2S
  NVIC_ENABLE_IRQ(IRQ_I2S0_TX);
  NVIC_ENABLE_IRQ(IRQ_I2S0_RX);

  // Software reset
  I2S0_TCSR = I2S_TCSR_SR;
  I2S0_RCSR = I2S_RCSR_SR;

  // Receive enabled | FIFO request interrupt enable
  I2S0_RCSR = I2S_RCSR_RE | I2S_RCSR_FRIE;

  // Transmit enabled | Bit clock enabled | FIFO request interrupt enable
  I2S0_TCSR = I2S_TCSR_TE | I2S_TCSR_BCE | I2S_TCSR_FRIE;
}

/*
 * Utility function that looks at the read/write pointers on the 
 * FIFO to determine if it is completely full yet.  The FIFO
 * pointers are 3-bit values.
 */
bool isTXFIFOFull() {
    // Read the FIFO pointers
    uint32_t v = I2S0_TFR0;
    uint32_t wfp = (v & 0b11110000000000000000) >> 16;
    uint32_t rfp = (v & 0b1111);
    // Full happens when the pointers are the same except for the MSB.
    if ((wfp & 0b111) == (rfp & 0b111) && wfp != rfp) {
      return true;
    } else {
      return false;
    }
}

/*
 * Utility function that looks at the read/write pointers on the 
 * FIFO to determine if it is completely full yet.  The FIFO
 * pointers are 3-bit values.
 */
bool isRXFIFOEmpty() {
    // Read the FIFO pointers
    uint32_t v = I2S0_RFR0;
    uint32_t wfp = (v & 0b11110000000000000000) >> 16;
    uint32_t rfp = (v & 0b1111);
    // Empty happens when the pointers are the same.
    if (wfp == rfp) {
      return true;
    } else {
      return false;
    }
}
/*            
              INV       (INV + 1) & 0b111
              ---       -----------------             
  000   0     111 + 1 = 000 ->  0
  001   1     110 + 1 = 111 -> -1
  010   2     101 + 1 = 110 -> -2 
  011   3     100 + 1 = 101 -> -3
  100  -4     011 + 1 = 100 -> -4
  101  -3     010 + 1 = 011 ->  3
  110  -2     001 + 1 = 010 ->  2
  111  -1     000 + 1 = 001 ->  1
*/
CircularBuffer<long,16> leftBuf;
CircularBuffer<long,16> rightBuf;

volatile bool isRightRead = false;
volatile bool isRightWrite = false;
volatile int maxSize = 0;

/**
 * This function writes as much as possible into the FIFO.
 */
void tryWrite() {
  // Loop
  while (!isTXFIFOFull()) {
    if (!isRightWrite) {
      if (!leftBuf.isEmpty()) {
        if (leftBuf.size() > maxSize) {
          maxSize = leftBuf.size();
        }
        I2S0_TDR0 = leftBuf.pop() & 0x00ffffff;
      }
      else {
        I2S0_TDR0 = 0;
      }
      isRightWrite = true;
    } else {
      if (!rightBuf.isEmpty()) {
        if (rightBuf.size() > maxSize) {
          maxSize = rightBuf.size();
        }
        I2S0_TDR0 = rightBuf.pop() & 0x00ffffff;
      } else {
        I2S0_TDR0 = 0;        
      }
      isRightWrite = false;
    }
  }
}

/**
 * This function writes as much as possible into the FIFO.
 */
void tryRead() {
  // Loop
  while (!isRXFIFOEmpty()) {
    // Read a value.  This automatically advances the read pointer
    long v = I2S0_RDR0;  
    // Sign extend the 24-bit value
    if (v & 0x00800000) {
      v |= 0xff000000;
    }
    if (!isRightRead) {
      leftBuf.push(v);
      isRightRead = true;
    } else {
      rightBuf.push(v);
      isRightRead = false;
    }
  }
}

// This gets called whenever the FIFO interupt is raised
void i2s0_tx_isr(void) {
  cli();
  // Replenish the FIFO as quickly as possible
  tryWrite();
  sei();
}

// This gets called whenever the FIFO interupt is raised
void i2s0_rx_isr(void) {
  cli();
  // Empty the FIFO as quickly as possible
  tryRead();
  sei();
}

long lastDisplay = 0;

void loop() {

  if (millis() - lastDisplay > 2000) {
    lastDisplay = millis();
    //Serial.print(leftBuf.pop());
    //Serial.print(" ");
    //Serial.println(rightBuf.pop());
    Serial.println(maxSize);
  }
}
