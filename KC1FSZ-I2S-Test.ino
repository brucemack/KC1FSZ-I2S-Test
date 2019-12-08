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
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

//AudioControlSGTL5000  sgtl5000_1;

// This is the largest magnitude that can be represented by the DAC
const float FullScale = (2 ^ 31) - 1;

// =================================================================================
// Sine Look Up Table 
//
// The LUT only has to cover 90 degrees of the phase range because 
// we have quadrant translation.
const unsigned int LutSize = 64;
// This is the number of phase buckets we manage.  More buckets means 
// a smoother transition through the baseband signals.
const unsigned int PhaseRange = LutSize * 4;
// This is the LUT (filled during setup())
float SinLut[LutSize];

void BuildLut() {
  // Build the look-up table.  This will only cover the first 90 
  // degrees of the sin() function.
  for (unsigned int i = 0; i < LutSize; i++) {
    float rad = ((float)i / (float)PhaseRange) * 2.0 * 3.1415926;
    SinLut[i] = sin(rad);
  }  
}

// This function takes an integer phase and returns the sin() value.  Quadrant
// translation is used to save memory.
//
// ph - Integer phase from 0 to PhaseRange - 1
// returns a float value from -1.0 to 1.0
//
float SineWithQuadrant(unsigned int ph) {
  // Figure out which quandrant we're in and adjust accordingly
  unsigned int quadrant = ph / LutSize;
  // There are some special cases here
  if (ph == LutSize) {
    return 1.0;
  } else if (ph == LutSize * 3) {
    return -1;
  } else {
    if (quadrant == 0) {
      return SinLut[ph];
    } else if (quadrant == 1) {
      return SinLut[PhaseRange / 2 - ph];
    } else if (quadrant == 2) {
      return -SinLut[ph - PhaseRange / 2];
    } else {
      return -SinLut[PhaseRange - ph];
    }
  }
}
// =================================================================================

const unsigned int SampleFreqHz = 44100;
unsigned int PhaseCounter = 0;

/**
 * Converts a counter value (incremeting at the sample freqency) and a target
 * frequency and produces a phase value from 0->PhaseRange-1.
 */
unsigned int GetPhase(unsigned int counter,unsigned int freqHz) {
  // We multiply first to maintain precision
  unsigned int i = (counter * freqHz * PhaseRange) / SampleFreqHz;
  // Check for wraps
  while (i >= PhaseRange) {
    i -= PhaseRange;
  }
  return i;
}

// Clocking calculation for 8K (AK4556)
//
//   96 MHz system clock
//   9/423 multiply/divide
//   2,042,533 Hz MCLK
//   DIV=1 -> divide by (1+1)*2=4
//   510,638 Hz BCLK
//   There are 24x2 bits in a frame (L+R)
//   Fs is 7,978 Hz 
//
//#define MCLK_MULT 9
//#define MCLK_DIVIDE 423
//

// Clocking calculations for 44.1K using 96 Mhz system clock
//
// MCLK needs to 256 * 44.100 kHz sample rate = 11.2896 MHz
// This is given by (CPU frequency * MCLK_MULT) / MCLK_DIV
// MCLK_MULT is a positive integer with range 1-256 (8 bits)
// MCLK_DIV is a positive integer with range 1-4096 (12 bits)
// MCLK_MULT must be <= MCLK_DIV
//
#define MCLK_MULT 2
#define MCLK_DIVIDE 17

// This is the location for DMA transmit operations
const int tx_buffer_size = 128;
// (From PJS) "DMAMEM is not required. It only serves to place your buffers lower in memory.
// The idea is typical programs will do most of their ordinary memory access to the stack, 
// located in the upper memory. If your buffers are in lower memory, odds are (maybe) less 
// of the memory controller adding a wait state if the CPU and DMA want to access the same 
// region of memory in the same clock cycle.
DMAMEM __attribute__((aligned(32))) static uint32_t i2s_tx_buffer[tx_buffer_size];

// This structure matches the layout of the K20 DMA Transfer Control Descriptor
//
// Address: 4000_8000h base + 1000h offset + (32d × i), where i=0d to 15d
//
struct __attribute__((packed, aligned(4))) TCD {    
  // Source Address 
  volatile const void * volatile SADDR;
  // Signed Source Address Offset - Sign-extended offset applied to the current source 
  // address to form the next-state value as each source read is completed.
  int16_t SOFF;
  // Transfer Attributes
  uint16_t ATTR;  
  // TCD word 2's register definition depends on the status of minor loop mapping. 
  union { 
    // (DISALBLED) Number of bytes to be transferred in each service request of the channel. 
    uint32_t NBYTES; 
    // (ENABLED) SMLOE/DMLOE/Minor Byte Transfer Count
    uint32_t NBYTES_MLNO;
    uint32_t NBYTES_MLOFFNO; 
    uint32_t NBYTES_MLOFFYES; 
  };
  // Adjustment value added to the source address at the completion of the major iteration count. 
  int32_t SLAST;
  // Destination address
  volatile void * volatile DADDR;
  // Signed Source Address Offset - Sign-extended offset applied to the current destination
  // address to form the next-state value as each source read is completed.  
  int16_t DOFF;
  // Current channel linking feature/major iteration count
  union { 
    volatile uint16_t CITER;
    volatile uint16_t CITER_ELINKYES; 
    volatile uint16_t CITER_ELINKNO; 
  };
  // Destination last address adjustment or the memory address for the next transfer control 
  // descriptor to be loaded into this channel (scatter/gather).
  int32_t DLASTSGA;
  // Control and Status
  volatile uint16_t CSR;
  //  Beginning Minor Loop Link
  union { 
    volatile uint16_t BITER;
    volatile uint16_t BITER_ELINKYES; 
    volatile uint16_t BITER_ELINKNO; 
  };
};

volatile long v = 0;

// Interrupt service routine from DMA controller
void dma_isr_function(void) {  

  v++;
  
  // The INT register provides a bit map for the 16 channels signaling the presence of an
  // interrupt request for each channel. Depending on the appropriate bit setting in the
  // transfer-control descriptors, the eDMA engine generates an interrupt on data transfer
  // completion. The outputs of this register are directly routed to the interrupt controller
  // (INTC). During the interrupt-service routine associated with any given channel, it is the
  // software’s responsibility to clear the appropriate bit, negating the interrupt request.
  // Typically, a write to the CINT register in the interrupt service routine is used for this
  // purpose.
  
  // Clear interrupt request for channel 0
  DMA_CINT = 0;
}

void setup() {

  Serial.begin(115200);
  delay(3000);
  Serial.println("KC1FSZ");

  //BuildLut();

  // Fill the transmit area
  for (int i = 0; i < tx_buffer_size; i++) {
    i2s_tx_buffer[i] = (i * 2 + 1) << 16 | i * 2;
  }

  //AudioMemory(10);

  //sgtl5000_1.enable();
  //sgtl5000_1.volume(1.0);

  // Configure the Teensy 3.2 pins per the reference and wiring
  PORTC_PCR3 = PORT_PCR_MUX(6); // Alt 6 is BLCK - T3.2 pin 9
  PORTC_PCR6 = PORT_PCR_MUX(6); // Alt 6 is MCLK - T3.2 pin 11
  PORTC_PCR1 = PORT_PCR_MUX(6); // Alt 6 is TXD0 - T3.2 pin 22
  PORTC_PCR2 = PORT_PCR_MUX(6); // Alt 6 is LRCLK - T3.2 pin 23
  PORTC_PCR5 = PORT_PCR_MUX(4); // Alt 4 is RXD0 - T3.2 pin 13

  // ----- Enable DMA ----------------------------------------------------
  //
  // Enable clocks for DMA Mux and eDMA 
  SIM_SCGC7 |= SIM_SCGC7_DMA;
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX;  

  int channel = 0;

  __enable_irq();
  
  DMA_CR = DMA_CR_EMLM | DMA_CR_EDBG; // minor loop mapping is available
  DMA_CERQ = channel;
  DMA_CERR = channel;
  DMA_CEEI = channel;
  DMA_CINT = channel;

  struct TCD* tcd = 0x40009000 + (32 * channel); 
  // Clear 
  uint32_t *p = (uint32_t *)tcd;
  *p++ = 0;
  *p++ = 0;
  *p++ = 0;
  *p++ = 0;
  *p++ = 0;
  *p++ = 0;
  *p++ = 0;
  *p++ = 0;

  // ----- Configure I2S ---------------------------------------------------------

  // System gating control register.  I2S clock gate control is enabled.
  SIM_SCGC6 |= SIM_SCGC6_I2S;
  
  // The MCLK is sourced from the system clock(0) or PLL(3) | The MCLK is enabled
  // QUESTION: IT LOOKS lIKE THE PJS CODE USES PLL >20MHz
  I2S0_MCR = I2S_MCR_MICS(3) | I2S_MCR_MOE;
  // Loop waiting for the divisor update to complete
  while (I2S0_MCR & I2S_MCR_DUF) ;
  // Divide down the system clock to get the MCLK
  I2S0_MDR = I2S_MDR_FRACT(MCLK_MULT-1) | I2S_MDR_DIVIDE(MCLK_DIVIDE-1);

  // CONFIGURE TRANSMIT
  // No mask - we are sending both channels of audio
  I2S0_TMR = 0;
  // Set the high water mark for the FIFO.  This determines when the interupt
  // needs to fire.
  I2S0_TCR1 = I2S_TCR1_TFW(1);
  // Asynchronous mode | Bit Clock Active Low | Master Clock 1 Selected |
  // Bit Clock generated internally (master) | Bit Clock Divide
  // Setting DIV=1 means (1+1)*2=4 division of MCLK->BCLK
  I2S0_TCR2 = I2S_TCR2_SYNC(0) | I2S_TCR2_BCP | I2S_TCR2_MSEL(1) |
     I2S_TCR2_BCD | I2S_TCR2_DIV(1);
  // Transmit channel 0 is enabled | Start of word flag = 0
  I2S0_TCR3 = I2S_TCR3_TCE | I2S_TCR3_WDFL(0);
  // Frame size = 2 | Sync width = 32 bit clocks | MSB first |
  // Frame sync asserts one bit before the first bit of the frame |
  // Frame sync is active low | Frame sync is generated internally (master)
  I2S0_TCR4 = I2S_TCR4_FRSZ(1) | I2S_TCR4_SYWD(31) | I2S_TCR4_MF |
    I2S_TCR4_FSE | I2S_TCR4_FSP | I2S_TCR4_FSD;
  // Word N Width = 32 | Word 0 Width = 32 | First Bit Shifted = 32
  I2S0_TCR5 = I2S_TCR5_WNW(31) | I2S_TCR5_W0W(31) | I2S_TCR5_FBT(31);

  // CONFIGURE RECEIVE
  // No mask - we are sending both channels of audio
  I2S0_RMR = 0;
  // Set the high water mark for the FIFO.  This determines when the interupt
  // needs to fire.
  I2S0_RCR1 = I2S_RCR1_RFW(1);
  // Synchronous with transmitter | Bit Clock Active Low | Master Clock 1 Selected |
  // Bit Clock generated internally (master) | Bit Clock Divide
  // Setting DIV=1 means (1+1)*2=4 division of MCLK->BCLK
  I2S0_RCR2 = I2S_RCR2_SYNC(1) | I2S_RCR2_BCP | I2S_RCR2_MSEL(1) |
     I2S_RCR2_BCD | I2S_RCR2_DIV(1);
  // Receive channel 0 is enabled | Start of word flag = 0
  I2S0_RCR3 = I2S_RCR3_RCE | I2S_RCR3_WDFL(0);
  // Frame size = 2 | Sync width = 32 bit clocks | MSB first |
  // Frame sync asserts one bit before the first bit of the frame |
  // Frame sync is active low | Frame sync is generated internally
  I2S0_RCR4 = I2S_RCR4_FRSZ(1) | I2S_RCR4_SYWD(31) | I2S_RCR4_MF |
    I2S_RCR4_FSE | I2S_RCR4_FSP | I2S_RCR4_FSD;
  // Word N Width = 32 | Word 0 Width = 32 | First Bit Shifted = 32
  I2S0_RCR5 = I2S_RCR5_WNW(31) | I2S_RCR5_W0W(31) | I2S_RCR5_FBT(31);

  // ----- Configure DMA -------------------------------------------------------

  // Configure the source source buffer
  tcd->SADDR = i2s_tx_buffer;
  // Amount that the source address is adjusted after each read.  
  // Note that we are handling 16-bit data so this value is 2 bytes.
  tcd->SOFF = 2;
  // Source and destination modulo disabled.  Source and destination transfer
  // sizes are 16 bits.
  tcd->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
  // Minor loop mapping is disabled.  Minor byte transfer count.
  tcd->NBYTES_MLNO = 2;
  // Adjustment value added to the source address at the completion of the major iteration count. 
  // Moving things back to the initial value.
  tcd->SLAST = -sizeof(i2s_tx_buffer);
  // Destination - High side of 32 bit register
  tcd->DADDR = (void*)((uint32_t)&I2S0_TDR0 + 2);
  // Sign-extended offset applied to the current destination address to form the next-state 
  // value as each destination write is completed. No movement needed here.
  tcd->DOFF = 0;
  // Channel linking is disabled, current major iteration count. This is half because
  // each tranfer moves two byes.
  tcd->CITER_ELINKNO = sizeof(i2s_tx_buffer) / 2;
  // Scatter/gather not used
  tcd->DLASTSGA = 0;
  // Channel linking is disabled, begin major iteration count
  tcd->BITER_ELINKNO = sizeof(i2s_tx_buffer) / 2;
  // Generate interrupt when major counter is half done.  Specifically, the comparison
  // performed by the eDMA engine is (CITER == (BITER >> 1)). 
  // Generate interrupt when major counter completes
  tcd->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;

  // Get the mux setup 
  uint8_t source = DMAMUX_SOURCE_I2S0_TX;
  volatile uint8_t* mux = (volatile uint8_t*)&(DMAMUX0_CHCFG0) + channel;
  *mux = 0;
  *mux = (source & 63) | DMAMUX_ENABLE;

  // Install the interrupt handler
  //
  // _VectorsRam is created by the Teensy library and it has been initialized
  // using the flash vector table. 
  //
  // void (* _VectorsRam[NVIC_NUM_INTERRUPTS+16])(void);
  //
  // This is the code that maps the RAM table:
  //
  // SCB_VTOR = (uint32_t)_VectorsRam; // use vector table in RAM
  //
  // There are 16 extra entries in the table before the normal IRQ entries
  //
  _VectorsRam[16 + IRQ_DMA_CH0 + channel] = dma_isr_function;
  NVIC_ENABLE_IRQ(IRQ_DMA_CH0 + channel);

  // Enable the system-level interrupt for I2S
  //NVIC_ENABLE_IRQ(IRQ_I2S0_TX);
  //NVIC_ENABLE_IRQ(IRQ_I2S0_RX);

  // ------ Final Enable -------------------------------------------------------
  
  // Set Enable Request Register (DMA_SERQ). Enable DMA for the specified channel 
  DMA_SERQ = channel;

  // Software reset
  I2S0_TCSR = I2S_TCSR_SR;
  //I2S0_RCSR = I2S_RCSR_SR;

  // Transmit enabled | Bit clock enabled | FIFO request DMA enable
  I2S0_TCSR = I2S_TCSR_TE | I2S_TCSR_BCE | I2S_TCSR_FRDE;
  // Receive enabled | FIFO request DMA enable
  //I2S0_RCSR = I2S_RCSR_RE | I2S_RCSR_FRDE;

  Serial.println("setup() done");
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
volatile bool isRightRead = false;
volatile bool isRightWrite = false;
volatile int maxSize = 0;
volatile int counter = 0;

volatile int LastR = 0;
volatile float LastS = 0;
volatile float LastT = 0;
volatile int LastU = 0;

/**
 * This function writes as much as possible into the FIFO.
 */
void tryWrite() {
  // Loop
  while (!isTXFIFOFull()) {
    if (!isRightWrite) {
      PhaseCounter++;
      // Create the phase pointer based on the target frequency
      unsigned int ph = GetPhase(PhaseCounter,2000);
      LastR = ph;
      // Convert the pointer to sin(phase)
      //LastS = SineWithQuadrant(ph);
      /*
      // Scale up to the DAC range (signed)
      LastT = LastS * FullScale;
      // Convert to integer (signed)
      LastU = (int)LastT;
      I2S0_TDR0 = (int)LastU;
      */
      counter += (1024 * 1024);
      I2S0_TDR0 = counter; 
      isRightWrite = true;
    } else {
      counter += (1024 * 1024);
      I2S0_TDR0 = counter; 
      isRightWrite = false;
    }
  }
}

/**
 * This function reads as much as possible from the FIFO.
 */
void tryRead() {
  // Loop
  while (!isRXFIFOEmpty()) {
    // Read a value.  This automatically advances the read pointer
    long v = I2S0_RDR0;  
    if (!isRightRead) {
      isRightRead = true;
    } else {
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

volatile long lastDisplay = 0;

void loop() {
  if (millis() - lastDisplay > 1000) {
    lastDisplay = millis();
    Serial.println(v);
  }
}
