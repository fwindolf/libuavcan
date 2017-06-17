/*
 * Teensy 3.2 header for UAVCAN
 * @author fwindolf - Florian Windolf  florianwindolf@gmail.com
 */

#include <Arduino.h>
#include <uavcan_teensy32/can.hpp>
#include <uavcan_teensy32/clock.hpp>
#include "kinetis_flexcan.h"

using namespace uavcan;

namespace uavcan_teensy32
{

// wait until the address bit is set in flexcan register
void wait_for(auto address)
{
  while(FLEXCANb_MCR(FLEXCAN0_BASE) & address)
  {
    ; // do nothing
  }
}


#define FLEXCANb_MCR(b)                   (*(vuint32_t*)(b))
#define FLEXCANb_CTRL1(b)                 (*(vuint32_t*)(b+4))
#define FLEXCANb_RXMGMASK(b)              (*(vuint32_t*)(b+0x10))
#define FLEXCANb_IFLAG1(b)                (*(vuint32_t*)(b+0x30))
#define FLEXCANb_RXFGMASK(b)              (*(vuint32_t*)(b+0x48))
#define FLEXCANb_MBn_CS(b, n)             (*(vuint32_t*)(b+0x80+n*0x10))
#define FLEXCANb_MBn_ID(b, n)             (*(vuint32_t*)(b+0x84+n*0x10))
#define FLEXCANb_MBn_WORD0(b, n)          (*(vuint32_t*)(b+0x88+n*0x10))
#define FLEXCANb_MBn_WORD1(b, n)          (*(vuint32_t*)(b+0x8C+n*0x10))
#define FLEXCANb_IDFLT_TAB(b, n)          (*(vuint32_t*)(b+0xE0+(n*4)))

// Buffers before first are occupied by FIFO
#define TX_BUFFER_FIRST                   8
#define TX_BUFFER_COUNT                   8
#define RX_BUFFER_FIRST                   0

CanDriver::CanDriver()
{
  // setup pins
  CORE_PIN3_CONFIG = PORT_PCR_MUX(2);
  CORE_PIN4_CONFIG = PORT_PCR_MUX(2);

  // select clock source
  SIM_SCGC6 |= SIM_SCGC6_FLEXCAN0;
  FLEXCANb_CTRL1(FLEXCAN0_BASE) &= ~FLEXCAN_CTRL_CLK_SRC;

  // enable CAN
  FLEXCANb_MCR(FLEXCAN0_BASE) |= FLEXCAN_MCR_FRZ;
  FLEXCANb_MCR(FLEXCAN0_BASE) &= ~FLEXCAN_MCR_MDIS;
  wait_for(FLEXCAN_MCR_MDIS);

  // do soft reset
  FLEXCANb_MCR(FLEXCAN0_BASE) ^= FLEXCAN_MCR_SOFT_RST;
  wait_for(FLEXCAN_MCR_SOFT_RST);

  wait_for(FLEXCAN_MCR_FRZ_ACK);

  // disable self-reception
  FLEXCANb_MCR(FLEXCAN0_BASE) |= FLEXCAN_MCR_SRX_DIS;

  // enable RX FIFO
  FLEXCANb_MCR(FLEXCAN0_BASE) |= FLEXCAN_MCR_FEN;
}


// TODO: provide implementation
uint32_t CanDriver::detectBitRate(void (*idle_callback)() = nullptr)
{
  return 500000;
}


int CanDriver::init(uint32_t bitrate, uint32t id)
{
  switch(baudrate)
  {
    case 50000:
      FLEXCANb_CTRL1(FLEXCAN0_BASE) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3)
                                      | FLEXCAN_CTRL_PRESDIV(19));
      break;
    case 100000:
      FLEXCANb_CTRL1(FLEXCAN0_BASE) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3)
                                      | FLEXCAN_CTRL_PRESDIV(9));
      break;
    case 250000:
      FLEXCANb_CTRL1(FLEXCAN0_BASE) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3)
                                      | FLEXCAN_CTRL_PRESDIV(3));
      break;
    case 500000:
      FLEXCANb_CTRL1(FLEXCAN0_BASE) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3)
                                      | FLEXCAN_CTRL_PRESDIV(1));
      break;
    case 1000000:
      FLEXCANb_CTRL1(FLEXCAN0_BASE) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(0)
                                      | FLEXCAN_CTRL_PSEG1(1) | FLEXCAN_CTRL_PSEG2(1)
                                      | FLEXCAN_CTRL_PRESDIV(1));
      break;
    default: // 125000
      FLEXCANb_CTRL1(FLEXCAN0_BASE) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                    | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3)
                                    | FLEXCAN_CTRL_PRESDIV(7));
  }

  // set default filter mask
  FLEXCANb_RXMGMASK(FLEXCAN0_BASE) = 0;

  // start the CAN
  FLECANb_MCR(FLEXCAN0_BASE) &= ~(FLEXCAN_MCR_HALT);
  wait_for(FLEXCAN_MCR_FRZ_ACK);
  wait_for(FLEXCAN_MCR_NOT_RDY);

  // activate tx buffers
  for(int i = TX_BUFFER_FIRST; i < TX_BUFFER_FIRST + TX_BUFFER_COUNT; i++)
  {
    FLEXCANb_MBn_CS(FLEXCAN0_BASE, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  }
}

int16_t CanDriver::int16_t send(const CanFrame& frame, MonotonicTime tx_deadline, CanIOFlags flags)
{
  // Frame was not transmitted until tx deadline
  if(clock::getMonotonic() - tx_deadline <= 0)
  {
    return -1;
  }

  // Search for available buffer
  int buffer = -1;
  for(int i = TX_BUFFER_FIRST; ;)
  {
    // Check if this buffer is available
    if((FLEXCANb_MBn_CS(FLEXCAN0_BASE, i) & FLEXCAN_MB_CS_CODE_MASK)
        == FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE))
    {
      buffer = i;
      break;
    }
    // If there is no deadline for transmission, check all available buffers
    if(!tx_deadline)
    {
      // Already checked every available buffer?
      if(i > TX_BUFFER_FIRST + TX_BUFFER_COUNT)
      {
        return 0; // no more buffers to check
      }
      i++;
    }
    else
    {
      // Check if timed out
      if(clock::getMonotonic() - tx_deadline <= 0)
      {
        return -1; // too late
      }
    }
    // pass on control to other tasks
    yield();
  }

  // Transmit the frame
  FLEXCANb_MBn_CS(FLEXCAN0_BASE, buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);

  // Set header
  if(frame.isExtended())
  {
    FLEXCAN_MBn_ID(FLEXCAN0_BASE, buffer) = (frame.id & FLEXCAN_MB_ID_EXT_MASK);
  }
  else
  {
    FLEXCAN_MBn_ID(FLEXCAN0_BASE, buffer) = FLEXCAN_MB_ID_IDSTD(frame.id);
  }

  // Transfer data to the buffer
  FLEXCANb_MBn_WORD1(FLEXCAN0_BASE, buffer) = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | (frame.data[3]);
  FLEXCANb_MBn_WORD2(FLEXCAN0_BASE, buffer) = (frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | (frame.data[7]);

  // Start transmission
  if(frame.isExtended())
  {
    FLEXCANb_MBn_CS(FLEXCAN0_BASE, buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                                         | FLEXCAN_MB_CS_LENGTH(frame.dlc) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
  }
  else
  {
    FLEXCANb_MBn_CS(FLEXCAN0_BASE, buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                                         | FLEXCAN_MB_CS_LENGTH(frame.dlc);
  }
  return 1;
}


int16_t CanDriver::receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic, UtcTime& out_ts_utc,
                        CanIOFlags& out_flags)
{
  // Check if a new frame is available
  if(FLEXCANb_IFLAG1(flexcanBase) & FLEXCAN_IMASK1_BUF5M == 0)
  {
    return 0;
  }

  // save timestamp
  out_ts_monotonic = clock::getMonotonic();
  out_ts_utc = 0; // TODO: change to clock::getUtc() when properly implemented

  // get identifier and dlc
  out_frame.dlc = FLEXCAN_get_length(FLEXCANb_MBn_CS(FLEXCAN0_BASE, RX_BUFFER_FIRST));
  out_frame.ext = (FLEXCANb_MBn_CS(FLEXCAN0_BASE, RX_BUFFER_FIRST) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
  out_frame.id = (FLEXCANb_MBn_CS(FLEXCAN0_BASE, RX_BUFFER_FIRST) & FLEXCAN_MB_ID_EXT_MASK) ? 1 : 0;

  // shift id to the right position if non-extended id
  if(!out_frame.ext)
  {
    out_frame.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
  }

  // copy data to message
  uint32_t data = FLEXCANb_MBn_WORD0(FLEXCAN0_BASE, RX_BUFFER_FIRST);
  out_frame.data[3] = data;
  data >>= 8;
  out_frame.data[2] = data;
  data >>= 8;
  out_frame.data[1] = data;
  data >>= 8;
  out_frame.data[0] = data;
  // shortcut if last bytes are empty anyways
  if(out_frame.dlc  > 4)
  {
    data = FLEXCANb_MBn_WORD1(FLEXCAN0_BASE, RX_BUFFER_FIRST);
    out_frame.data[7] = data;
    data >>= 8;
    out_frame.data[6] = data;
    data >>= 8;
    out_frame.data[5] = data;
    data >>= 8;
    out_frame.data[4] = data;
  }

  // set read flags
  FLEXCANb_IFLAG1(FLEXCAN0_BASE) = FLEXCAN_IMASK1_BUF5M;

  return 1;
}
