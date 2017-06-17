/*
 * Teensy 3.2 header for UAVCAN
 * @author fwindolf - Florian Windolf  florianwindolf@gmail.com
 */

#include <uavcan_teensy32/clock.hpp>

namespace uavcan_teensy32
{
namespace clock
{
namespace
{

bool initialized = false;
bool utc_set = false;

elapsedMicros usec; // automatic rollover handling

std::int64_t prev_adjustment = 0;
std::int64_t utc_correction = 0;

#ifdef __GNUC__
__attribute__((noreturn))
#endif
static void fail()
{
  while(true) {}
}

} // nameless namespace


void init()
{
  if(!initialized)
  {
    initialized = true;
    usec = 0;
  }
}

uavcan::MonotonicTime getMonotonic()
{
  if(!initialized)
  {
    fail();
  }
  // Calculate the milliseconds
  return uavcan::MonotonicTime::fromUSec(usec);
}

uavcan::UtcTime getUtc()
{
  if(!initialized)
  {
    fail();
  }

  // Only return the time if time was adjusted
  std::uint64_t _usec = 0;
  if(utc_set)
  {
    _usec = usec;
  }
  return uavcan::UtcTime::fromUSec(_usec);
}

void adjustUtc(uavcan::UtcDuration adjustment)
{
  const std::int64_t adj_delta = adjustment.toUSec() - prev_adjustment;
  prev_adjustment = adjustment.toUSec();

  // On first time
  if(!utc_set)
  {
    utc_set = true;
    utc_correction = 0;
  }

  // TODO: find better implementation for compensating clock speed, for example
  // add x ticks at every timer overflow and handle elapsed time with micros()
  // plus overflowing in the correct ISR
  usec += adjustment.toUSec();
}

} // clock

// Init static variable
SystemClock SystemClock::self;

SystemClock& SystemClock::instance()
{
  clock::init();
  return self;
}

} // uavcan_teensy32
