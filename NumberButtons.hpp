#ifndef NUMBER_BUTTONS_H
#define NUMBER_BUTTONS_H

#ifndef NUMBER_POLL_INTERVAL
#define NUMBER_POLL_INTERVAL 100
#endif // NUMBER_POLL_INTERVAL
#ifndef NUMBER_SCROLL_START
#define NUMBER_SCROLL_START NUMBER_POLL_INTERVAL * 5
#endif // NUMBER_SCROLL_START
#ifndef NUMBER_WRITE_DELAY
#define NUMBER_WRITE_DELAY NUMBER_POLL_INTERVAL * 10
#endif // NUMBER_WRITE_INTERVAL
#ifndef NUMBER_EEPROM_OFFSET
#define NUMBER_EEPROM_OFFSET 0
#endif // NUMBER_EEPROM_OFFSET

#define NUMBER_CHANGED 1
#define NUMBER_READY 2

#include <Arduino.h>
#include <inttypes.h>
#include <EEPROM.h>

template<typename _Tp>
struct is_signed
{
  static bool const value = _Tp(-1) < _Tp(0);
};

template <typename T>
class NumberButtons{
private:
  enum FLAGS {
    SCROLLING = 0x1,
    WRITE = 0x2
  };
  uint8_t flags;
  uint16_t check_time;
  uint16_t scroll_write_time;
public:
  uint8_t pins[2];
  T value;
  T limit;
  
  NumberButtons (uint8_t add_pin, uint8_t sub_pin, T sym_limit, T startvalue = 0)
  : pins{add_pin, sub_pin}, limit{sym_limit}, value{startvalue}
  , flags{0}, check_time{0} {};
  
  uint8_t tick (uint16_t time) {
    if (time - check_time >= 0x8000) {
      return 0;
    }
    check_time += NUMBER_POLL_INTERVAL;
    bool add = !digitalRead(pins[0]);
    if (add == !digitalRead(pins[1])) { // If both or none buttons are pressed
      if (flags & NumberButtons<T>::SCROLLING) {
        scroll_write_time = time + NUMBER_WRITE_DELAY;
        flags &= ~NumberButtons<T>::SCROLLING;
      }
      if ((flags & NumberButtons<T>::WRITE) == 0 || time - scroll_write_time >= 0x8000)
        return 0;

      flags &= ~NumberButtons<T>::WRITE;
      return NUMBER_READY;
    }
    if ((flags & NumberButtons<T>::SCROLLING) && time - scroll_write_time >= 0x8000)
      return 0;
    value += (add) ? 1 : -1;
    if (value > limit)
      value = (is_signed<T>::value) ? -limit : 0;
    if (is_signed<T>::value && value < -limit)
      value = limit;
    if ((flags & NumberButtons<T>::SCROLLING) == 0) {
      scroll_write_time = time + NUMBER_SCROLL_START;
    }
    flags |= NumberButtons<T>::SCROLLING | NumberButtons<T>::WRITE;
    return NUMBER_CHANGED;
  }
  inline void tick () {
    tick(millis());
  };
};
#endif // NUMBER_BUTTONS_H