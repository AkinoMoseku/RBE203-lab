// Project-local compatibility fix for Arduino AVR's weak operator-new references.
// PinChangeInt allocates its first encoder entry with new. With the affected
// core, a weak reference can leave core.a's allocation implementation unlinked,
// turning that allocation into a call to address zero.
//
// Arduino IDE compiles this sketch-local .cpp directly, providing strong
// definitions without changing any course library or requiring linker flags.
// Do not include Arduino.h, new.h or <new> here: their weak declarations would
// also make these definitions weak. Match the AVR core's allocation semantics.
#include <stddef.h>
#include <stdlib.h>

#if defined(ARDUINO_ARCH_AVR)
void *operator new(size_t size)
{
  return malloc(size ? size : 1);
}

void *operator new[](size_t size)
{
  return malloc(size ? size : 1);
}

void operator delete(void *ptr) noexcept
{
  free(ptr);
}

void operator delete[](void *ptr) noexcept
{
  free(ptr);
}
#endif
