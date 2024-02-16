#ifndef RESET_HPP
#define RESET_HPP

extern bool reset_requested;

inline void requestReset()
{
  reset_requested = true;
}

#endif // #ifndef RESET_HPP