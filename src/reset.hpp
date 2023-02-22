#ifndef RESET_HPP
#define RESET_HPP

extern bool reset_requested;

inline void requestReset() {
  reset_requested = true;
}

#endif  // End of include guard for RESET_HPP