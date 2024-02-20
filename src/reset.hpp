#ifndef RESET_HPP
#define RESET_HPP

extern bool reset_requested;

/**
 * @brief Triggers topside connection reset on next main loop cycle
 *
 */
inline void requestReset()
{
  // Sets reset_requested to true
  reset_requested = true;
}

#endif // #ifndef RESET_HPP