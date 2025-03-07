/**
 * @File: diagnostics_wrapper.cpp
 * @Date: October 2022
 * @Author: James Swedeen
 **/

/* Local Headers */
#include<sim_kalman_filter/diagnostics_wrapper.hpp>

namespace skf
{
/**
 * @pubLoop
 *
 * @brief
 * Loop that periodically publishes the diagnostic status.
 **/
void DiagnosticsWrapper::pubLoop()
{
  std::lock_guard<std::mutex> lock(this->pub_mux);
  // Publish the internally held diagnostic message
  this->diag_pub->publish(this->output_msg);
}
} // skf

/* diagnostics_wrapper.cpp */
