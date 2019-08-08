#pragma once

#include <diagnostic_updater/DiagnosticStatusWrapper.h>

namespace mitre
{
  namespace KVH
  {
    class DiagnosticsContainer
    {
    public:
      DiagnosticsContainer();

      ///////////////////////////////////////////////////////////////////////
      //All of these functions interpret and store data from messages.
      //They are NOT the functions called by the DiagnosticUpdater callbacks.
      ///////////////////////////////////////////////////////////////////////
      void SetSystemStatus(uint16_t _status) {systemStatus_ = _status; receivedSystemStatus_ = true;}
      void SetFilterStatus(uint16_t _status) {filterStatus_ = _status; receivedFilterStatus_ = true;}

      ///////////////////////////////////////////////////////////////////////
      // Callback functions for different diagnostics
      // Each of these may aggregate data from multiple systems
      ///////////////////////////////////////////////////////////////////////
      void UpdateSystemStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
      void UpdateFilterStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

    private:
      uint16_t systemStatus_;
      bool receivedSystemStatus_;
      uint16_t filterStatus_;
      bool receivedFilterStatus_;
    }; //end: class DiagnosticsContainer
  } //end: namespace KVH
} //end: namespace mitre
