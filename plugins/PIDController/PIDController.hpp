// PluginPIDController.hpp
// Jordan Henderson (j.henderson.music@outlook.com)

#pragma once

#include "SC_PlugIn.hpp"

namespace PIDController {

class PIDController : public SCUnit {
public:
    PIDController();

    // Destructor
    // ~PIDController();

private:
    // Calc function
    void next(int nSamples);

    // Member variables
};

} // namespace PIDController
