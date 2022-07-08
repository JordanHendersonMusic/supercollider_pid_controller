// PluginPIDController.cpp
// Jordan Henderson (j.henderson.music@outlook.com)

#include "SC_PlugIn.hpp"
#include "PIDController.hpp"

static InterfaceTable* ft;

namespace PIDController {

PIDController::PIDController() {
    mCalcFunc = make_calc_function<PIDController, &PIDController::next>();
    next(1);
}

void PIDController::next(int nSamples) {
    const float* input = in(0);
    const float* gain = in(1);
    float* outbuf = out(0);

    // simple gain function
    for (int i = 0; i < nSamples; ++i) {
        outbuf[i] = input[i] * gain[i];
    }
}

} // namespace PIDController

PluginLoad(PIDControllerUGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PIDController::PIDController>(ft, "PIDController", false);
}
