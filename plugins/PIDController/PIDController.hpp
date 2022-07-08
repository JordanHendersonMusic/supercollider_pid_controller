// PluginPIDController.hpp
// Jordan Henderson (j.henderson.music@outlook.com)

#pragma once

#include "SC_PlugIn.hpp"

namespace PIDController {

class PIDController : public SCUnit {
public:
    PIDController();
private:
	void next_k(int number_of_samples);
    void next_a(int nSamples);

	enum : int { Target=0, K1=1, K2=2, K3=3 };
	struct PreviousKs { float k1{0}, k2{0}, k3{0}; } previous;

	const float audio_time_step{static_cast<float>(sampleDur())};
	const float control_time_step{static_cast<float>(controlDur())};

	float final_value{0}, final_value_delta{0};

	float prev_input{0};
	float input_delta{0};


};

} // namespace PIDController
