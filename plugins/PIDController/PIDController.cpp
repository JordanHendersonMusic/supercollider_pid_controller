// PluginPIDController.cpp
// Jordan Henderson (j.henderson.music@outlook.com)

#include "SC_PlugIn.hpp"
#include "PIDController.hpp"

static InterfaceTable* ft;

namespace PIDController {

PIDController::PIDController() {
	if (inRate(Target) == calc_FullRate) {
		mCalcFunc = make_calc_function<PIDController, &PIDController::next_a>();
		next_a(1);
	} else {
		mCalcFunc = make_calc_function<PIDController, &PIDController::next_k>();
		next_k(1);
	}
}

void PIDController::next_a(int nSamples) {
	const float* input{in(Target)};
	const float* k1{in(K1)};
	const float* k2{in(K2)};
	const float* k3{in(K3)};

	float* output{out(0)};

	for (int i = 0; i < nSamples; ++i) {
		input_delta = (input[i] - prev_input) / audio_time_step;
		final_value = final_value + (audio_time_step * final_value_delta);
		const float amount = (input[i] + k3[i] * input_delta -
		                      final_value - k1[i] * final_value_delta);
		final_value_delta = final_value_delta + audio_time_step * amount / k2[i];
		output[i] = final_value;
		prev_input = input[i];
	}
}

void PIDController::next_k(int nSamples) {
    const float* input{in(Target)};

	const auto get_consumable_k = [&](int index, float& prev){
		const float k{in(index)[0]};
		return makeSlope(k, prev);
	};

	auto k1 = get_consumable_k(K1, previous.k1);
	auto k2 = get_consumable_k(K2, previous.k2);
	auto k3 = get_consumable_k(K3, previous.k3);

    float* output{out(0)};

	for (int i = 0; i < nSamples; ++i) {
		input_delta = (input[i] - prev_input) / control_time_step;
		final_value = final_value + (control_time_step * final_value_delta);
		const float amount = (input[i] + k3.consume() * input_delta -
		                      final_value - k1.consume() * final_value_delta);
		final_value_delta = final_value_delta + control_time_step * amount / k2.consume();
		output[i] = final_value;
		prev_input = input[i];
	}

	previous.k1 = k1.value;
	previous.k2 = k2.value;
	previous.k3 = k3.value;
}




} // namespace PIDController

PluginLoad(PIDControllerUGens) {
	// Plugin magic
	ft = inTable;
	registerUnit<PIDController::PIDController>(ft, "PIDController", false);
}
