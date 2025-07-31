/* Copyright 2025+ Signalsmith Audio Ltd. / Geraint Luff
MIT License - see LICENSE.txt and SUPPORT.txt */
#pragma once

#include "../../modules/hilbert-iir/hilbert.h"

#include "../stfx/stfx-library.h"

#include <cmath>

namespace signalsmith { namespace basics {

template<class BaseEffect>
class FreqShifterSTFX;

using FreqShifterFloat = stfx::LibraryEffect<float, FreqShifterSTFX>;
using FreqShifterDouble = stfx::LibraryEffect<double, FreqShifterSTFX>;

template<class BaseEffect>
struct FreqShifterSTFX : public BaseEffect {
	using typename BaseEffect::Sample;
	using Complex = std::complex<Sample>;
	using typename BaseEffect::ParamRange;
	using typename BaseEffect::ParamStepped;
	
	ParamRange mix{1};
	ParamRange shiftHz{50};
	ParamStepped reflect{0};

	template<class Storage>
	void state(Storage &storage) {
		storage.info("Frequency Shifter", "A Hilbert / Bode single-sideband modulator");
		int version = storage.version(0);
		if (version != 0) return;

		stfx::units::rangePercent(storage.range("mix", mix)
			.info("mix", "wet/dry")
			.range(0, 0.5, 1));

		// convert to/from a friendly [-1, 1] control space
		auto shiftHzToControl = [](double y) {
			return (std::sqrt(2500 + 1.1*y*y) - 50)/y;
		};
		auto controlToShiftHz = [](double x) {
			return 100*x/(1.1 - x*x);
		};
		storage.range("shiftHz", shiftHz)
			.info("shift", "pre-distortion input gain")
			.range(-1000, 1000, shiftHzToControl, controlToShiftHz)
			.unit("Hz", 1, -9.99, 9.99)
			.unit("Hz", 0);

		storage.stepped("reflect", reflect)
			.info("reflect", "0Hz reflection mode")
			.range(0, 3)
			.label(0, "no reflect/duplicate", "reflect below 0Hz", "duplicate above 0Hz", "always reflect/duplicate");
	}
	
	template<class Config>
	void configureSTFX(Config &config) {
		auto channels = config.outputChannels = config.inputChannels;
		config.auxInputs.resize(0);
		config.auxOutputs.resize(0);
		
		hilbert = {Sample(config.sampleRate), int(channels)};
	}
	
	void reset() {
		shiftPhaseBefore = shiftPhaseAfter = 0;
		shiftBefore = shiftAfter = 1;
		hilbert.reset();
	}
	
	template <class Io, class Config, class Block>
	void processSTFX(Io &io, Config &config, Block &block) {
		// Smooth these values out to avoid clicks, but put them through a mapping function first
		auto dry = block.smooth(mix, [](double m){return 1 - m*m;});
		auto wet = block.smooth(mix, [](double m){return m*(2 - m);});

		auto phaseStep = block.smooth(shiftHz, [&](double hz){
			return hz/config.sampleRate;
		});
		
		int mode = reflect;
		bool reflectDown = (mode == 1 || mode == 3);
		bool duplicateUp = (mode == 2 || mode == 3);
		
		for (size_t i = 0; i < block.length; ++i) {
			auto gainDry = dry.at(i), gainWet = wet.at(i);
			for (size_t c = 0; c < config.inputChannels; ++c) {
				Sample x = io.input[c][i];
				// In general, this may alias at the high end when shifting up
				// but our Hilbert has a 20kHz lowpass, so that's enough
				// room for our maximum +1000Hz shift
				Complex y = shiftAfter*hilbert(x*shiftBefore, int(c));
				io.output[c][i] = gainDry*x + gainWet*y.real();
			}

			auto ps = phaseStep.at(i);
			bool shiftInput = (ps < 0) ? !reflectDown : duplicateUp;
			if (shiftInput) {
				shiftPhaseBefore += ps;
				shiftBefore = std::polar(Sample(1), shiftPhaseBefore*Sample(2*M_PI));
			} else {
				shiftPhaseAfter += ps;
				shiftAfter = std::polar(Sample(1), shiftPhaseAfter*Sample(2*M_PI));
			}
		}
		shiftPhaseBefore -= std::floor(shiftPhaseBefore);
		shiftPhaseAfter -= std::floor(shiftPhaseAfter);
	}
	
private:
	Sample shiftPhaseBefore = 0, shiftPhaseAfter = 0;
	Complex shiftBefore = 1, shiftAfter = 1;
	signalsmith::hilbert::HilbertIIR<Sample> hilbert;
};

}} // namespace
