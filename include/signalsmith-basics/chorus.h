/* Copyright 2025+ Signalsmith Audio Ltd. / Geraint Luff
MIT License - see LICENSE.txt and SUPPORT.txt */
#pragma once

#include "../stfx/stfx-library.h"

#include "../../modules/dsp/delay.h"

#include <vector>

namespace signalsmith { namespace basics {

template<class BaseEffect>
struct ChorusSTFX;

using ChorusFloat = stfx::LibraryEffect<float, ChorusSTFX>;
using ChorusDouble = stfx::LibraryEffect<double, ChorusSTFX>;

template<class BaseEffect>
struct ChorusSTFX : public BaseEffect {
	using typename BaseEffect::Sample;
	using Complex = std::complex<Sample>;
	using typename BaseEffect::ParamRange;
	using typename BaseEffect::ParamStepped;

	// Chosen so that points when the delays cross over and
	static constexpr Complex oscillatorOffsets[6] = {
		{Sample(-0.267409), Sample(0.195109)},
		{Sample(-0.680406), Sample(-0.363478)},
		{Sample(0.802226), Sample(0.551654)},
		{Sample(0.188176), Sample(-0.980498)},
		{Sample(0.973565), Sample(-0.204023)},
		{Sample(-0.742801), Sample(0.669512)}
	};

	const Sample maxDepthMs;
	
	ChorusSTFX(Sample maxDepthMs=50) : maxDepthMs(maxDepthMs) {}
	
	ParamRange mix{0.6};
	ParamRange depthMs{15};
	ParamRange detune{5};
	ParamRange stereo{1};
	
	template<class Storage>
	void state(Storage &storage) {
		storage.info("Chorus", "");
		storage.version(0);

		stfx::units::rangePercent(storage.range("mix", mix)
			.info("mix", "")
			.range(0, 0.5, 1));

		storage.range("depthMs", depthMs)
			.info("depth", "chorus delay range")
			.range(2, 15, maxDepthMs)
			.unit("ms", 1, 0, 9.9).unit("ms", 0);
		
		storage.range("detune", detune)
			.info("detune", "detuning depth")
			.range(1, 8, 50)
			.unit(" cents", 0);

		stfx::units::rangePercent(storage.range("stereo", stereo)
			.info("stereo", "")
			.range(0, 0.5, 1.5));
	}

	template<class Config>
	void configureSTFX(Config &config) {
		sampleRate = config.sampleRate;
		config.outputChannels = config.inputChannels;
		config.auxInputs = config.auxOutputs = {};

		Sample maxDelaySamples = maxDepthMs*0.001*config.sampleRate;
		delay.resize(6, maxDelaySamples);
	}
	
	void reset() {
		delay.reset();
		phase = 0;
		stereoPhase = 0;
	}
	
	template<class Io, class Config, class Block>
	void processSTFX(Io &io, Config &config, Block &block) {
		Sample detuneHz = detune*0.45f/depthMs; // 0.45ms oscillation at 1Hz is about 1 semitone
		Sample phaseStep = detuneHz/config.sampleRate;
		Sample stereoPhaseStep = phaseStep/Sample(M_PI);
		Complex stereoComplexPerOutput = std::polar(Sample(1), Sample(2*M_PI)/config.outputChannels);
		Complex stereoComplexPerInternal = std::polar(Sample(1), Sample(1));
		
		bool fading = depthMs.from() != depthMs.to();
		Sample depthSamples = depthMs.to()*0.001*config.sampleRate;
		Sample depthSamplesFrom = depthMs.from()*0.001*config.sampleRate;

		std::array<Sample, 6> multiIn, multiOut;
		std::array<Sample, 6> delaySamples;
		
		auto wetDryFn = [](double mix, double width, bool isWet){
			// Equal-power wet/dry fade
			double m = isWet ? std::sin(M_PI*mix/2)/std::sqrt(6) : std::cos(M_PI*mix/2);
			// Compensate for extra energy from stereo
			double mw = mix*width;
			return m/std::sqrt((1 + mw*mw/2));
		};
		auto dry = block.smooth(wetDryFn(mix.from(), stereo.from(), false), wetDryFn(mix.to(), stereo.to(), false));
		auto wet = block.smooth(wetDryFn(mix.from(), stereo.from(), true), wetDryFn(mix.to(), stereo.to(), true));
		bool notMono = (config.outputChannels > 1);
		auto width = block.smooth(stereo);
		
		for (size_t i = 0; i < block.length; ++i) {
			for (size_t c = 0; c < 6; ++c) {
				size_t inputC = c%config.inputChannels;
				multiIn[c] = io.input[inputC][i];
			}
			delay.write(multiIn);
			
			Complex phaseComplex = std::polar(Sample(1), phase*Sample(2*M_PI));
			for (size_t c = 0; c < 6; ++c) {
				Sample osc = (phaseComplex*oscillatorOffsets[c]).real();
				delaySamples[c] = depthSamples*Sample(0.5)*(1 + osc);
			}
			delay.readMulti(delaySamples, multiOut);

			if (fading) {
				// read a second set of delay times, and fade between them
				std::array<Sample, 6> multiOutFrom;
				for (size_t c = 0; c < 6; ++c) {
					Sample osc = (phaseComplex*oscillatorOffsets[c]).real();
					delaySamples[c] = depthSamplesFrom*Sample(0.5)*(1 + osc);
				}
				delay.readMulti(delaySamples, multiOutFrom);
				for (size_t c = 0; c < 6; ++c) {
					multiOut[c] = block.fade(i, multiOutFrom[c], multiOut[c]);
				}
			}

			// Arbitrarily chosen gains, 4 positive, 2 negative
			Sample sum = multiOut[0] - multiOut[1] + multiOut[2] + multiOut[3] - multiOut[4] + multiOut[5];
			if (notMono) {
				Complex stereoComplex = std::polar(Sample(width.at(i)), stereoPhase*Sample(2*M_PI));
				for (size_t oc = 0; oc < config.outputChannels; ++oc) {
					Sample sumC = sum;
					Complex rot = stereoComplex;
					for (size_t i = 0; i < 6; ++i) {
						sumC += rot.real()*multiOut[i];
						rot *= stereoComplexPerInternal;
					}

					io.output[oc][i] = multiIn[oc]*dry.at(i) + sumC*wet.at(i);
					stereoComplex *= stereoComplexPerOutput;
				}
				stereoPhase += stereoPhaseStep;
			} else {
				io.output[0][i] = multiIn[0]*dry.at(i) + sum*wet.at(i);
			}

			phase += phaseStep;
		}
		phase -= std::floor(phase);
		stereoPhase -= std::floor(stereoPhase);
	}
	
	template<class Storage>
	void meterState(Storage &storage) {}
	
private:
	Sample sampleRate;

	signalsmith::delay::MultiDelay<Sample, signalsmith::delay::InterpolatorLagrange7> delay;
	Sample phase = 0;
	Sample stereoPhase = 0;
};

}} // namespace
