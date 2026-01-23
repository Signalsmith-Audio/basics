/* Copyright 2022+ Signalsmith Audio Ltd. / Geraint Luff
MIT License - see LICENSE.txt and SUPPORT.txt */
#pragma once

#include "signalsmith-dsp/delay.h"
#include "signalsmith-dsp/filters.h"
#include "signalsmith-dsp/envelopes.h"
SIGNALSMITH_DSP_VERSION_CHECK(1, 7, 0)

#include "../stfx/stfx-library.h"

#include <cmath>

namespace signalsmith { namespace basics {

template<class BaseEffect>
class DynamicsSTFX;

using DynamicsFloat = stfx::LibraryEffect<float, DynamicsSTFX>;
using DynamicsDouble = stfx::LibraryEffect<double, DynamicsSTFX>;

template<class BaseEffect>
struct DynamicsSTFX : public BaseEffect {
	using typename BaseEffect::Sample;
	using typename BaseEffect::ParamRange;
	using typename BaseEffect::ParamStepped;

	ParamRange compressorDb{-20};
	ParamRange compressorRatioInv{0.5}; // 0.5 = 2:1 compression
	ParamRange expanderDb{-40};
	ParamRange expanderRatio{1}; // 2 = 2:1 expansion
	ParamRange kneeDb{3};
	ParamRange gateDb{-80};

	ParamRange lookaheadMs{0};
	ParamRange attackMs{30};
	ParamRange releaseMs{150};

	ParamStepped autoGain{1};
	ParamRange mix{1};
	
	DynamicsSTFX(double maxLookaheadMs=20, double autoGainRefDb=-20) : maxLookaheadMs(maxLookaheadMs), autoGainRefDb(autoGainRefDb) {}

	template<class Storage>
	void state(Storage &storage) {
		storage.info("Dynamics", "Compressor / Expander / Gate");
		int version = storage.version(0);

		using stfx::units::dbToGain;
		storage.range("compressorDb", compressorDb)
			.info("compressor", "compressor threshold")
			.range(-60, -30, 0)
			.unit("dB", 0);
		storage.range("compressorRatioInv", compressorRatioInv)
			.info("comp. ratio", "compressor ratio")
			.range(1, 0.5, 0)
			.unit("", 1, inv, inv).exact(0, "Inf");
		storage.range("expanderDb", expanderDb)
			.info("expander", "expander threshold")
			.range(-60, -30, 0)
			.unit("dB", 0);
		storage.range("expanderRatio", expanderRatio)
			.info("exp. ratio", "expander ratio")
			.range(1, 4, 100)
			.unit("", 1);
		storage.range("kneeDb", kneeDb)
			.info("knee", "compressor/expander knee")
			.range(0, 3, 12)
			.unit("dB", 1);
		storage.range("gateDb", gateDb)
			.info("gate", "gate threshold")
			.range(-80, -50, -20)
			.unit("dB", 0);

		stfx::units::rangeMs(storage.range("lookaheadMs", lookaheadMs)
			.info("lookahead", "lookahead (and max/smoothing) period")
			.range(0, 5, maxLookaheadMs));
		stfx::units::rangeMs(storage.range("attackMs", attackMs)
			.info("attack", "reaction time to increased volume")
			.range(0, 15, 50));
		stfx::units::rangeMs(storage.range("releaseMs", releaseMs)
			.info("release", "reaction time to decrease volume")
			.range(20, 100, 250));

		storage.stepped("autoGain", autoGain)
			.info("auto-gain", "gain compensation")
			.label(0, "off", "on");
		stfx::units::rangePercent(storage.range("mix", mix)
			.info("mix", "wet/dry mix")
			.range(0, 0.5, 1));
	}
	
	template<class Config>
	void configureSTFX(Config &config) {
		sampleRate = config.sampleRate;
		channels = config.outputChannels = config.inputChannels;
		config.auxOutputs.resize(0);
		hasAux = false;
		if (!config.auxInputs.empty()) {
			hasAux = true;
			config.auxInputs.resize(1);
			config.auxInputs[0] = channels;
		}
		
		envelopeHP.resize(channels);
		envelopeLP.resize(channels);
		
		int maxDelaySamples = std::ceil(maxLookaheadMs*0.001*sampleRate);
		delayBuffer.resize(int(channels), maxDelaySamples + 1 + config.maxBlockSize);
		peakHold.resize(maxDelaySamples + 1);
		boxFilter.resize(maxDelaySamples + 1);
	}
	
	void reset() {
		envelope.reset();

		delayBuffer.reset();
		peakHold.reset();
		boxFilter.reset();
		smoothedGain1 = smoothedGain2 = 1;
	}
	
	size_t latencySamples() const {
		return std::round(lookaheadMs*0.001*sampleRate);
	}
	
	double mapDb(double inDb) const {
		Sample outDb = inDb;
		if (inDb > compressorDb) {
			outDb += (inDb - compressorDb)*(compressorRatioInv - 1);
		}
		if (inDb < expanderDb) {
			outDb += (inDb - expanderDb)*(expanderRatio - 1);
		}
		if (inDb < gateDb) outDb -= 1000;
		return outDb;
	}
	
	template <class Io, class Config, class Block>
	void processSTFX(Io &io, Config &, Block &block) {
		// If we change the lookahead, we want to fade between the two delay times
		int lookaheadSamplesFrom = std::round(lookaheadMs.from()*0.001*sampleRate);
		int lookaheadSamplesTo = std::round(lookaheadMs.to()*0.001*sampleRate);
		peakHold.set(lookaheadSamplesTo + 1);
		boxFilter.set(lookaheadSamplesTo + 1);

		auto attackSamples = attackMs*0.001f*sampleRate;
		auto releaseSamples = releaseMs*0.001f*sampleRate;
		envelope.setPeriods(attackSamples, releaseSamples);
		
		auto smoothedMix = block.smooth(mix);
		
		for (int i = 0; i < int(block.length); ++i) {
			Sample e = 0;
			for (size_t c = 0; c < channels; ++c) {
//				Sample x = hasAux ? io.auxInput[0][c][i] : io.input[c][i];
				Sample x = io.input[c][i];
				x = envelopeHP[c](envelopeLP[c](x));
				e += x*x;
			}
			auto result = envelope(e/channels);
			auto lookaheadEnergy = boxFilter(peakHold(result.energy));
			
			auto inDb = 10*std::log10(lookaheadEnergy);
			auto outDb = Sample(mapDb(inDb));
			
			Sample gain = std::pow(10, (outDb - inDb)/20);
			if (gain < Sample(1e-30)) gain = 0;
			
			smoothedGain1 += (gain - smoothedGain1)*result.slew;
			smoothedGain2 += (smoothedGain1 - smoothedGain2)*result.slew;
			
			gain = 1 + smoothedMix.at(i)*(smoothedGain2 - 1);

			for (int c = 0; c < int(channels); ++c) {
				delayBuffer[c][i] = io.input[c][i];
				Sample delayed = block.fade(i,
					delayBuffer[c][i - lookaheadSamplesFrom],
					delayBuffer[c][i - lookaheadSamplesTo]
				);
				io.output[c][i] = delayed*gain;
			}
			io.output[0][i] = std::sqrt(lookaheadEnergy);
		}
		delayBuffer += int(block.length);
	}

private:
	const Sample maxLookaheadMs, autoGainRefDb;
	Sample sampleRate;

	signalsmith::delay::MultiBuffer<Sample> delayBuffer;
	signalsmith::envelopes::PeakHold<Sample> peakHold{1};
	signalsmith::envelopes::BoxFilter<Sample> boxFilter{1};
	size_t channels = 0;
	bool hasAux = false;
	
	Sample smoothedGain1 = 1, smoothedGain2 = 1;
	
	static double inv(double x) {
		return 1/x;
	}
	
	using Filter = signalsmith::filters::BiquadStatic<Sample>;
	std::vector<Filter> envelopeHP, envelopeLP;
	
	// An envelope follower with adaptive attack
	struct EnvelopeFollower {
		void reset() {
			e1 = e2 = 0;
		}
		
		void setPeriods(Sample attack, Sample release) {
			attackSamples = attack;
			releaseDecay = std::exp(-1/release);
		}
		
		struct Result {
			Sample energy, period, slew;
		};
		
		Result operator()(Sample energy) {
			Sample ratio = energy/(e2 + Sample(1e-30));
			Sample period = attackSamples*std::exp(-ratio);
			Sample slew = 1/(period + 1);
			
			e1 = std::max<Sample>(e1*releaseDecay, e1 + (energy - e1)*slew);
			e2 = std::max<Sample>(e2*releaseDecay, e2 + (e1 - e2)*slew);

			return {e2, period, slew};
		}
	private:
		Sample attackSamples = 1;
		Sample releaseDecay = 0;
		Sample e1 = 0, e2 = 0;
	};
	EnvelopeFollower envelope;
};

}} // namespace
