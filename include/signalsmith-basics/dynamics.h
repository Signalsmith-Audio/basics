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
	
	static constexpr Sample maxLookaheadMs = 20;

	struct Compressor {
		ParamRange limitDb{-20};
		ParamRange invRatio{0.5}; // 0.5 = 2:1 compression
		ParamStepped autoGain{1};
		ParamRange makeUpGain{0}; // in addition to auto-gain

		template<class Storage>
		void state(Storage &storage) {
			storage.info("Compressor", "");
			storage.version(0);
			
			storage.range("limitDb", limitDb)
				.info("limit", "compressor threshold")
				.range(-60, -30, 0)
				.unit("dB", 0);
			storage.range("invRatio", invRatio)
				.info("ratio", "compressor ratio")
				.range(1, 0.5, 0)
				.unit("", 1, inv, inv).exact(0, "Inf");
			storage.stepped("autoGain", autoGain)
				.info("auto-gain", "gain compensation")
				.label(0, "off", "on");
			storage.range("makeUpGain", makeUpGain)
				.info("make-up", "additional make-up gain")
				.range(0, 6, 20)
				.unit("dB", 1).exact(0, "off");
		}
	} compressor;

	struct Expander {
		ParamRange limitDb{-40};
		ParamRange ratio{1}; // 2 = 2:1 expansion

		template<class Storage>
		void state(Storage &storage) {
			storage.info("Expander", "");
			storage.version(0);

			storage.range("limitDb", limitDb)
				.info("limit", "expander threshold")
				.range(-60, -30, 0)
				.unit("dB", 0);
			storage.range("ratio", ratio)
				.info("ratio", "expander ratio")
				.range(1, 2, 10)
				.unit("", 1);
		}
	} expander;

	struct Gate {
		ParamRange limitDb{-80};
		ParamRange hysteresis{3};

		template<class Storage>
		void state(Storage &storage) {
			storage.info("Gate", "");
			storage.version(0);

			storage.range("limitDb", limitDb)
				.info("limit", "gate limit threshold")
				.range(-80, -40, 0)
				.unit("dB", 0);
			storage.range("hysteresis", hysteresis)
				.info("hysteresis", "expander ratio")
				.range(0, 3, 12)
				.unit("dB", 1);
		}
	} gate;

	ParamRange kneeDb{3};

	struct Timing {
		ParamRange lookaheadMs{0};
		ParamRange attackMs{20};
		ParamRange releaseMs{150};

		template<class Storage>
		void state(Storage &storage) {
			storage.info("Timing", "");
			storage.version(0);

			stfx::units::rangeMs(storage.range("lookaheadMs", lookaheadMs)
				.info("lookahead", "lookahead (and max/smoothing) period")
				.range(0, 5, maxLookaheadMs));
			stfx::units::rangeMs(storage.range("attackMs", attackMs)
				.info("attack", "reaction time to increased volume")
				.range(0, 20, 50));
			stfx::units::rangeMs(storage.range("releaseMs", releaseMs)
				.info("release", "reaction time to decrease volume")
				.range(20, 100, 250));
		}
	} timing;

	ParamRange mix{1};
	
	DynamicsSTFX(double autoGainRefDb=-20) : autoGainRefDb(autoGainRefDb) {}

	template<class Storage>
	void state(Storage &storage) {
		storage.info("Dynamics", "Compressor / Expander / Gate");
		storage.version(0);
		
		storage("compressor", compressor);
		storage("expander", expander);
		storage("gate", gate);

		storage.range("kneeDb", kneeDb)
			.info("knee", "compressor/expander knee")
			.range(0, 3, 12)
			.unit("dB", 1);

		storage("timing", timing);

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
		gateIsOpen = true;
	}
	
	size_t latencySamples() const {
		return std::round(timing.lookaheadMs*0.001*sampleRate);
	}
	
	double mapDb(double inDb, bool gateOpen=true) const {
		double outDb = inDb;
		if (inDb > compressor.limitDb) {
			outDb += (inDb - compressor.limitDb)*(compressor.invRatio - 1);
		}
		if (inDb < expander.limitDb) {
			outDb += (inDb - expander.limitDb)*(expander.ratio - 1);
		}

		// Knees are implemented as BLEP-like curve, cancelling out the hard corner to produce a smooth gradient
		double dist = std::abs(inDb - compressor.limitDb);
		if (dist < kneeDb) {
			double kneeCurve = kneeDb - dist;
			kneeCurve = kneeCurve*kneeCurve*kneeCurve/(6*kneeDb*kneeDb);
			outDb += (compressor.invRatio - 1)*kneeCurve; // gradient change at the corner
		}
		dist = std::abs(inDb - expander.limitDb);
		if (dist < kneeDb) {
			double kneeCurve = kneeDb - dist;
			kneeCurve = kneeCurve*kneeCurve*kneeCurve/(6*kneeDb*kneeDb);
			outDb += (1 - expander.ratio)*kneeCurve;
		}

		double gateDb = gate.limitDb + (gateOpen ? 0.0 : double(gate.hysteresis));
		if (inDb < gateDb) outDb -= 1000;
		return outDb;
	}
	double autoGainDb() const {
		double refDb = std::max(autoGainRefDb, std::max<Sample>(expander.limitDb, gate.limitDb));
		return refDb - mapDb(refDb, true);
	}
	
	template <class Io, class Config, class Block>
	void processSTFX(Io &io, Config &, Block &block) {
		// If we change the lookahead, we want to fade between the two delay times
		int lookaheadSamplesFrom = std::round(timing.lookaheadMs.from()*0.001*sampleRate);
		int lookaheadSamplesTo = std::round(timing.lookaheadMs.to()*0.001*sampleRate);
		peakHold.set(lookaheadSamplesTo + 1);
		boxFilter.set(lookaheadSamplesTo + 1);

		auto attackSamples = timing.attackMs*0.001f*sampleRate;
		auto releaseSamples = timing.releaseMs*0.001f*sampleRate;
		envelope.setPeriods(attackSamples, releaseSamples);
		
		auto smoothedMix = block.smooth(mix);
		
		Sample extraDb = (compressor.autoGain == 0) ? Sample(0) : Sample(autoGainDb());
		
		for (int i = 0; i < int(block.length); ++i) {
			Sample e = 0;
			for (size_t c = 0; c < channels; ++c) {
				Sample x = hasAux ? io.auxInput[0][c][i] : io.input[c][i];
				e += x*x;
			}
			auto result = envelope(e/channels);
			auto lookaheadEnergy = boxFilter(peakHold(result.energy));
			
			auto inDb = 10*std::log10(lookaheadEnergy + Sample(1e-30));
			auto outDb = Sample(mapDb(inDb, gateIsOpen)) + extraDb;
			
			Sample gain = std::pow(10, (outDb - inDb)/20);
			gateIsOpen = (gain >= Sample(1e-30));
			if (!gateIsOpen) gain = 0;
			
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
		}
		delayBuffer += int(block.length);
	}

private:
	const Sample autoGainRefDb;
	Sample sampleRate;

	signalsmith::delay::MultiBuffer<Sample> delayBuffer;
	signalsmith::envelopes::PeakHold<Sample> peakHold{1};
	signalsmith::envelopes::BoxFilter<Sample> boxFilter{1};
	size_t channels = 0;
	bool hasAux = false;
	
	Sample smoothedGain1 = 1, smoothedGain2 = 1;
	bool gateIsOpen = true;;
	
	static double inv(double x) {
		return 1/x;
	}
	
	using Filter = signalsmith::filters::BiquadStatic<Sample>;
	
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
