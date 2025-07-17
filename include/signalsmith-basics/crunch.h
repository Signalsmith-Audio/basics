/* Copyright 2022+ Signalsmith Audio Ltd. / Geraint Luff
MIT License - see LICENSE.txt and SUPPORT.txt */
#pragma once

#include "../../modules/dsp/rates.h"
#include "../../modules/dsp/filters.h"
SIGNALSMITH_DSP_VERSION_CHECK(1, 4, 1)

#include "../stfx/stfx-library.h"

#include <cmath>

namespace signalsmith { namespace basics {

template<class BaseEffect>
class CrunchSTFX;

using CrunchFloat = stfx::LibraryEffect<float, CrunchSTFX>;
using CrunchDouble = stfx::LibraryEffect<double, CrunchSTFX>;

template<class BaseEffect>
struct CrunchSTFX : public BaseEffect {
	using typename BaseEffect::Sample;
	using typename BaseEffect::ParamRange;
	using typename BaseEffect::ParamStepped;

	static constexpr int oversampleHalfLatency = 16;
	static constexpr Sample autoGainLevel = 0.1;

	const bool autoGain;

	ParamRange drive{4};//stfx::units::dbToGain(autoGain ? 24 : 12)};
	ParamRange fuzz{0};
	ParamRange toneHz{2000};
	ParamRange cutHz{50};
	ParamRange outGain{1};

	CrunchSTFX(bool autoGain=true) : autoGain(autoGain) {}

	template<class Storage>
	void state(Storage &storage) {
		storage.info("Crunch", "A simple distortion/saturation");
		int version = storage.version(0);
		if (version != 0) return;

		using stfx::units::dbToGain;
		stfx::units::rangeGain(storage.range("drive", drive)
			.info("drive", "pre-distortion input gain")
			.range(dbToGain(-12), 4, dbToGain(40)));
		stfx::units::rangePercent(storage.range("fuzz", fuzz)
			.info("fuzz", "amplitude-independent distortion")
			.range(0, 0.5, 1));
		stfx::units::rangeHz(storage.range("toneHz", toneHz)
			.info("tone", "limits the brightness of the distortion")
			.range(100, 4000, 20000));
		stfx::units::rangeHz(storage.range("cutHz", cutHz)
			.info("cut", "prevents low frequencies from driving the distortion")
			.range(20, 100, 500));

		stfx::units::rangeGain(storage.range("outGain", outGain)
			.info("out", "output gain")
			.range(dbToGain(-12), 1, dbToGain(24)));
	}
	
	template<class Config>
	void configureSTFX(Config &config) {
		channels = config.outputChannels = config.inputChannels;
		config.auxInputs.resize(0);
		config.auxOutputs.resize(0);
		
		oversampler.resize(int(channels), int(config.maxBlockSize), oversampleHalfLatency, std::min(0.45, 21000/config.sampleRate));
		gainshapers.resize(channels);
		cutFilters.resize(channels);
		toneFilters.resize(channels);
		outputFilters.resize(channels);
	}
	
	void reset() {
		oversampler.reset();
		for (auto &g : gainshapers) g.reset();
		for (auto &f : cutFilters) f.reset();
		for (auto &f : toneFilters) f.reset();
		for (auto &f : outputFilters) f.reset();
	}
	
	int latencySamples() const {
		return oversampleHalfLatency*2;
	}
	
	template <class Io, class Config, class Block>
	void processSTFX(Io &io, Config &config, Block &block) {
		auto inputGain = block.smooth(drive);

		double outputGainFrom = outGain.from();
		double outputGainTo = outGain.to();
		if (autoGain) {
			Sample cutRatioFrom = 1 - cutHz.from()/(cutHz.from() + 200);
			Sample averageGainFrom = gainshapers[0].averageGain(autoGainLevel*cutRatioFrom*drive.from());
			outputGainFrom /= drive.from()*averageGainFrom;
			Sample cutRatioTo = 1 - cutHz.to()/(cutHz.to() + 200);
			Sample averageGainTo = gainshapers[0].averageGain(autoGainLevel*cutRatioTo*drive.to());
			outputGainTo /= drive.to()*averageGainTo;
		}
		auto outputGain = block.smooth(outputGainFrom, outputGainTo);
		
		for (size_t c = 0; c < channels; ++c) {
			auto &cutFilter = cutFilters[c];
			cutFilter.highpass(cutHz/(config.sampleRate*2));
			auto &gainshaper = gainshapers[c];
			gainshaper.setFuzzFactor(fuzz);
			auto &toneFilter = toneFilters[c];
			toneFilter.lowpass(toneHz/(config.sampleRate*2));
			auto &outputFilter = outputFilters[c];
			outputFilter.highpass((10 + 40*fuzz)/(config.sampleRate*2)); // more aggressive when fuzz is enabled, since it's very asymmetrical

			oversampler.upChannel(int(c), io.input[c], int(block.length));
			Sample *samples = oversampler[int(c)];
			for (size_t i = 0; i < block.length*2; ++i) {
				double hi = i*0.5;
				Sample x = samples[i]*inputGain.at(hi);
				Sample gain = gainshaper(cutFilter(x))*outputGain.at(hi);
				Sample y = x*toneFilter(gain);
				samples[i] = outputFilter(y);
			}
			oversampler.downChannel(int(c), io.output[c], int(block.length));
		}
	}
	
private:
	size_t channels = 0;
	signalsmith::rates::Oversampler2xFIR<Sample> oversampler;
	struct GainshapeADAA {
		Sample prevX = 0, prevIntegral = 0;
		Sample fuzzPositive = 1, fuzzNegative = 1;

		void setFuzzFactor(Sample k) {
			fuzzPositive = 1 + k - k*k;
			fuzzNegative = 1 - k - k*k;
			prevIntegral = integralGain(prevX);
		}

		Sample gain(Sample x) const {
			Sample fuzzGain = (x >= 0 ? fuzzPositive : fuzzNegative);
			return fuzzGain/std::sqrt(1 + x*x);
		}
		Sample integralGain(Sample x) const {
			if (x >= 0) {
				return fuzzPositive*std::log(std::sqrt(1 + x*x) + x);
			} else { // more accurate if we flip it
				return -fuzzNegative*std::log(std::sqrt(1 + x*x) - x);
			}
		}
		Sample averageGain(Sample range) const {
			// Average gain from 0-range, ignoring fuzz
			return std::log(std::sqrt(1 + range*range) + range)/range;
		}
		static constexpr Sample minDiffX = 1e-4;
	
		void reset() {
			prevX = 0;
			prevIntegral = integralGain(prevX);
		}

		Sample operator()(Sample x) {
			Sample diffX = x - prevX;
			Sample integral = integralGain(x);
			Sample diffIntegral = integral - prevIntegral;
			prevX = x;
			prevIntegral = integral;
			if (std::abs(diffX) < minDiffX) return gain(x);
			return diffIntegral/diffX;
		}
	};
	std::vector<GainshapeADAA> gainshapers;
	using Filter = signalsmith::filters::BiquadStatic<Sample>;
	std::vector<Filter> cutFilters, toneFilters, outputFilters;
};

}} // namespace
