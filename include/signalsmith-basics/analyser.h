/* Copyright 2025+ Signalsmith Audio Ltd. / Geraint Luff
MIT License - see LICENSE.txt and SUPPORT.txt */
#pragma once

#include "../stfx/stfx-library.h"

#include "../../modules/dsp/curves.h"
#include "../../modules/linear/stft.h"
#include "../../modules/linear/linear.h"

#include <vector>

namespace signalsmith { namespace basics {

template<class BaseEffect>
struct AnalyserSTFX;

using AnalyserFloat = stfx::LibraryEffect<float, AnalyserSTFX>;
using AnalyserDouble = stfx::LibraryEffect<double, AnalyserSTFX>;

template<class BaseEffect>
struct AnalyserSTFX : public BaseEffect {
	using typename BaseEffect::Sample;
	using Complex = std::complex<Sample>;
	using typename BaseEffect::ParamRange;
	using typename BaseEffect::ParamStepped;
	
	static constexpr Sample stftBlockMs = 30, stftIntervalMs = 5;
	
	ParamRange barkResolution = 10;

	template<class Storage>
	void state(Storage &storage) {
		storage.info("Analyser", "A Bark-scale spectrum analyser");
		storage.version(0);
		
		storage.range("barkResolution", barkResolution)
			.info("res.", "in Bark scale")
			.range(2, 10, 25)
			.unit("", 0);
		
		if (storage.extra()) {
			storage("spectrum", spectrum);
		}
	}

	template<class Config>
	void configureSTFX(Config &config) {
		sampleRate = config.sampleRate;
		channels = config.inputChannels;
		config.auxInputs = config.auxOutputs = {};

		stft.configure(channels, 0, stftBlockMs*0.001*sampleRate, stftIntervalMs*0.001*sampleRate);
		subRate = sampleRate/stft.defaultInterval();
		bands = spectrum.resize(channels, barkResolution, sampleRate, stft.blockSamples());
		updateBands();
		tmp.resize(stft.defaultInterval());
	}
	
	void reset() {
		spectrum.reset();
	}
	
	template<class Io, class Config, class Block>
	void processSTFX(Io &io, Config &config, Block &block) {
		for (size_t c = 0; c < config.outputChannels; ++c) {
			auto &input = io.input[c%config.inputChannels];
			auto &output = io.output[c];
			for (size_t i = 0; i < block.length; ++i) {
				output[i] = input[i];
			}
		}
		
		bool processedSpectrum = false;
		size_t index = 0;
		while (index < block.length) {
			size_t remaining = stft.defaultInterval() - stft.samplesSinceAnalysis();
			size_t consume = std::min<size_t>(remaining, block.length - index);
			// copy input
			for (size_t c = 0; c < config.inputChannels; ++c) {
				auto &input = io.input[c];
				for (size_t i = 0; i < consume; ++i) {
					tmp[i] = input[index + i];
				}
				stft.writeInput(c, consume, tmp.data());
			}
			stft.moveInput(consume);
			if (remaining == consume) {
				stft.analyse();
				stftStep();
				processedSpectrum = true;
			}
			index += consume;
		}
		
		if (processedSpectrum && block.wantsMeters()) {
			for (size_t c = 0; c < channels; ++c) {
				size_t offset = c*bands;
				auto state2 = linear.wrap(state2Real.data() + offset, state2Imag.data() + offset, bands);
				linear.wrap(spectrum.energy[c]) = state2.norm();
			}
		}
	}
	
	template<class Storage>
	void meterState(Storage &storage) {
		storage("spectrum", spectrum);
	}
	
	const std::vector<Sample> & spectrumHz() const {
		return spectrum.hz;
	}
	const std::vector<Sample> & spectrumEnergy(size_t channel) const {
		return spectrum.energy[channel];
	}
	
private:
	signalsmith::linear::DynamicSTFT<Sample> stft;
	signalsmith::linear::Linear linear;
	std::vector<Sample> tmp;
	double prevBarkResolution = -1;

	Sample sampleRate, subRate;
	size_t channels = 0;
	size_t bands = 0;
	std::vector<Sample> inputBin;
	std::vector<Sample> bandInputReal, bandInputImag;
	std::vector<Sample> state1Real, state1Imag;
	std::vector<Sample> state2Real, state2Imag;
	std::vector<Sample> twistReal, twistImag, inputGain;
	
	void updateBands() {
		linear.reserve<Sample>(bands);
		
		bandInputReal.resize(bands*channels);
		bandInputImag.resize(bands*channels);
		state1Real.resize(bands*channels);
		state1Imag.resize(bands*channels);
		linear.wrap(state1Real) = 0;
		linear.wrap(state1Imag) = 0;
		state2Real.resize(bands*channels);
		state2Imag.resize(bands*channels);
		linear.wrap(state2Real) = 0;
		linear.wrap(state2Imag) = 0;

		inputBin.resize(0);
		twistReal.resize(0);
		twistImag.resize(0);

		for (size_t b = 0; b < bands; ++b) {
			Sample hz = spectrum.hz[b];
			Sample bwHz = spectrum.bwHz[b];
			
			inputBin.push_back(stft.freqToBin(hz/sampleRate));
			Sample freq = Sample(2*M_PI/subRate)*hz;
			Sample bw = Sample(2*M_PI/subRate)*bwHz;
			Complex twist = std::exp(Complex{-bw, freq});
			twistReal.push_back(twist.real());
			twistImag.push_back(twist.imag());
		}
		inputGain.resize(bands);
		linear.wrap(inputGain) = 1 - linear.wrap(twistReal, twistImag).abs();
	}
	
	void stftStep() {
		if (barkResolution != prevBarkResolution) {
			prevBarkResolution = barkResolution;
			bands = spectrum.resize(channels, barkResolution, sampleRate, stft.blockSamples());
			updateBands();
		}

		// Load inputs from the spectrum
		Sample scalingFactor = Sample(1)/stft.defaultInterval();
		for (size_t c = 0; c < channels; ++c) {
			auto *spectrum = stft.spectrum(c);
			auto *inputR = bandInputReal.data() + bands*c;
			auto *inputI = bandInputImag.data() + bands*c;
			for (size_t b = 0; b < bands; ++b) {
				Sample index = std::min<Sample>(inputBin[b], stft.bands() - Sample(1.001));
				size_t indexLow = std::floor(index);
				Sample indexFrac = index - std::floor(index);
				Complex v = spectrum[indexLow] + (spectrum[indexLow + 1] - spectrum[indexLow])*indexFrac;
				v *= scalingFactor;
				inputR[b] = v.real();
				inputI[b] = v.imag();
			}
		}

		auto twist = linear.wrap(twistReal, twistImag);
		auto gain = linear.wrap(inputGain);
		for (size_t c = 0; c < channels; ++c) {
			size_t offset = c*bands;
			auto state1 = linear.wrap(state1Real.data() + offset, state1Imag.data() + offset, bands);
			auto state2 = linear.wrap(state2Real.data() + offset, state2Imag.data() + offset, bands);
			auto input = linear.wrap(bandInputReal.data() + offset, bandInputImag.data() + offset, bands);
			state1 = state1*twist + input*gain;
			state2 = state2*twist + state1*gain;
		}
	}

	struct Spectrum {
		bool bandsChanged = false;
		std::vector<Sample> hz, bwHz;
		std::vector<std::vector<Sample>> energy;
		
		size_t resize(size_t channels, Sample barkResolution, Sample sampleRate, size_t fftBands) {
			bandsChanged = true;
			hz.resize(0);
			bwHz.resize(0);

			Sample maxBwHz = sampleRate/fftBands;
			auto barkScale = signalsmith::curves::Reciprocal<Sample>::barkScale();
			Sample barkStep = 1/barkResolution, barkEnd = barkScale.inverse(sampleRate/2);
			for (Sample bark = barkScale.inverse(0); bark < barkEnd; bark += barkStep) {
				auto bw = barkScale.dx(bark)*barkStep;
				if (!hz.empty() && bw >= maxBwHz) {
					// our Bark scale would be less dense than the FFT, so we might miss data
					for (auto linearHz = hz.back() + maxBwHz; linearHz < sampleRate/2; linearHz += maxBwHz) {
						hz.push_back(linearHz);
						bwHz.push_back(maxBwHz);
					}
					break;
				}
				hz.push_back(barkScale(bark));
				bwHz.push_back(bw);
			}
			hz.push_back(sampleRate/2);
			bwHz.push_back(barkScale.dx(barkEnd)*barkStep);
			
			energy.resize(channels);
			for (auto &e : energy) e.resize(hz.size());

			return hz.size();
		}
		
		void reset() {
			for (auto &e : energy) {
				for (auto &v : e) v = 0;
			}
		}
		
		template<class Storage>
		void state(Storage &storage) {
			if (bandsChanged || storage.extra()) {
				bandsChanged = false;
				storage.extra("$type", "Spectrum");
				storage("hz", hz);
			}
			storage("energy", energy);
		}
	} spectrum;
};

}} // namespace
