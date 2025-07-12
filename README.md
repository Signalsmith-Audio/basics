# Signalsmith Basics

A collection of basic effects, available as plugins and re-usable open-source (MIT) C++ classes.

* **Analyser**
* **Chorus**
* **Crunch**
* **FreqShifter**
* **Limiter**
* **Reverb**

## How to use

Each effect can be used just by including the corresponding header (e.g. `limiter.h`).  This defines two classes `...Float` and `...Double`.  Some of these have initialisation arguments (e.g. maximum lookahead) but these are always optional.

```cpp
// Limiter with maximum attack/lookahead of 100ms
signalsmith::basics::LimiterFloat effect(100);
```

Before use, these classes must be configured.  The `.configure()` method returns `true` if the config was accepted.  They can be configured multiple times, but (obviously) not while actively processing audio.

```cpp
effect.configure(sampleRate, maxBlockSize, channels);
```

To process audio, pass in a classic array-of-buffers for input and output (which shouldn't be the same):

```cpp
float **inputBuffers, **outputBuffers;
int blockSize;
effect.process(inputBuffers, outputBuffers, blockSize);
```

When playback (re)starts, you can also call `.reset()` to clear any tails/state.  You can also inspect latency (`effect.latencySamples()`) and tail length (`effect.tailSamples()`). 

The actual implementations are templates (e.g. `LimiterSTFX`) which you shouldn't every need to use directly, although they should be fairly readable.  These are wrapped up into the `...Float` and `...Double` classes by the code in `stfx/stfx-library.h`, which also provides helpers for parameter-smoothing/etc., and more typical `.configure()` and `.process()` functions (from `.configureSTFX()`/`.processSTFX()`). 

### Parameters

Floating-point parameters are declared as `ParamRange`s in the `...STFX` template.  This is an opaque type which can be assigned from / converted into `double` from any thread.  Parameter smoothing is handled internally.

Integer parameters are declared as `ParamStepped`s.  This is a similarly opaque type which converts to/from `int`.

### Meters

The **Analyser** doesn't need any audio outputs (and will simply pass through audio if configured with them).  The spectrum data is provided using the two methods `.spectrumHz()` and `.spectrumEnergy(channel)`.

However, since the spectrum data is typically wanted on the main/UI thread, we need to know when it's OK to call the above methods.  This is done using two "meter" methods:

* `.wantsMeters(bool=true)` (thread-safe) - indicates whether meters should be provided
* `.hasMeters()` (`const` and thread-safe) - whether the values are ready and safe to read

 After any call to `.wantsMeters()`, `.hasMeters()` will immediately return `false`, indicating that no thread should read meter values.  If multiple threads are reading the meters, it is up to *you* to make sure another thread isn't calling `.wantsMeters()` between you checking `.hasMeters()` and using the values.
 
### Implementation

The `.state()` method of these templates contain a lot of detail, almost all of which is ignored (and optimised away) when using the `...Float`/`...Double` classes.

The `.configureSTFX()` and `.processSTFX()` methods are wrapped into more typical forms by `stfx/stfx-library.h`.

### Dependencies

Some of the effects use the submodules in `modules/`: DSP library.

### License and support

This project is released under the [MIT License](LICENSE.txt).  See [SUPPORT.txt](SUPPORT.txt) for the current (expected) support period and details.

If you'd like bespoke alterations to any of these effects, [get in touch](https://signalsmith-audio.co.uk/contact/)! 
