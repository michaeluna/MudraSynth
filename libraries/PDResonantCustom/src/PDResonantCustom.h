#ifndef PD_RESONANT_CUSTOM_H
#define PD_RESONANT_CUSTOM_H

#include <Arduino.h>
#include <mozzi_midi.h>
#include <ADSR.h>
#include <Oscil.h>
#include <Phasor.h>
#include <tables/sin2048_int8.h>

class PDResonantCustom {
public:
  PDResonantCustom()
      : pdDepth(0.05f), frequency(0.0f), previousBaseCounter(0),
        currentAmplitude(0) {
    oscillator.setTable(SIN2048_DATA);
    amplitudeEnvelope.setADLevels(255, 255);
    amplitudeEnvelope.setTimes(50, 300, 60000, 1000);
    resonantEnvelope.setADLevels(255, 100);
    resonantEnvelope.setTimes(50, 1500, 60000, 1000);
  }

  void noteOn(byte channel, byte pitch, byte velocity) {
    (void)channel;
    (void)velocity;
    resonantEnvelope.noteOn();
    amplitudeEnvelope.noteOn();
    setPitch(pitch);
  }

  // Change pitch without restarting either envelope.
  void setPitch(byte pitch) {
    setFrequency(mtof(pitch));
  }

  // Change to an arbitrary frequency without restarting either envelope.
  void setFrequency(float newFrequency) {
    frequency = newFrequency;
    baseCounter.setFreq(frequency);
    resonanceCounter.setFreq(frequency);
  }

  void noteOff(byte channel, byte pitch, byte velocity) {
    (void)channel;
    (void)pitch;
    (void)velocity;
    amplitudeEnvelope.noteOff();
    resonantEnvelope.noteOff();
  }

  void setPDEnv(uint16_t attackMs, uint16_t decayMs) {
    resonantEnvelope.setTimes(attackMs, decayMs, 60000, 1000);
  }

  void setPDDepth(float depth) {
    if (depth < 0.0f) depth = 0.0f;
    if (depth > 1.0f) depth = 1.0f;
    pdDepth = depth;
  }

  void setAmplitudeAttack(uint16_t attackMs) {
    amplitudeEnvelope.setAttackTime(attackMs);
  }

  float getPDDepth() const {
    return pdDepth;
  }

  byte getAmplitude() const {
    return currentAmplitude;
  }

  void update() {
    amplitudeEnvelope.update();
    resonantEnvelope.update();
    const float resonanceFrequency = frequency
        + frequency * resonantEnvelope.next() * pdDepth;
    resonanceCounter.setFreq(resonanceFrequency);
  }

  int8_t next() {
    return nextSample(true);
  }

  int8_t nextUnenveloped() {
    return nextSample(false);
  }

private:
  int8_t nextSample(bool applyAmplitudeEnvelope) {
    const byte basePosition = baseCounter.next() >> 24;
    if (basePosition < previousBaseCounter) {
      resonanceCounter.set(0);
    }
    previousBaseCounter = basePosition;

    const unsigned int index = resonanceCounter.next() >> 21;
    const byte amplitudeRamp = 255 - basePosition;
    currentAmplitude = amplitudeEnvelope.next();
    const byte outputAmplitude =
        applyAmplitudeEnvelope ? currentAmplitude : 255;
    return ((long)outputAmplitude
            * amplitudeRamp
            * oscillator.atIndex(index)) >> 16;
  }

  float pdDepth;
  float frequency;
  byte previousBaseCounter;
  byte currentAmplitude;
  Phasor<MOZZI_AUDIO_RATE> baseCounter;
  Phasor<MOZZI_AUDIO_RATE> resonanceCounter;
  Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> oscillator;
  ADSR<MOZZI_CONTROL_RATE, MOZZI_AUDIO_RATE> amplitudeEnvelope;
  ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> resonantEnvelope;
};

#endif
