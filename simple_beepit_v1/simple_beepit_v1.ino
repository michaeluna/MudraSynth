/*  Single-voice phase-distortion resonant instrument using Mozzi.

    A0: LDR voltage divider selects C major notes from C1 to C7.
    D3: Active-low pushbutton gates the note.
    D5: LED PWM output; 10% at rest and 10-100% follows the internal envelope.
    D6: Active-low looper button: record, playback, then exit.
    D9: Mozzi standard single-pin PWM audio output.

    LDR wiring:
      5V -- LDR --+-- A0
                  |
                 10k
                  |
                 GND
*/

#define MOZZI_CONTROL_RATE 64
#include <Mozzi.h>
#include <mozzi_midi.h>
#include <PDResonantCustom.h>

PDResonantCustom voice;
PDResonantCustom loopVoice;

const uint8_t C_MAJOR_OFFSETS_FROM_C[] PROGMEM = {0, 2, 4, 5, 7, 9, 11};

const uint8_t LDR_PIN = A0;
const uint8_t GATE_PIN = 3;
const uint8_t LED_PIN = 5;
const uint8_t LOOPER_BUTTON_PIN = 6;
const uint8_t LED_MIN_PWM = 26;
const uint8_t LED_PWM_RANGE = 229;
// Temporary PD-tuning mode. Set false to restore both volume envelopes.
const bool BYPASS_VOLUME_ENVELOPES = true;
const uint8_t LOWEST_MIDI_NOTE = 24;  // C1
const uint8_t HIGHEST_MIDI_NOTE = 96; // C7
const uint8_t SCALE_NOTES_PER_OCTAVE =
    sizeof(C_MAJOR_OFFSETS_FROM_C) / sizeof(C_MAJOR_OFFSETS_FROM_C[0]);
const uint8_t SCALE_OCTAVES =
    (HIGHEST_MIDI_NOTE - LOWEST_MIDI_NOTE) / 12;
const uint8_t SCALE_NOTE_COUNT =
    SCALE_OCTAVES * SCALE_NOTES_PER_OCTAVE + 1;

const float PD_DEPTH = 0.05f;
const float PD_SUSTAIN_DEPTH = PD_DEPTH * 0.80f;
const uint16_t PD_FILTER_ATTACK_TIME_MS = 25;
const uint16_t DEFAULT_AMPLITUDE_ATTACK_TIME_MS = 50;
const uint16_t MIN_AMPLITUDE_ATTACK_TIME_MS = 10;
const uint8_t AMPLITUDE_ATTACK_STEP_MS = 5;
const uint16_t RAPID_NOTE_WINDOW_MS = 500;
const uint8_t AMPLITUDE_ATTACK_RECOVERY_INTERVAL_MS = 10;
const uint16_t RELEASE_TIME_MS = 1500;
const uint16_t PD_SUSTAIN_MOVEMENT_MS = 3000;
const uint16_t PD_SUSTAIN_START_TICKS =
    ((uint32_t)MOZZI_CONTROL_RATE
     * (PD_FILTER_ATTACK_TIME_MS + RELEASE_TIME_MS) + 999) / 1000;
const uint16_t PD_SUSTAIN_MOVEMENT_TICKS =
    ((uint32_t)MOZZI_CONTROL_RATE * PD_SUSTAIN_MOVEMENT_MS + 999) / 1000;
const float PD_SUSTAIN_DEPTH_STEP =
    (PD_DEPTH - PD_SUSTAIN_DEPTH) / PD_SUSTAIN_MOVEMENT_TICKS;
const uint16_t PORTAMENTO_TIME_MS = 50;
const uint8_t PORTAMENTO_STEPS =
    ((uint32_t)MOZZI_CONTROL_RATE * PORTAMENTO_TIME_MS + 500) / 1000;
const uint16_t AMPLITUDE_MAX = 32767;
const uint32_t AMPLITUDE_MAX_Q8 = (uint32_t)AMPLITUDE_MAX << 8;
const uint32_t RELEASE_RAMP_SAMPLES =
    ((uint32_t)MOZZI_AUDIO_RATE * RELEASE_TIME_MS + 999) / 1000;
const uint32_t RELEASE_AMPLITUDE_STEP_Q8 =
    (AMPLITUDE_MAX_Q8 + RELEASE_RAMP_SAMPLES - 1) / RELEASE_RAMP_SAMPLES;
// Q8 EMA weight 37/256 at 64 Hz gives approximately a 100 ms time constant.
const uint8_t LDR_SMOOTHING_WEIGHT_Q8 = 37;
const uint8_t NOTE_STABILITY_SAMPLES = 2;
const uint8_t EVENT_CAPACITY = 64;
const uint8_t RECORD_INTERVAL_MS = 47;
const uint8_t LOOPER_DEBOUNCE_MS = 25;
const uint8_t NOTE_CONTROL_ASSOCIATION_TICKS = 4;
const uint16_t QUANTIZER_NO_TARGET = 0xFFFF;

enum LooperMode : uint8_t {
  LOOPER_NORMAL,
  LOOPER_RECORDING,
  LOOPER_PLAYBACK
};

enum RecordedControl : uint8_t {
  CONTROL_NOTE,
  CONTROL_GATE
};

enum QuantizerState : uint8_t {
  QUANTIZER_IDLE,
  QUANTIZER_SCORING,
  QUANTIZER_ADJUSTING,
  QUANTIZER_READY
};

struct ControlEvent {
  uint16_t tick;
  uint16_t controlAndValue;
};

static_assert(sizeof(ControlEvent) == 4, "ControlEvent must remain 4 bytes");

int32_t smoothedLdrQ8 = 0;
bool ldrSmootherInitialized = false;
uint8_t currentMidiNote = LOWEST_MIDI_NOTE;
uint8_t stableMidiNote = LOWEST_MIDI_NOTE;
uint8_t candidateMidiNote = LOWEST_MIDI_NOTE;
uint8_t candidateNoteSamples = 0;
float currentFrequency = 0.0f;
float portamentoStartFrequency = 0.0f;
float portamentoTargetFrequency = 0.0f;
uint8_t portamentoStep = PORTAMENTO_STEPS;
uint8_t gateDebounceHistory = 0x07;
bool gateOpen = false;
bool voiceGateOpen = false;
uint32_t gateAmplitudeQ8 = 0;
uint16_t heldControlTicks = 0;
float currentPdDepth = PD_DEPTH;
uint16_t currentAmplitudeAttackTimeMs = DEFAULT_AMPLITUDE_ATTACK_TIME_MS;
uint32_t liveAttackAmplitudeStepQ8 = 1;
uint32_t lastLiveNoteOnMs = 0;
uint32_t nextLiveAttackRecoveryMs = 0;
bool hasLiveNoteOn = false;
bool loopGateOpen = false;
bool loopVoiceGateOpen = false;
uint8_t loopMidiNote = LOWEST_MIDI_NOTE;
float loopCurrentFrequency = 0.0f;
float loopPortamentoStartFrequency = 0.0f;
float loopPortamentoTargetFrequency = 0.0f;
uint8_t loopPortamentoStep = PORTAMENTO_STEPS;
uint32_t loopGateAmplitudeQ8 = 0;
uint16_t loopHeldControlTicks = 0;
float loopCurrentPdDepth = PD_DEPTH;
uint16_t loopAmplitudeAttackTimeMs = DEFAULT_AMPLITUDE_ATTACK_TIME_MS;
uint32_t loopAttackAmplitudeStepQ8 = 1;
uint32_t lastLoopNoteOnMs = 0;
uint32_t nextLoopAttackRecoveryMs = 0;
bool hasLoopNoteOn = false;
ControlEvent events[EVENT_CAPACITY];
volatile LooperMode looperMode = LOOPER_NORMAL;
uint8_t eventCount = 0;
uint8_t playbackIndex = 0;
uint8_t lastRecordedMidiNote = LOWEST_MIDI_NOTE;
bool lastRecordedGate = false;
bool recordingFull = false;
uint16_t loopDurationTicks = 1;
uint32_t recordStartMs = 0;
uint32_t nextRecordSampleMs = 0;
uint32_t playbackStartMs = 0;
uint32_t playbackLoopEndMs = 0;
uint32_t nextPlaybackEventMs = 0;
bool lastLooperButtonReading = HIGH;
bool stableLooperButtonState = HIGH;
uint32_t looperDebounceStartMs = 0;
QuantizerState quantizerState = QUANTIZER_IDLE;
bool quantizedPlaybackActive = false;
uint16_t quantizerCandidateBars = 1;
uint16_t quantizerMaximumBars = 1;
uint16_t quantizerBestBars = 1;
uint8_t quantizerScoreEventIndex = 0;
uint8_t quantizerScoreCount = 0;
uint32_t quantizerScoreSum = 0;
uint32_t quantizerBestScore = 0xFFFFFFFFUL;
int16_t quantizerAdjustIndex = -1;
uint16_t quantizerNextEffectiveTick = 0;
uint16_t quantizerNextNoteOnGrid = QUANTIZER_NO_TARGET;
uint16_t quantizerPendingGateOffGrid = QUANTIZER_NO_TARGET;
uint16_t quantizerPendingPitchTarget = QUANTIZER_NO_TARGET;
uint16_t quantizerPendingGateTick = 0;
bool quantizerPendingPitch = false;

uint16_t smoothLdr(uint16_t input) {
  const int32_t targetQ8 = (int32_t)input << 8;
  if (!ldrSmootherInitialized) {
    smoothedLdrQ8 = targetQ8;
    ldrSmootherInitialized = true;
  } else {
    smoothedLdrQ8 +=
        ((targetQ8 - smoothedLdrQ8) * LDR_SMOOTHING_WEIGHT_Q8) >> 8;
  }
  return (smoothedLdrQ8 + 128) >> 8;
}

uint8_t midiNoteFromLdr(uint16_t ldrValue) {
  const uint8_t scaleIndex =
      ((uint32_t)ldrValue * SCALE_NOTE_COUNT) >> 10;
  const uint8_t octave = scaleIndex / SCALE_NOTES_PER_OCTAVE;
  const uint8_t scaleDegree = scaleIndex % SCALE_NOTES_PER_OCTAVE;
  const uint8_t semitoneOffset =
      pgm_read_byte(&C_MAJOR_OFFSETS_FROM_C[scaleDegree]);
  return LOWEST_MIDI_NOTE + octave * 12 + semitoneOffset;
}

uint8_t stabilizeMidiNote(uint8_t midiNote) {
  if (midiNote != candidateMidiNote) {
    candidateMidiNote = midiNote;
    candidateNoteSamples = 1;
  } else if (candidateNoteSamples < NOTE_STABILITY_SAMPLES) {
    ++candidateNoteSamples;
  }

  if (candidateNoteSamples >= NOTE_STABILITY_SAMPLES) {
    stableMidiNote = candidateMidiNote;
  }
  return stableMidiNote;
}

void recoverAmplitudeAttack(uint16_t &attackTimeMs,
                     uint32_t lastNoteOnMs, uint32_t &nextRecoveryMs,
                     bool hasNoteOn, uint32_t now) {
  if (!hasNoteOn || attackTimeMs >= DEFAULT_AMPLITUDE_ATTACK_TIME_MS
      || (uint32_t)(now - lastNoteOnMs) < RAPID_NOTE_WINDOW_MS) {
    return;
  }

  while (attackTimeMs < DEFAULT_AMPLITUDE_ATTACK_TIME_MS
         && (int32_t)(now - nextRecoveryMs) >= 0) {
    attackTimeMs += AMPLITUDE_ATTACK_STEP_MS;
    if (attackTimeMs > DEFAULT_AMPLITUDE_ATTACK_TIME_MS) {
      attackTimeMs = DEFAULT_AMPLITUDE_ATTACK_TIME_MS;
    }
    nextRecoveryMs += AMPLITUDE_ATTACK_RECOVERY_INTERVAL_MS;
  }
}

void registerAmplitudeNoteOn(uint16_t &attackTimeMs,
                      uint32_t &lastNoteOnMs,
                      uint32_t &nextRecoveryMs, bool &hasNoteOn,
                      uint32_t now) {
  if (hasNoteOn
      && (uint32_t)(now - lastNoteOnMs) < RAPID_NOTE_WINDOW_MS) {
    if (attackTimeMs
        > MIN_AMPLITUDE_ATTACK_TIME_MS + AMPLITUDE_ATTACK_STEP_MS) {
      attackTimeMs -= AMPLITUDE_ATTACK_STEP_MS;
    } else {
      attackTimeMs = MIN_AMPLITUDE_ATTACK_TIME_MS;
    }
  }
  lastNoteOnMs = now;
  nextRecoveryMs = now + RAPID_NOTE_WINDOW_MS;
  hasNoteOn = true;
}

uint32_t amplitudeStepForAttack(uint16_t attackTimeMs) {
  const uint32_t rampSamples =
      ((uint32_t)MOZZI_AUDIO_RATE * attackTimeMs + 999) / 1000;
  return (AMPLITUDE_MAX_Q8 + rampSamples - 1) / rampSamples;
}

void applyPdSettings() {
  voice.setPDEnv(PD_FILTER_ATTACK_TIME_MS, RELEASE_TIME_MS);
  voice.setAmplitudeAttack(currentAmplitudeAttackTimeMs);
  voice.setPDDepth(PD_DEPTH);
}

void triggerVoice() {
  registerAmplitudeNoteOn(currentAmplitudeAttackTimeMs,
                   lastLiveNoteOnMs, nextLiveAttackRecoveryMs,
                   hasLiveNoteOn, millis());
  liveAttackAmplitudeStepQ8 =
      amplitudeStepForAttack(currentAmplitudeAttackTimeMs);
  heldControlTicks = 0;
  currentPdDepth = PD_DEPTH;
  applyPdSettings();
  voice.noteOn(1, currentMidiNote, 127);
  currentFrequency = mtof((float)currentMidiNote);
  portamentoStartFrequency = currentFrequency;
  portamentoTargetFrequency = currentFrequency;
  portamentoStep = PORTAMENTO_STEPS;
}

void syncVoiceGate() {
  if (gateOpen == voiceGateOpen) return;

  if (gateOpen) {
    triggerVoice();
  } else {
    voice.noteOff(1, currentMidiNote, 0);
  }
  voiceGateOpen = gateOpen;
}

void updatePdSustainMovement() {
  if (!voiceGateOpen) return;

  if (heldControlTicks < PD_SUSTAIN_START_TICKS
      + PD_SUSTAIN_MOVEMENT_TICKS) {
    ++heldControlTicks;
  }

  if (heldControlTicks > PD_SUSTAIN_START_TICKS
      && currentPdDepth > PD_SUSTAIN_DEPTH) {
    currentPdDepth -= PD_SUSTAIN_DEPTH_STEP;
    if (currentPdDepth < PD_SUSTAIN_DEPTH) {
      currentPdDepth = PD_SUSTAIN_DEPTH;
    }
    voice.setPDDepth(currentPdDepth);
  }
}

void setPitchWithoutRetrigger(uint8_t midiNote) {
  if (midiNote != currentMidiNote) {
    currentMidiNote = midiNote;
    portamentoStartFrequency = currentFrequency;
    portamentoTargetFrequency = mtof((float)currentMidiNote);
    portamentoStep = 0;
  }
}

void updatePortamento() {
  if (!voiceGateOpen || portamentoStep >= PORTAMENTO_STEPS) return;

  ++portamentoStep;
  currentFrequency = portamentoStartFrequency
      + (portamentoTargetFrequency - portamentoStartFrequency)
        * portamentoStep / PORTAMENTO_STEPS;
  voice.setFrequency(currentFrequency);
}

void applyLoopPdSettings() {
  loopVoice.setPDEnv(PD_FILTER_ATTACK_TIME_MS, RELEASE_TIME_MS);
  loopVoice.setAmplitudeAttack(loopAmplitudeAttackTimeMs);
  loopVoice.setPDDepth(PD_DEPTH);
}

void triggerLoopVoice() {
  registerAmplitudeNoteOn(loopAmplitudeAttackTimeMs,
                   lastLoopNoteOnMs, nextLoopAttackRecoveryMs,
                   hasLoopNoteOn, millis());
  loopAttackAmplitudeStepQ8 =
      amplitudeStepForAttack(loopAmplitudeAttackTimeMs);
  loopHeldControlTicks = 0;
  loopCurrentPdDepth = PD_DEPTH;
  applyLoopPdSettings();
  loopVoice.noteOn(1, loopMidiNote, 127);
  loopCurrentFrequency = mtof((float)loopMidiNote);
  loopPortamentoStartFrequency = loopCurrentFrequency;
  loopPortamentoTargetFrequency = loopCurrentFrequency;
  loopPortamentoStep = PORTAMENTO_STEPS;
}

void syncLoopVoiceGate() {
  if (loopGateOpen == loopVoiceGateOpen) return;
  if (loopGateOpen) {
    triggerLoopVoice();
  } else {
    loopVoice.noteOff(1, loopMidiNote, 0);
  }
  loopVoiceGateOpen = loopGateOpen;
}

void setLoopPitchWithoutRetrigger(uint8_t midiNote) {
  if (midiNote != loopMidiNote) {
    loopMidiNote = midiNote;
    loopPortamentoStartFrequency = loopCurrentFrequency;
    loopPortamentoTargetFrequency = mtof((float)loopMidiNote);
    loopPortamentoStep = 0;
  }
}

void updateLoopPortamento() {
  if (!loopVoiceGateOpen || loopPortamentoStep >= PORTAMENTO_STEPS) return;
  ++loopPortamentoStep;
  loopCurrentFrequency = loopPortamentoStartFrequency
      + (loopPortamentoTargetFrequency - loopPortamentoStartFrequency)
        * loopPortamentoStep / PORTAMENTO_STEPS;
  loopVoice.setFrequency(loopCurrentFrequency);
}

void updateLoopPdSustainMovement() {
  if (!loopVoiceGateOpen) return;
  if (loopHeldControlTicks < PD_SUSTAIN_START_TICKS
      + PD_SUSTAIN_MOVEMENT_TICKS) {
    ++loopHeldControlTicks;
  }
  if (loopHeldControlTicks > PD_SUSTAIN_START_TICKS
      && loopCurrentPdDepth > PD_SUSTAIN_DEPTH) {
    loopCurrentPdDepth -= PD_SUSTAIN_DEPTH_STEP;
    if (loopCurrentPdDepth < PD_SUSTAIN_DEPTH) {
      loopCurrentPdDepth = PD_SUSTAIN_DEPTH;
    }
    loopVoice.setPDDepth(loopCurrentPdDepth);
  }
}

uint16_t encodeControlEvent(RecordedControl control, uint16_t value) {
  return ((uint16_t)control << 10) | (value & 0x03FF);
}

RecordedControl eventControl(const ControlEvent &event) {
  return (RecordedControl)((event.controlAndValue >> 10) & 0x03);
}

int8_t eventTimingNudge(const ControlEvent &event) {
  int8_t nudge = (event.controlAndValue >> 12) & 0x0F;
  if (nudge >= 8) nudge -= 16;
  return nudge;
}

void setEventTimingNudge(ControlEvent &event, int8_t nudge) {
  if (nudge < -8) nudge = -8;
  if (nudge > 7) nudge = 7;
  event.controlAndValue = (event.controlAndValue & 0x0FFF)
      | ((uint16_t)(nudge & 0x0F) << 12);
}

uint16_t eventPlaybackTick(const ControlEvent &event) {
  int32_t tick = event.tick;
  if (quantizedPlaybackActive) tick += eventTimingNudge(event);
  if (tick < 0) return 0;
  if (tick >= loopDurationTicks) return loopDurationTicks - 1;
  return tick;
}

bool addControlEvent(uint16_t tick, RecordedControl control, uint16_t value) {
  if (eventCount >= EVENT_CAPACITY) {
    recordingFull = true;
    return false;
  }
  events[eventCount].tick = tick;
  events[eventCount].controlAndValue = encodeControlEvent(control, value);
  ++eventCount;
  return true;
}

void captureInitialControlState() {
  lastRecordedMidiNote = currentMidiNote;
  lastRecordedGate = gateOpen;
  addControlEvent(0, CONTROL_NOTE, lastRecordedMidiNote);
  addControlEvent(0, CONTROL_GATE, lastRecordedGate);
}

void startRecording() {
  loopGateOpen = false;
  eventCount = 0;
  recordingFull = false;
  recordStartMs = millis();
  nextRecordSampleMs = recordStartMs + RECORD_INTERVAL_MS;
  captureInitialControlState();
  looperMode = LOOPER_RECORDING;
  quantizerState = QUANTIZER_IDLE;
  quantizedPlaybackActive = false;
}

void sampleControlChanges(uint32_t now) {
  if (recordingFull || (int32_t)(now - nextRecordSampleMs) < 0) return;
  nextRecordSampleMs += RECORD_INTERVAL_MS;
  if ((int32_t)(now - nextRecordSampleMs) >= 0) {
    nextRecordSampleMs = now + RECORD_INTERVAL_MS;
  }
  const uint32_t elapsedTicks = (now - recordStartMs) / RECORD_INTERVAL_MS;
  if (elapsedTicks > 0xFFFF) {
    recordingFull = true;
    return;
  }
  const uint16_t tick = elapsedTicks;
  if (currentMidiNote != lastRecordedMidiNote) {
    lastRecordedMidiNote = currentMidiNote;
    addControlEvent(tick, CONTROL_NOTE, lastRecordedMidiNote);
  }
  if (gateOpen != lastRecordedGate) {
    lastRecordedGate = gateOpen;
    addControlEvent(tick, CONTROL_GATE, lastRecordedGate);
  }
}

void applyControlEvent(const ControlEvent &event) {
  const uint16_t value = event.controlAndValue & 0x03FF;
  if (eventControl(event) == CONTROL_NOTE) {
    setLoopPitchWithoutRetrigger((uint8_t)value);
  } else {
    loopGateOpen = value != 0;
  }
}

uint16_t quantizerSubdivisions(uint16_t bars) {
  return bars * 16;
}

uint16_t nearestGridIndex(uint16_t tick, uint16_t bars) {
  const uint16_t subdivisions = quantizerSubdivisions(bars);
  uint16_t gridIndex = ((uint32_t)tick * subdivisions
      + loopDurationTicks / 2) / loopDurationTicks;
  if (gridIndex >= subdivisions) gridIndex = subdivisions - 1;
  return gridIndex;
}

uint16_t tickAtGridIndex(uint16_t gridIndex, uint16_t bars) {
  const uint16_t subdivisions = quantizerSubdivisions(bars);
  uint32_t tick = ((uint32_t)gridIndex * loopDurationTicks
      + subdivisions / 2) / subdivisions;
  if (tick >= loopDurationTicks) tick = loopDurationTicks - 1;
  return tick;
}

void beginLoopQuantization() {
  const uint32_t durationMs =
      (uint32_t)loopDurationTicks * RECORD_INTERVAL_MS;
  uint16_t minimumBars = (durationMs + 3999UL) / 4000UL;
  uint16_t maximumBars = durationMs / 1500UL;
  if (minimumBars < 1) minimumBars = 1;
  if (maximumBars < minimumBars) {
    minimumBars = (durationMs + 1000UL) / 2000UL;
    if (minimumBars < 1) minimumBars = 1;
    maximumBars = minimumBars;
  }
  quantizerCandidateBars = minimumBars;
  quantizerMaximumBars = maximumBars;
  quantizerBestBars = minimumBars;
  quantizerScoreEventIndex = 0;
  quantizerScoreCount = 0;
  quantizerScoreSum = 0;
  quantizerBestScore = 0xFFFFFFFFUL;
  quantizerState = QUANTIZER_SCORING;
  quantizedPlaybackActive = false;
}

void finishQuantizerCandidate() {
  const uint32_t timingScore = quantizerScoreCount
      ? quantizerScoreSum / quantizerScoreCount : 0;
  const uint16_t bpm = ((uint32_t)quantizerCandidateBars * 240000UL
      + ((uint32_t)loopDurationTicks * RECORD_INTERVAL_MS) / 2)
      / ((uint32_t)loopDurationTicks * RECORD_INTERVAL_MS);
  const uint8_t tempoPrior = (abs((int)bpm - 120) + 7) / 8;
  const uint32_t score = timingScore + tempoPrior;
  if (score < quantizerBestScore) {
    quantizerBestScore = score;
    quantizerBestBars = quantizerCandidateBars;
  }
  if (quantizerCandidateBars < quantizerMaximumBars) {
    ++quantizerCandidateBars;
    quantizerScoreEventIndex = 0;
    quantizerScoreCount = 0;
    quantizerScoreSum = 0;
    return;
  }
  quantizerAdjustIndex = eventCount - 1;
  quantizerNextEffectiveTick = loopDurationTicks - 1;
  quantizerNextNoteOnGrid = QUANTIZER_NO_TARGET;
  quantizerPendingGateOffGrid = QUANTIZER_NO_TARGET;
  quantizerPendingPitchTarget = QUANTIZER_NO_TARGET;
  quantizerPendingPitch = false;
  quantizerState = QUANTIZER_ADJUSTING;
}

void scoreOneQuantizerEvent() {
  if (quantizerScoreEventIndex >= eventCount) {
    finishQuantizerCandidate();
    return;
  }
  const ControlEvent &event = events[quantizerScoreEventIndex++];
  if (event.tick == 0 || eventControl(event) != CONTROL_GATE) return;
  const uint16_t gridIndex =
      nearestGridIndex(event.tick, quantizerCandidateBars);
  const uint16_t targetTick =
      tickAtGridIndex(gridIndex, quantizerCandidateBars);
  const uint16_t error = abs((int)targetTick - (int)event.tick);
  quantizerScoreSum += ((uint32_t)error
      * quantizerSubdivisions(quantizerCandidateBars) * 256UL)
      / loopDurationTicks;
  ++quantizerScoreCount;
}

void adjustOneQuantizerEvent() {
  if (quantizerAdjustIndex < 0) {
    quantizerState = QUANTIZER_READY;
    return;
  }
  ControlEvent &event = events[quantizerAdjustIndex];
  const RecordedControl control = eventControl(event);
  uint16_t desiredTick = event.tick;
  if (event.tick > 0 && control == CONTROL_GATE) {
    uint16_t gridIndex = nearestGridIndex(event.tick, quantizerBestBars);
    const bool gateOn = (event.controlAndValue & 0x03FF) != 0;
    if (gateOn) {
      if (quantizerPendingGateOffGrid != QUANTIZER_NO_TARGET
          && gridIndex >= quantizerPendingGateOffGrid) {
        gridIndex = quantizerPendingGateOffGrid == 0
            ? 0 : quantizerPendingGateOffGrid - 1;
      }
      if (quantizerNextNoteOnGrid != QUANTIZER_NO_TARGET
          && gridIndex >= quantizerNextNoteOnGrid) {
        gridIndex = quantizerNextNoteOnGrid == 0
            ? 0 : quantizerNextNoteOnGrid - 1;
      }
      quantizerNextNoteOnGrid = gridIndex;
      quantizerPendingGateOffGrid = QUANTIZER_NO_TARGET;
      quantizerPendingPitchTarget =
          tickAtGridIndex(gridIndex, quantizerBestBars);
      quantizerPendingGateTick = event.tick;
      quantizerPendingPitch = true;
    } else {
      quantizerPendingGateOffGrid = gridIndex;
    }
    desiredTick = tickAtGridIndex(gridIndex, quantizerBestBars);
  } else if (event.tick > 0 && control == CONTROL_NOTE
             && quantizerPendingPitch) {
    if (quantizerPendingGateTick >= event.tick
        && quantizerPendingGateTick - event.tick
            <= NOTE_CONTROL_ASSOCIATION_TICKS) {
      desiredTick = quantizerPendingPitchTarget;
    }
    quantizerPendingPitch = false;
  }
  const uint16_t lowerBound = quantizerAdjustIndex > 0
      ? events[quantizerAdjustIndex - 1].tick : 0;
  if (desiredTick < lowerBound) desiredTick = lowerBound;
  if (desiredTick > quantizerNextEffectiveTick) {
    desiredTick = quantizerNextEffectiveTick;
  }
  int16_t nudge = (int16_t)desiredTick - event.tick;
  if (nudge < -8) nudge = -8;
  if (nudge > 7) nudge = 7;
  setEventTimingNudge(event, nudge);
  quantizerNextEffectiveTick = event.tick + nudge;
  --quantizerAdjustIndex;
}

void serviceLoopQuantizer() {
  if (quantizerState == QUANTIZER_SCORING) {
    scoreOneQuantizerEvent();
  } else if (quantizerState == QUANTIZER_ADJUSTING) {
    adjustOneQuantizerEvent();
  }
}

void scheduleNextPlaybackEvent() {
  if (playbackIndex < eventCount) {
    nextPlaybackEventMs = playbackStartMs
        + (uint32_t)eventPlaybackTick(events[playbackIndex])
          * RECORD_INTERVAL_MS;
  } else {
    nextPlaybackEventMs = playbackLoopEndMs;
  }
}

void startPlayback() {
  uint32_t durationTicks =
      (millis() - recordStartMs) / RECORD_INTERVAL_MS;
  if (durationTicks < 1) durationTicks = 1;
  if (durationTicks > 0xFFFF) durationTicks = 0xFFFF;
  loopDurationTicks = durationTicks;
  playbackIndex = 0;
  playbackStartMs = millis();
  playbackLoopEndMs = playbackStartMs
      + (uint32_t)loopDurationTicks * RECORD_INTERVAL_MS;
  looperMode = LOOPER_PLAYBACK;
  beginLoopQuantization();
  scheduleNextPlaybackEvent();
}

void servicePlayback(uint32_t now) {
  const uint32_t durationMs =
      (uint32_t)loopDurationTicks * RECORD_INTERVAL_MS;
  while ((int32_t)(now - playbackLoopEndMs) >= 0) {
    playbackStartMs = playbackLoopEndMs;
    playbackLoopEndMs += durationMs;
    if (!quantizedPlaybackActive && quantizerState == QUANTIZER_READY) {
      quantizedPlaybackActive = true;
    }
    playbackIndex = 0;
    scheduleNextPlaybackEvent();
  }
  while (playbackIndex < eventCount
         && (int32_t)(now - nextPlaybackEventMs) >= 0) {
    applyControlEvent(events[playbackIndex]);
    ++playbackIndex;
    scheduleNextPlaybackEvent();
  }
}

void stopPlayback() {
  looperMode = LOOPER_NORMAL;
  loopGateOpen = false;
  quantizerState = QUANTIZER_IDLE;
  quantizedPlaybackActive = false;
}

void advanceLooperMode() {
  if (looperMode == LOOPER_NORMAL) {
    startRecording();
  } else if (looperMode == LOOPER_RECORDING) {
    startPlayback();
  } else {
    stopPlayback();
  }
}

void serviceLooperButton(uint32_t now) {
  const bool reading = digitalRead(LOOPER_BUTTON_PIN);
  if (reading != lastLooperButtonReading) {
    lastLooperButtonReading = reading;
    looperDebounceStartMs = now;
  }
  if ((uint32_t)(now - looperDebounceStartMs) >= LOOPER_DEBOUNCE_MS
      && reading != stableLooperButtonState) {
    stableLooperButtonState = reading;
    if (stableLooperButtonState == LOW) advanceLooperMode();
  }
}

void serviceLooper() {
  const uint32_t now = millis();
  serviceLooperButton(now);
  if (looperMode == LOOPER_RECORDING) {
    sampleControlChanges(now);
  } else if (looperMode == LOOPER_PLAYBACK) {
    servicePlayback(now);
  }
}

void setup() {
  pinMode(GATE_PIN, INPUT_PULLUP);
  pinMode(LOOPER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, LED_MIN_PWM);
  startMozzi();
}

void updateControl() {
  const uint32_t now = millis();
  recoverAmplitudeAttack(currentAmplitudeAttackTimeMs,
                  lastLiveNoteOnMs, nextLiveAttackRecoveryMs,
                  hasLiveNoteOn, now);
  recoverAmplitudeAttack(loopAmplitudeAttackTimeMs,
                  lastLoopNoteOnMs, nextLoopAttackRecoveryMs,
                  hasLoopNoteOn, now);

  gateDebounceHistory =
      ((gateDebounceHistory << 1) | digitalRead(GATE_PIN)) & 0x07;
  if (gateDebounceHistory == 0) {
    gateOpen = true;
  } else if (gateDebounceHistory == 0x07) {
    gateOpen = false;
  }

  const uint16_t ldrValue = smoothLdr(mozziAnalogRead<10>(LDR_PIN));
  setPitchWithoutRetrigger(
      stabilizeMidiNote(midiNoteFromLdr(ldrValue)));

  serviceLooper();
  serviceLoopQuantizer();

  const uint8_t liveEnvelopeAmplitude = voice.getAmplitude();
  const uint8_t loopEnvelopeAmplitude = loopVoice.getAmplitude();
  const uint8_t envelopeAmplitude =
      liveEnvelopeAmplitude > loopEnvelopeAmplitude
      ? liveEnvelopeAmplitude : loopEnvelopeAmplitude;
  const uint8_t ledPwm = LED_MIN_PWM
      + ((uint16_t)envelopeAmplitude * LED_PWM_RANGE >> 8);
  analogWrite(LED_PIN, ledPwm);

  syncVoiceGate();
  updatePortamento();
  updatePdSustainMovement();
  voice.update();
  syncLoopVoiceGate();
  updateLoopPortamento();
  updateLoopPdSustainMovement();
  loopVoice.update();
}

AudioOutput updateAudio() {
  if (gateOpen) {
    if (gateAmplitudeQ8 < AMPLITUDE_MAX_Q8 - liveAttackAmplitudeStepQ8) {
      gateAmplitudeQ8 += liveAttackAmplitudeStepQ8;
    } else {
      gateAmplitudeQ8 = AMPLITUDE_MAX_Q8;
    }
  } else {
    if (gateAmplitudeQ8 > RELEASE_AMPLITUDE_STEP_Q8) {
      gateAmplitudeQ8 -= RELEASE_AMPLITUDE_STEP_Q8;
    } else {
      gateAmplitudeQ8 = 0;
    }
  }

  if (loopGateOpen) {
    if (loopGateAmplitudeQ8
        < AMPLITUDE_MAX_Q8 - loopAttackAmplitudeStepQ8) {
      loopGateAmplitudeQ8 += loopAttackAmplitudeStepQ8;
    } else {
      loopGateAmplitudeQ8 = AMPLITUDE_MAX_Q8;
    }
  } else {
    if (loopGateAmplitudeQ8 > RELEASE_AMPLITUDE_STEP_Q8) {
      loopGateAmplitudeQ8 -= RELEASE_AMPLITUDE_STEP_Q8;
    } else {
      loopGateAmplitudeQ8 = 0;
    }
  }

  const uint16_t gateAmplitude = gateAmplitudeQ8 >> 8;
  const uint16_t loopGateAmplitude = loopGateAmplitudeQ8 >> 8;
  const bool liveActive = BYPASS_VOLUME_ENVELOPES
      ? voiceGateOpen || voice.getAmplitude() > 0
      : gateAmplitude > 0;
  const bool loopActive = BYPASS_VOLUME_ENVELOPES
      ? loopVoiceGateOpen || loopVoice.getAmplitude() > 0
      : loopGateAmplitude > 0;
  const int16_t liveOutput = !liveActive ? 0
      : BYPASS_VOLUME_ENVELOPES ? voice.nextUnenveloped()
      : ((int32_t)voice.next() * gateAmplitude) >> 15;
  const int16_t loopOutput = !loopActive ? 0
      : BYPASS_VOLUME_ENVELOPES ? loopVoice.nextUnenveloped()
      : ((int32_t)loopVoice.next() * loopGateAmplitude) >> 15;
  const int16_t mixedOutput = liveActive && loopActive
      ? (liveOutput + loopOutput) >> 1
      : liveOutput + loopOutput;
  return MonoOutput::from8Bit(mixedOutput);
}

void loop() {
  audioHook();
}
