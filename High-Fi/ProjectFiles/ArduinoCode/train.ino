/*
 * Train Music Player
 * Board: Arduino Nano 33 BLE Sense Rev2
 *
 * When the on-board IMU detects the "train" moving, this plays one of several
 * preloaded melodies through a passive buzzer. Use the buttons to switch songs
 * and to arm/disarm motion sensing. A 10 k potentiometer wired in series with
 * the buzzer gives you an analog volume knob.
 *
 * REQUIRED LIBRARY
 *   Install "Arduino_BMI270_BMM150" from the Library Manager.
 *
 * HARDWARE
 *   D3 -> Passive buzzer (+)
 *   Buzzer (-) -> Potentiometer wiper
 *   Potentiometer outer pin -> GND   (this gives hardware volume control)
 *   D4 -> Button 1 (song switch) -> GND    (uses INPUT_PULLUP)
 *   D5 -> Button 2 (arm / disarm) -> GND   (uses INPUT_PULLUP)
 *   D6 -> 220 ohm -> LED anode -> LED cathode -> GND
 *
 * SONGS
 *   0: Twinkle Twinkle            (red)
 *   1: Tetris (Korobeiniki, loops)(green)
 *   2: Super Mario Bros Overworld (blue)
 *   3: Jingle Bells (full chorus) (yellow)
 *   4: Für Elise (complete)       (magenta)
 *   5: Torrent (Chopin Op.10 No.4)(cyan)
 *
 * INDICATORS
 *   External LED (D6):      off = disarmed, solid = armed & still, blinking = playing
 *   On-board RGB LED:       current song  (see colors above)
 *   On-board orange LED:    mirrors the external LED, useful for debugging
 */

#include <Arduino_BMI270_BMM150.h>
#include <math.h>

#ifndef IMU
#define IMU IMU_BMI270_BMM150
#endif

// ==================== PINS ====================
const int BUZZER_PIN      = 3;
const int BUTTON_SONG_PIN = 4;
const int BUTTON_ARM_PIN  = 5;
const int STATUS_LED_PIN  = 6;

// ==================== NOTE FREQUENCIES ====================
#define NOTE_REST 0
#define NOTE_A3   220
#define NOTE_C4   262
#define NOTE_D4   294
#define NOTE_E4   330
#define NOTE_F4   349
#define NOTE_FS4  370     // F#4
#define NOTE_G4   392
#define NOTE_GS4  415     // G#4
#define NOTE_A4   440
#define NOTE_AS4  466     // A#4 / Bb4
#define NOTE_B4   494
#define NOTE_C5   523
#define NOTE_CS5  554     // C#5
#define NOTE_D5   587
#define NOTE_DS5  622     // D#5
#define NOTE_E5   659
#define NOTE_F5   698
#define NOTE_FS5  740     // F#5
#define NOTE_G5   784
#define NOTE_GS5  831     // G#5
#define NOTE_A5   880
#define NOTE_AS5  932     // A#5 / Bb5
#define NOTE_B5   988
#define NOTE_C6   1047
#define NOTE_D6   1175
#define NOTE_E6   1319

// ==================== SONG DATA ====================
// Durations: 1 = whole note, 2 = half, 4 = quarter, 8 = eighth,
//            16 = sixteenth, 32 = thirty-second.
// Negative values = dotted notes (e.g. -8 = dotted eighth = 1.5x eighth).

// Song 0: Twinkle Twinkle Little Star
const int song0_notes[] = {
  NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
  NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4
};
const int song0_durs[]  = { 4,4,4,4,4,4,2, 4,4,4,4,4,4,2 };

// Song 1: Tetris theme (Korobeiniki)
const int song1_notes[] = {
  NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4,
  NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_C5, NOTE_A4, NOTE_A4, NOTE_REST,
  NOTE_REST, NOTE_D5, NOTE_F5, NOTE_A5, NOTE_G5, NOTE_F5,
  NOTE_E5, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_C5, NOTE_A4, NOTE_A4, NOTE_REST,
  NOTE_E5, NOTE_C5,
  NOTE_D5, NOTE_B4,
  NOTE_C5, NOTE_A4,
  NOTE_B4,
  NOTE_E5, NOTE_C5,
  NOTE_D5, NOTE_B4,
  NOTE_C5, NOTE_E5, NOTE_A5,
  NOTE_GS5,
  NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4,
  NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_C5, NOTE_A4, NOTE_A4, NOTE_REST,
  NOTE_REST, NOTE_D5, NOTE_F5, NOTE_A5, NOTE_G5, NOTE_F5,
  NOTE_REST, NOTE_E5, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_REST, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_REST, NOTE_C5, NOTE_A4, NOTE_A4, NOTE_REST
};
const int song1_durs[] = {
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  -4, 8, 4, 4,
  4, 4, 4, 4,
  8, 4, 8, 4, 8, 8,
  -4, 8, 4, 8, 8,
  4, 8, 8, 4, 4,
  4, 4, 4, 4,
  2, 2,
  2, 2,
  2, 2,
  1,
  2, 2,
  2, 2,
  4, 4, 2,
  1,
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  -4, 8, 4, 4,
  4, 4, 4, 4,
  8, 4, 8, 4, 8, 8,
  8, 4, 8, 4, 8, 8,
  8, 4, 8, 4, 4,
  8, 4, 8, 4, 4
};

// Song 2: Super Mario Bros Overworld theme
const int song2_notes[] = {
  // Intro motif (bar 1-2) with the correct 8th note delay added
  NOTE_E5, NOTE_E5, NOTE_REST, NOTE_E5, NOTE_REST, NOTE_C5, NOTE_E5, NOTE_REST,
  NOTE_G5, NOTE_REST, NOTE_G4, NOTE_REST,
  // Main melody (bar 3-6)
  NOTE_C5, NOTE_G4, NOTE_REST, NOTE_E4,
  NOTE_A4, NOTE_B4, NOTE_AS4, NOTE_A4,
  NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5, NOTE_F5, NOTE_G5,
  NOTE_REST, NOTE_E5, NOTE_C5, NOTE_D5, NOTE_B4,
  // Repeat of bars 3-6
  NOTE_C5, NOTE_G4, NOTE_REST, NOTE_E4,
  NOTE_A4, NOTE_B4, NOTE_AS4, NOTE_A4,
  NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5, NOTE_F5, NOTE_G5,
  NOTE_REST, NOTE_E5, NOTE_C5, NOTE_D5, NOTE_B4,
  // Underworld-flavored answer phrase completely restored (bars 7-10)
  NOTE_REST, NOTE_G5, NOTE_FS5, NOTE_F5, NOTE_DS5, NOTE_E5,
  NOTE_REST, NOTE_GS4, NOTE_A4, NOTE_C5, NOTE_REST, NOTE_A4, NOTE_C5, NOTE_D5,
  NOTE_REST, NOTE_G5, NOTE_FS5, NOTE_F5, NOTE_DS5, NOTE_E5,
  NOTE_REST, NOTE_C6, NOTE_C6, NOTE_C6, NOTE_REST,
  NOTE_REST, NOTE_G5, NOTE_FS5, NOTE_F5, NOTE_DS5, NOTE_E5,
  NOTE_REST, NOTE_GS4, NOTE_A4, NOTE_C5, NOTE_REST, NOTE_A4, NOTE_C5, NOTE_D5,
  NOTE_REST, NOTE_DS5, NOTE_REST, NOTE_D5,
  NOTE_C5, NOTE_REST,
  // Bouncy C-D-E phrase (bar 11-14)
  NOTE_C5, NOTE_C5, NOTE_C5, NOTE_REST, NOTE_C5, NOTE_D5,
  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_G4,
  NOTE_C5, NOTE_C5, NOTE_C5, NOTE_REST, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_REST,
  NOTE_C5, NOTE_C5, NOTE_C5, NOTE_REST, NOTE_C5, NOTE_D5,
  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_G4,
  // Return of intro motif (bar 17-18)
  NOTE_E5, NOTE_E5, NOTE_REST, NOTE_E5, NOTE_REST, NOTE_C5, NOTE_E5, NOTE_REST,
  NOTE_G5, NOTE_REST, NOTE_G4, NOTE_REST,
  // Main melody again (bar 19-22)
  NOTE_C5, NOTE_G4, NOTE_REST, NOTE_E4,
  NOTE_A4, NOTE_B4, NOTE_AS4, NOTE_A4,
  NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5, NOTE_F5, NOTE_G5,
  NOTE_REST, NOTE_E5, NOTE_C5, NOTE_D5, NOTE_B4,
  NOTE_C5, NOTE_G4, NOTE_REST, NOTE_E4,
  NOTE_A4, NOTE_B4, NOTE_AS4, NOTE_A4,
  NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5, NOTE_F5, NOTE_G5,
  NOTE_REST, NOTE_E5, NOTE_C5, NOTE_D5, NOTE_B4,
  // Star power / F-major section (bar 23-26)
  NOTE_E5, NOTE_C5, NOTE_G4, NOTE_REST, NOTE_GS4,
  NOTE_A4, NOTE_F5, NOTE_F5, NOTE_A4,
  NOTE_D5, NOTE_A5, NOTE_A5, NOTE_A5, NOTE_G5, NOTE_F5,
  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_G4,
  NOTE_E5, NOTE_C5, NOTE_G4, NOTE_REST, NOTE_GS4,
  NOTE_A4, NOTE_F5, NOTE_F5, NOTE_A4,
  NOTE_B4, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_E5, NOTE_D5,
  NOTE_C5, NOTE_E4, NOTE_E4, NOTE_C4,
  // Repeat of star power section
  NOTE_E5, NOTE_C5, NOTE_G4, NOTE_REST, NOTE_GS4,
  NOTE_A4, NOTE_F5, NOTE_F5, NOTE_A4,
  NOTE_D5, NOTE_A5, NOTE_A5, NOTE_A5, NOTE_G5, NOTE_F5,
  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_G4,
  NOTE_E5, NOTE_C5, NOTE_G4, NOTE_REST, NOTE_GS4,
  NOTE_A4, NOTE_F5, NOTE_F5, NOTE_A4,
  NOTE_B4, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_E5, NOTE_D5,
  NOTE_C5, NOTE_E4, NOTE_E4, NOTE_C4,
  // Bouncy C-D-E phrase return
  NOTE_C5, NOTE_C5, NOTE_C5, NOTE_REST, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_REST,
  NOTE_C5, NOTE_C5, NOTE_C5, NOTE_REST, NOTE_C5, NOTE_D5,
  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_G4,
  NOTE_E5, NOTE_E5, NOTE_REST, NOTE_E5, NOTE_REST, NOTE_C5, NOTE_E5, NOTE_REST,
  NOTE_G5, NOTE_REST, NOTE_G4, NOTE_REST,
  // Final star power section (bar 37-44)
  NOTE_E5, NOTE_C5, NOTE_G4, NOTE_REST, NOTE_GS4,
  NOTE_A4, NOTE_F5, NOTE_F5, NOTE_A4,
  NOTE_D5, NOTE_A5, NOTE_A5, NOTE_A5, NOTE_G5, NOTE_F5,
  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_G4,
  NOTE_E5, NOTE_C5, NOTE_G4, NOTE_REST, NOTE_GS4,
  NOTE_A4, NOTE_F5, NOTE_F5, NOTE_A4,
  NOTE_B4, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_E5, NOTE_D5,
  NOTE_C5, NOTE_E4, NOTE_E4, NOTE_C4
};

const int song2_durs[] = {
  // Intro motif
  8, 8, 8, 8, 8, 8, 8, 8,
  4, 4, 8, 4,
  // Main melody
  -4, 8, 4, -4,
  4, 4, 8, 4,
  -8, -8, -8, 4, 8, 8,
  8, 4, 8, 8, -4,
  // Repeat of main melody
  -4, 8, 4, -4,
  4, 4, 8, 4,
  -8, -8, -8, 4, 8, 8,
  8, 4, 8, 8, -4,
  // Underworld
  4, 8, 8, 8, 4, 8,
  8, 8, 8, 8, 8, 8, 8, 8,
  4, 8, 8, 8, 4, 8,
  8, 4, 4, 4, 8,
  4, 8, 8, 8, 4, 8,
  8, 8, 8, 8, 8, 8, 8, 8,
  4, 4, 8, -4,
  2, 2,
  // Bouncy C-D-E
  8, 4, 8, 8, 8, 4,
  8, 4, 8, 2,
  8, 4, 8, 8, 8, 8, 8,
  1,
  8, 4, 8, 8, 8, 4,
  8, 4, 8, 2,
  // Return of intro motif
  8, 8, 8, 8, 8, 8, 8, 8,
  4, 4, 4, 4,
  // Main melody again
  -4, 8, 4, -4,
  4, 4, 8, 4,
  -8, -8, -8, 4, 8, 8,
  8, 4, 8, 8, -4,
  -4, 8, 4, -4,
  4, 4, 8, 4,
  -8, -8, -8, 4, 8, 8,
  8, 4, 8, 8, -4,
  // Star power / F-major section
  8, 4, 8, 4, 4,
  8, 4, 8, 2,
  -8, -8, -8, -8, -8, -8,
  8, 4, 8, 2,
  8, 4, 8, 4, 4,
  8, 4, 8, 2,
  8, 4, 8, -8, -8, -8,
  8, 4, 8, 2,
  // Repeat of star power
  8, 4, 8, 4, 4,
  8, 4, 8, 2,
  -8, -8, -8, -8, -8, -8,
  8, 4, 8, 2,
  8, 4, 8, 4, 4,
  8, 4, 8, 2,
  8, 4, 8, -8, -8, -8,
  8, 4, 8, 2,
  // Bouncy C-D-E return
  8, 4, 8, 8, 8, 8, 8,
  1,
  8, 4, 8, 8, 8, 4,
  8, 4, 8, 2,
  8, 8, 8, 8, 8, 8, 8, 8,
  4, 4, 4, 4,
  // Final star power section
  8, 4, 8, 4, 4,
  8, 4, 8, 2,
  -8, -8, -8, -8, -8, -8,
  8, 4, 8, 2,
  8, 4, 8, 4, 4,
  8, 4, 8, 2,
  8, 4, 8, -8, -8, -8,
  8, 4, 8, 2
};

// Song 3: Jingle Bells
const int song3_notes[] = {
  NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_F5, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_D5, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_G5,
  NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_F5, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_G5, NOTE_G5, NOTE_F5, NOTE_D5, NOTE_C5
};
const int song3_durs[] = {
  4, 4, 2, 4, 4, 2, 4, 4, 4, 4, 1,
  4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2,
  4, 4, 2, 4, 4, 2, 4, 4, 4, 4, 1,
  4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1
};

// Song 4: Für Elise
const int song4_notes[] = {
  NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_REST, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_REST, NOTE_E4, NOTE_GS4, NOTE_B4, NOTE_C5, NOTE_REST, NOTE_E4, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_REST, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_REST, NOTE_E4, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_REST,
  NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_REST, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_REST, NOTE_E4, NOTE_GS4, NOTE_B4, NOTE_C5, NOTE_REST, NOTE_E4, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_REST, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_REST, NOTE_E4, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_REST,
  NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_G4, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_F4, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_E4, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_REST,
  NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_REST, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_REST, NOTE_E4, NOTE_GS4, NOTE_B4, NOTE_C5, NOTE_REST, NOTE_E4, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_REST, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_REST, NOTE_E4, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_REST,
  NOTE_C5, NOTE_G5, NOTE_G4, NOTE_G5, NOTE_A4, NOTE_G5, NOTE_B4, NOTE_G5, NOTE_C5, NOTE_G5, NOTE_D5, NOTE_G5, NOTE_E5, NOTE_G5, NOTE_C6, NOTE_B5, NOTE_A5, NOTE_G5, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_G5, NOTE_F5, NOTE_D5, NOTE_C5, NOTE_G5, NOTE_G4, NOTE_G5, NOTE_A4, NOTE_G5, NOTE_B4, NOTE_G5, NOTE_C5, NOTE_G5, NOTE_D5, NOTE_G5, NOTE_E5, NOTE_G5, NOTE_C6, NOTE_B5, NOTE_A5, NOTE_G5, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_G5, NOTE_F5, NOTE_D5,
  NOTE_E5, NOTE_F5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_E5, NOTE_REST,
  NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_REST, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_REST, NOTE_E4, NOTE_GS4, NOTE_B4, NOTE_C5, NOTE_REST, NOTE_E4, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_REST, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_REST, NOTE_E4, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_REST,
  NOTE_A3, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_A5, NOTE_C6, NOTE_E6, NOTE_D6, NOTE_C6, NOTE_B5, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_A5, NOTE_C6, NOTE_E6, NOTE_D6, NOTE_C6, NOTE_B5, NOTE_AS5, NOTE_A5, NOTE_GS5, NOTE_G5, NOTE_FS5, NOTE_F5, NOTE_E5, NOTE_DS5, NOTE_D5, NOTE_CS5, NOTE_C5, NOTE_B4, NOTE_AS4, NOTE_A4, NOTE_GS4, NOTE_G4, NOTE_FS4, NOTE_F4,
  NOTE_E4, NOTE_GS4, NOTE_B4, NOTE_C5, NOTE_REST,
  NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_REST, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_REST, NOTE_E4, NOTE_GS4, NOTE_B4, NOTE_C5, NOTE_REST, NOTE_E4, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_REST, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_REST, NOTE_E4, NOTE_C5, NOTE_B4, NOTE_A4
};
const int song4_durs[] = {
  16, 16, 16, 16, 16, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 16, 16, 16, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 4, 4,
  16, 16, 16, 16, 16, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 16, 16, 16, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 4, 4,
  16, 16, 16, -8, 16, 16, 16, -8, 16, 16, 16, -8, 16, 16, 16, -8, 4,
  16, 16, 16, 16, 16, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 16, 16, 16, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 4, 4,
  32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
  32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, -8, 16, 16, 16, -8, 16, 16, 16,
  16, 16, 16, 16, 16, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 16, 16, 16, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 4, 4,
  -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32,
  16, 16, 16, -8, 4,
  16, 16, 16, 16, 16, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 16, 16, 16, 16, 16, 16, 4, 8, 16, 16, 16, 4, 8, 16, 16, 16, 2
};


// ==================== SONG 5 (TORRENT) DATA & ENGINE ====================
// Arrays populated by the runtime parser to save us from writing 1300 notes
#define MAX_TORRENT_NOTES 1500
int torrent_notes[MAX_TORRENT_NOTES];
int torrent_durs[MAX_TORRENT_NOTES];

const char* const torrent_RH[] = {
  // 1
  "6|----C------------------C--",
  "5|GFeDCDcCDeCDeFDeFGeFGaG-FG",
  "4|G-------------------------",
  "END",
  // 2
  "6|-C---C---C--e---D---C---c-",
  "5|F-eFe-DeD-CG-acF-G-e-F-D-e",
  "4|------------------G---G---",
  "END",
  // 3
  "6|C-------------------------",
  "5|C--a-G-g-G-CD-------------",
  "4|------------g---b---g-----",
  "4|------------D---G---D---G-",
  "4|------------A-------------",
  "4|--a-G-g-G-C-C---D---C---D-",
  "3|----------------b-------b-",
  "END",
  // 4
  "4|----------G-------A---b---",
  "4|--G---G---e---G---D---G---",
  "4|--e---D---C---D---C---D---",
  "3|--A---b---------------b---",
  "END",
  // 5
  "6|D-------------------------",
  "5|A---b---------------------",
  "5|D---G---D--------------c-C",
  "4|--------A---b-------------",
  "4|--------D---G-egDGgAFba-G-",
  "END",
  // 6
  "6|---------------------C---C",
  "5|cDCDcCDeCDeFDeFGeFGaG-FGF-",
  "END",
  // 7
  "6|---C---C--e---D---C-------",
  "5|eFe-DeD-CG-acF-G-D-e-Cbd-C",
  "4|----------------G---G---a-",
  "END",
  // 8
  "6|---CaFC-------------------",
  "5|aFa----aFC----------------",
  "4|--------------d-----------",
  "4|----------aFd-G---F---f---",
  "3|-------------ab-----------",
  "END",
  // 9
  "5|--------------------D---C-",
  "4|d---d-----------d---a-----",
  "4|F---G---F---f---F---F-----",
  "3|a---b-----------a---------",
  "END",
  // 10
  "6|----------------------C---",
  "5|--c---C-------c---C---C---",
  "4|------G---D-------G-------",
  "4|------e---a---G---e-------",
  "END",
  // 11
  "5|-------------C--CDCA-C-G--",
  "4|gGfgGAgGAbGAb-Ab----b-b-Ab",
  "END",
  // 12
  "5|-g---GC-------------------",
  "4|----------b---A-----------",
  "4|------A---G---g---G---g---",
  "4|A-GAG-e---D---C---D---e---",
  "3|------------------b---A---",
  "END",
  // 13
  "6|--------D-----------------",
  "5|--------D--------c--cC-cCD",
  "4|----A---------------------",
  "4|G---D---------------------",
  "4|D---C-------aAGaA-aA--A---",
  "3|b-------------------------",
  "END",
  // 14
  "6|-----c--------------------",
  "5|------------------D-------",
  "5|cCDfD-CDCAcCca-c-Ac---C---",
  "4|----------------------A---",
  "4|--------------A-A-F---f---",
  "END",
  // 15
  "5|c-------------------------",
  "4|----A---------------------",
  "4|a---f---G---F---f---e---c-",
  "4|D---C---f---D---C---c---fF",
  "3|--------b---A---A---A-----",
  "END",
  // 16
  "5|----------G---------------",
  "4|--C---C-----------D---D---",
  "4|efFGfFgGFgGAgGDAgGaAGaAbaA",
  "END",
  // 17
  "5|bc---c--cC-cCdcCdDCdDedDef",
  "4|----F---------------------",
  "4|b-Abb-Abg-b-G---a---A---b-",
  "END",
  // 18
  "5|--c---C-----------e-------",
  "5|DefFefFgFeDedDeFDeabGadeCD",
  "4|--------------b-------a---",
  "END",
  // 19
  "5|d-------c-----------------",
  "5|gaFgcd-cFgfF-c--ec--d---c-",
  "4|------------F-------------",
  "4|----g-b-----b-Abb-ababgaga",
  "END",
  // 20
  "4|--F---e---d---c-------G---",
  "4|FgbgeFaFdegecdFd-cec--d---",
  "3|----------------b-b-abG---",
  "3|----------------------b---",
  "END",
  // 21
  "6|--------------------G-----",
  "5|----------------b---b-----",
  "5|--------G-------G---G-----",
  "4|b-------b-----------------",
  "4|G-------G-------b---------",
  "3|b-------------------------",
  "END",
  // 22
  "6|F-f----C-G---f---d--------",
  "5|b-b-----------------------",
  "5|F-f---b-b-GAG-fFf-dedb-C-G",
  "4|----------------------b-b-",
  "END",
  // 23
  "5|--------------------A-----",
  "5|---f---dC---C-------C-----",
  "4|--------A---A-------------",
  "4|GAG-fFf-e---C-------A-----",
  "END",
  // 24
  "6|------A-----G-g-----------",
  "6|--C---C-----C-C---CDCA-c-g",
  "5|--A-----------------------",
  "5|--C---A-----G-g-------A-A-",
  "END",
  // 25
  "6|---e---C------------------",
  "5|gGg-eFe-CDCA-c-g---e-DeDC-",
  "4|------------A-A-gGg-D----b",
  "END",
  // 26
  "5|-----e---C-DeDC------F---D",
  "4|AGgGg-eFe-D----bAGaAa-FGF-",
  "END",
  // 27
  "5|-fFfDCc--C-G---fcCca---FCD",
  "4|f------Ab-b-GAG-----aAa---",
  "END",
  // 28
  "7|--------------c-----------",
  "6|----------cDFa-DeGFcCeD---",
  "5|CA-c-gcDFa-------------FGb",
  "4|--A-A---------------------",
  "END",
  // 29
  "5|acCeD---------------------",
  "4|------------G-------------",
  "4|------------F-------------",
  "4|-----FGbaFdCD-DefFfFgGFgGa",
  "3|------------G-------------",
  "END",
  // 30
  "4|DefFefFgfFgGFgGaDefFefFgfF",
  "END",
  // 31
  "5|-----------------c--cC-cCD",
  "4|gGFgGagGaAGaAbaAb-Ab--b---",
  "END",
  // 32
  "6|-----------------C---C---C",
  "5|cCDeCDeFDeFGeFGaG-FGF-eFe-",
  "END",
  // 33
  "6|---C--e---D---C---c-C-----",
  "5|DeD-CG-acF-G-e-F-D-eC--a-G",
  "4|------------G---G-----a-G-",
  "END",
  // 34
  "5|-g-G-CD-------------------",
  "4|------g---b---g-----------",
  "4|------D---G---D---G---G---",
  "4|------A-------------------",
  "4|g-G-C-C---D---C---D---e---",
  "3|----------b-------b---A---",
  "END",
  // 35
  "6|--------------------D-----",
  "5|--------------------A---b-",
  "5|--------------------D---G-",
  "4|----G-------A---b---------",
  "4|G---e---G---D---G---------",
  "4|D---C---D---C---D---------",
  "3|b---------------b---------",
  "END",
  // 36
  "5|--D--------------c-CcDCDcC",
  "4|--A---b-------------------",
  "4|--D---G-egDGgAFba-G-------",
  "END",
  // 37
  "6|---------------C---C---C--",
  "5|DeCDeFDeFGeFGaG-FGF-eFe-De",
  "END",
  // 38
  "6|-C--e---D---C----------CaF",
  "5|D-CG-acF-G-D-e-Cbd-CaFa---",
  "4|----------G---G---a-------",
  "END",
  // 39
  "6|C-------------------------",
  "5|-aFC----------------------",
  "4|--------d-----------d---d-",
  "4|----aFd-G---F---f---F---G-",
  "3|-------ab-----------a---b-",
  "END",
  // 40
  "5|--------------D---C---c---",
  "4|----------C---a-----------",
  "4|--F---f---F---F-----------",
  "END",
  // 41
  "5|C---D---C---c---C---C---C-",
  "4|----------------G---a-----",
  "4|a---a-----------C---F---b-",
  "4|e---F-----------b---C---f-",
  "END",
  // 42
  "5|--------------F---G---D---",
  "5|----------------------a---",
  "5|--C---D---e---c---C---C---",
  "4|--a---a---G---------------",
  "4|--F---F---e---G---G-------",
  "END",
  // 43
  "7|--------------------C-----",
  "6|------------e---e---e---D-",
  "6|--------C---C---C---C---C-",
  "6|----------------G---G---a-",
  "5|e-------e---G-------------",
  "5|G-------G-----------------",
  "5|C-------C---e---G-------a-",
  "END",
  // 44
  "6|-------------e---e---D---d",
  "5|--c---C---GaG-gGg-FGF-fFf-",
  "4|--G---G-------------------",
  "4|--F---e-------------------",
  "4|--c---C-------------------",
  "END",
  // 45
  "6|---C---C---c---C---e---e--",
  "5|eFe-DeD-DeD-CDC-GaG-gGg-FG",
  "END",
  // 46
  "6|-D---d---C---C---c---C---e",
  "5|F-fFf-eFe-DeD-DeD-CDC-GaG-",
  "END",
  // 47
  "6|---e---D---d---C---c------",
  "5|gGg-FgF-fFf-eFe-DeD-dDdbCd",
  "END",
  // 48
  "5|CAcCca-c-G---g---F---f---e",
  "4|------b-b-AbA-aAa-GaG-gGg-",
  "END",
  // 49
  "7|-------------------Ce-----",
  "6|---------------eCCG--GCCe-",
  "5|---DC------eCCG----------G",
  "4|FGF-e--eCCG---------------",
  "3|------G-------------------",
  "END",
  // 50
  "6|------------------C-------",
  "5|------------------e-------",
  "5|------------------G-------",
  "5|CCe---------------C-------",
  "4|---GCCe------------------C",
  "3|-------GCC---------------C",
  "END",
  // 51
  "4|C-------------------------",
  "3|C-------------------------",
  "END",
  NULL
};

struct Song {
  const int* notes;
  const int* durs;
  int length;
  int tempoBPM;
  const char* name;
  bool isMidi;
  bool isRawSlots;
};

// Global struct allows modifications inside setup() (e.g., dynamically updated lengths)
Song SONGS[] = {
  { song0_notes, song0_durs, sizeof(song0_notes)/sizeof(int), 110, "Twinkle Twinkle", false, false },
  { song1_notes, song1_durs, sizeof(song1_notes)/sizeof(int), 144, "Tetris", false, false },
  { song2_notes, song2_durs, sizeof(song2_notes)/sizeof(int), 200, "Super Mario Bros", false, false },
  { song3_notes, song3_durs, sizeof(song3_notes)/sizeof(int), 140, "Jingle Bells", false, false },
  { song4_notes, song4_durs, sizeof(song4_notes)/sizeof(int), 100, "Fur Elise", false, false },
  { torrent_notes, torrent_durs, 0, 160, "Torrent (Chopin)", true, true }
};
const int NUM_SONGS = sizeof(SONGS) / sizeof(Song);

// Routine parses the provided ASCII sheet notation for "Torrent" 
// Extracting only the highest possible RH note for the buzzer setup.
void parseTorrent() {
  int block_start = 0;
  int current_note = 0;
  int current_dur = 0;
  int torrent_length = 0;

  while (torrent_RH[block_start] != NULL) {
    int block_end = block_start;
    while (torrent_RH[block_end] != NULL && strcmp(torrent_RH[block_end], "END") != 0) {
      block_end++;
    }

    if (block_start == block_end) {
      if (torrent_RH[block_end] == NULL) break;
      block_start++;
      continue;
    }

    int max_len = 0;
    for (int l = block_start; l < block_end; l++) {
      int len = strlen(torrent_RH[l]);
      if (len > max_len) max_len = len;
    }

    for (int str_idx = 2; str_idx < max_len; str_idx++) {
      // Exclude bar lines entirely
      bool is_barline = false;
      for (int l = block_start; l < block_end; l++) {
        if (str_idx < (int)strlen(torrent_RH[l]) && torrent_RH[l][str_idx] == '|') {
          is_barline = true;
          break;
        }
      }
      if (is_barline) continue;

      int highest_midi = 0;
      for (int l = block_start; l < block_end; l++) {
        const char* line = torrent_RH[l];
        if (str_idx < (int)strlen(line)) {
          char nc = line[str_idx];
          int octave = line[0] - '0';

          if (nc != '-' && nc != '|' && nc != ' ') {
            int note_idx = -1;
            switch (nc) {
              case 'c': note_idx = 0; break;
              case 'C': note_idx = 1; break;
              case 'd': note_idx = 2; break;
              case 'D': note_idx = 3; break;
              case 'e': case 'E': note_idx = 4; break;
              case 'f': note_idx = 5; break;
              case 'F': note_idx = 6; break;
              case 'g': note_idx = 7; break;
              case 'G': note_idx = 8; break;
              case 'a': note_idx = 9; break;
              case 'A': note_idx = 10; break;
              case 'b': case 'B': note_idx = 11; break;
            }
            if (note_idx != -1) {
              int midi = (octave + 1) * 12 + note_idx;
              if (midi > highest_midi) {
                highest_midi = midi;
              }
            }
          }
        }
      }

      if (highest_midi > 0) {
        // Log & Commit previous extended note
        if (current_dur > 0 && torrent_length < MAX_TORRENT_NOTES) {
          torrent_notes[torrent_length] = current_note;
          torrent_durs[torrent_length]  = current_dur;
          torrent_length++;
        }
        current_note = highest_midi; // Save the newly found maximum pitch note
        current_dur = 1;             // Start base duration metric (1 slot)
      } else {
        // Maintain active note / extended silence
        if (current_dur > 0) current_dur++;
        else { current_note = 0; current_dur = 1; }
      }
    }

    if (torrent_RH[block_end] == NULL) break;
    block_start = block_end + 1;
  }

  // Ensure very last note commits seamlessly
  if (current_dur > 0 && torrent_length < MAX_TORRENT_NOTES) {
    torrent_notes[torrent_length] = current_note;
    torrent_durs[torrent_length]  = current_dur;
    torrent_length++;
  }

  // Bind new populated size back to the array config
  SONGS[5].length = torrent_length;
}

// ==================== MOTION DETECTION ====================
// Deviation from 1 g (gravity) in any direction, above which we call it "moving".
const float         MOTION_THRESHOLD  = 0.03f;
const unsigned long MOTION_TIMEOUT_MS = 500;

// ==================== STATE ====================
int           currentSong      = 0;
bool          armed            = false;
int           noteIndex        = 0;
unsigned long noteStartTime    = 0;
unsigned long currentNoteMs    = 0;
bool          noteActive       = false;
unsigned long lastMotionTime   = 0;

// Button debounce
int           lastSongBtn       = HIGH;
int           lastArmBtn        = HIGH;
unsigned long lastSongBtnChange = 0;
unsigned long lastArmBtnChange  = 0;
const unsigned long DEBOUNCE_MS = 40;

// LED blinking while playing
unsigned long lastBlink = 0;
bool          blinkOn   = false;

// ==================== SETUP ====================
void setup() {
  pinMode(BUZZER_PIN,      OUTPUT);
  pinMode(BUTTON_SONG_PIN, INPUT_PULLUP);
  pinMode(BUTTON_ARM_PIN,  INPUT_PULLUP);
  pinMode(STATUS_LED_PIN,  OUTPUT);

  // On-board RGB LED is ACTIVE LOW: HIGH = off, LOW = on
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Compile Torrent strings into valid note sequences 
  parseTorrent();

  Serial.begin(9600);
  delay(200);

  if (!IMU.begin()) {
    Serial.println("IMU init failed. Halting.");
    while (true) {
      digitalWrite(LEDR, LOW);  delay(200);
      digitalWrite(LEDR, HIGH); delay(200);
    }
  }

  Serial.println("Train Music Player ready.");
  Serial.print("Song: ");
  Serial.println(SONGS[currentSong].name);
  updateSongLED();
}

// ==================== HELPERS ====================
void updateSongLED() {
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  switch (currentSong) {
    case 0: digitalWrite(LEDR, LOW); break;                              // red
    case 1: digitalWrite(LEDG, LOW); break;                              // green
    case 2: digitalWrite(LEDB, LOW); break;                              // blue
    case 3: digitalWrite(LEDR, LOW); digitalWrite(LEDG, LOW); break;     // yellow
    case 4: digitalWrite(LEDR, LOW); digitalWrite(LEDB, LOW); break;     // magenta
    case 5: digitalWrite(LEDG, LOW); digitalWrite(LEDB, LOW); break;     // cyan (G+B)
  }
}

bool detectMotion() {
  float x, y, z;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    float mag = sqrt(x*x + y*y + z*z);
    float dev = fabs(mag - 1.0f);
    if (dev > MOTION_THRESHOLD) {
      lastMotionTime = millis();
      return true;
    }
  }
  return (millis() - lastMotionTime) < MOTION_TIMEOUT_MS;
}

void handleButtons() {
  int sb = digitalRead(BUTTON_SONG_PIN);
  if (sb != lastSongBtn && (millis() - lastSongBtnChange) > DEBOUNCE_MS) {
    lastSongBtnChange = millis();
    if (sb == LOW) {
      currentSong = (currentSong + 1) % NUM_SONGS;
      noteIndex   = 0;
      noteActive  = false;
      noTone(BUZZER_PIN);
      Serial.print("Song: ");
      Serial.println(SONGS[currentSong].name);
      updateSongLED();
    }
    lastSongBtn = sb;
  }

  int ab = digitalRead(BUTTON_ARM_PIN);
  if (ab != lastArmBtn && (millis() - lastArmBtnChange) > DEBOUNCE_MS) {
    lastArmBtnChange = millis();
    if (ab == LOW) {
      armed = !armed;
      Serial.print("Armed: ");
      Serial.println(armed ? "YES" : "no");
      if (!armed) {
        noTone(BUZZER_PIN);
        noteActive = false;
        noteIndex  = 0; 
      }
    }
    lastArmBtn = ab;
  }
}

void playSongTick() {
  const Song& song = SONGS[currentSong];
  unsigned long now = millis();

  if (!noteActive) {
    int freq = song.notes[noteIndex];
    int dur  = song.durs[noteIndex];
    unsigned long slotMs = 0;

    if (song.isRawSlots) {
      // Formula: Single 16th note timeframe scaled to BPM multiplied by raw slots occupied
      slotMs = (15000UL / song.tempoBPM) * dur; 
    } else {
      bool dotted = false;
      if (dur < 0) {
        dur = -dur;
        dotted = true;
      }
      slotMs = (60000UL / song.tempoBPM) * 4 / dur;
      if (dotted) slotMs = slotMs * 3 / 2;
    }

    if (freq == NOTE_REST || freq == 0) {
      noTone(BUZZER_PIN);
    } else {
      int playHz = freq;
      
      if (song.isMidi) {
        // Convert the computed standard MIDI note back into an absolute Frequency
        playHz = round(440.0 * pow(2.0, (freq - 69.0) / 12.0));
      }
      
      tone(BUZZER_PIN, playHz, slotMs * 9 / 10);
    }
    
    noteStartTime = now;
    currentNoteMs = slotMs;
    noteActive    = true;
  } else if (now - noteStartTime >= currentNoteMs) {
    noteActive = false;
    noteIndex  = (noteIndex + 1) % song.length;
  }
}

// ==================== LOOP ====================
void loop() {
  handleButtons();

  bool moving = detectMotion();

  // Status LEDs
  if (armed) {
    if (moving) {
      if (millis() - lastBlink > 100) {
        lastBlink = millis();
        blinkOn = !blinkOn;
        digitalWrite(STATUS_LED_PIN, blinkOn ? HIGH : LOW);
        digitalWrite(LED_BUILTIN,    blinkOn ? HIGH : LOW);
      }
    } else {
      digitalWrite(STATUS_LED_PIN, HIGH);
      digitalWrite(LED_BUILTIN,    LOW);
    }
  } else {
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(LED_BUILTIN,    LOW);
  }

  // Playback gate
  if (armed && moving) {
    playSongTick();
  } else if (noteActive) {
    noTone(BUZZER_PIN);
    noteActive = false;
  }
}