/* Copyright 2015-2017 Jack Humbert
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "muse.h"


enum planck_layers {
  _QWERTY,
  _LOWER,
  _RAISE,
  _SPACEFN,
  _NAV,
  _TERM,
  _ADJUST
};

enum planck_keycodes {
  QWERTY = SAFE_RANGE,
  BACKLIT
};

#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)
#define SPACEFN LT(_SPACEFN, KC_SPACE)
#define NAV MO(_NAV)
#define TERM MO(_TERM)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

/* Qwerty
 * ,-----------------------------------------------------------------------------------.
 * | Tab  |   Q  |   W  |   E  |   R  |   T  |   Y  |   U  |   I  |   O  |   P  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | C/Esc|   A  |   S  |   D  |   F  |   G  |   H  |   J  |   K  |   L  |   ;  |  "   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Lsft |   Z  |   X  |   C  |   V  |   B  |   N  |   M  |   ,  |   .  |   /  |Lsft/E|
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Ctrl |  Alt | GUI  |  Nav |Lower |   SpaceFn   |Raise | Term | Hypr |  GUI | RAlt |
 * `-----------------------------------------------------------------------------------'
 */
[_QWERTY] = LAYOUT_planck_mit(
  KC_TAB,         KC_Q,    KC_W,    KC_E,  KC_R,  KC_T,  KC_Y,  KC_U,  KC_I,    KC_O,    KC_P,    KC_BSPC,
  LCTL_T(KC_ESC), KC_A,    KC_S,    KC_D,  KC_F,  KC_G,  KC_H,  KC_J,  KC_K,    KC_L,    KC_SCLN, KC_QUOT,
  KC_LSFT,        KC_Z,    KC_X,    KC_C,  KC_V,  KC_B,  KC_N,  KC_M,  KC_COMM, KC_DOT,  KC_SLSH, RSFT_T(KC_ENTER),
  KC_LCTL,        KC_LALT, KC_LGUI, NAV,   LOWER,    SPACEFN,   RAISE, TERM,    KC_HYPR, KC_LGUI, KC_RALT
),

/* Lower
 * ,-----------------------------------------------------------------------------------.
 * |   ~  |   !  |   @  |   #  |   $  |   %  |   ^  |   &  |   *  |   (  |   )  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   (  |   {  |   [  |   +  |   *  |   ~  |   =  |   ]  |   }  |   )  |  |   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   \  |   <  | xxxx |   -  |   /  |   `  |   _  | xxxx |   >  |   /  |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      | xxxx |      |             |      | xxxx |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_LOWER] = LAYOUT_planck_mit(
  KC_TILD, KC_EXLM, KC_AT,   KC_HASH, KC_DLR,  KC_PERC, KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_BSPC,
  _______, KC_LPRN, KC_LCBR, KC_LBRC, KC_PPLS, KC_PAST, KC_TILD, KC_PEQL, KC_RBRC, KC_RCBR, KC_RPRN, KC_PIPE,
  _______, KC_BSLS, KC_LT,   XXXXXXX, KC_PMNS, KC_PSLS, KC_GRV,  KC_UNDS, XXXXXXX, KC_GT,   KC_SLSH, _______,
  _______, _______, _______, XXXXXXX, _______,      KC_SPC,      _______, XXXXXXX, _______, _______, _______
),

/* Raise
 * ,-----------------------------------------------------------------------------------.
 * |   `  |   1  |   2  |   3  |   4  |   5  |   6  |   7  |   8  |   9  |   0  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   (  |   {  |   [  |   +  |   *  |   ~  |   =  |   ]  |   }  |   )  |  |   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   \  |   <  | xxxx |   -  |   /  |   `  |   _  | xxxx |   >  |   /  |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      | xxxx |      |             |      | xxxx |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_RAISE] = LAYOUT_planck_mit(
  KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_BSPC,
  _______, KC_LPRN, KC_LCBR, KC_LBRC, KC_PPLS, KC_PAST, KC_TILD, KC_PEQL, KC_RBRC, KC_RCBR, KC_RPRN, KC_PIPE,
  _______, KC_BSLS, KC_LT,   XXXXXXX, KC_PMNS, KC_PSLS, KC_GRV,  KC_UNDS, XXXXXXX, KC_GT,   KC_SLSH, _______,
  _______, _______, _______, XXXXXXX, _______,       KC_SPC,     _______, XXXXXXX, _______, _______, _______
),

/* Adjust (Lower + Raise)
 *                                           v---------------RGB CONTROL---------------v
 * ,-----------------------------------------------------------------------------------.
 * | RGB  |RGBMOD| xxxx | F14  | F15  | xxxx | HUE- | HUE+ | SAT- | SAT+ |BRGTH-|BRGTH+|
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | xxxx | xxxx | Vol- | Mute | Vol+ |Musoff|Mus on|MIDIof|MIDIon|MUSmod|Audoff|Aud on|
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | xxxx | xxxx | Prev | Play | Next |Voice-|Voice+| xxxx | xxxx | xxxx | xxxx | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | xxxx | xxxx | xxxx | xxxx |      |     xxxx    |      | xxxx | xxxx |Debug |Reset |
 * `-----------------------------------------------------------------------------------'
 */
[_ADJUST] = LAYOUT_planck_mit(
  RGB_TOG, RGB_MOD, XXXXXXX, KC_F14,  KC_F15,  XXXXXXX, RGB_HUD, RGB_HUI, RGB_SAD, RGB_SAI, RGB_VAD, RGB_VAI,
  XXXXXXX, XXXXXXX, KC_VOLD, KC_MUTE, KC_VOLU, MU_OFF,  MU_ON,   MI_OFF,  MI_ON,   MU_MOD,  AU_OFF,  AU_ON,
  XXXXXXX, XXXXXXX, KC_MRWD, KC_MPLY, KC_MFFD, MUV_DE,  MUV_IN,  XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
  XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, _______,     XXXXXXX,      _______, XXXXXXX, XXXXXXX, DEBUG,   RESET
),

/* SpaceFn
 * ,-----------------------------------------------------------------------------------.
 * |C(Tab)|  F1  |  F2  |  F3  |  F4  | F13  |A(<-) | PgDn | PgUp |A(->) | xxxx | Del  |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |  F5  |  F6  |  F7  |  F8  |A(Tab)| Left | Down |  Up  |Right |G(Tab)| xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |S/Caps|  F9  |  F10 |  F11 |  F12 |A(Grv)| Home | xxxx | xxxx | End  |G(Grv)|      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      | xxxx | xxxx |             | xxxx | xxxx |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_SPACEFN] = LAYOUT_planck_mit(
  LCTL(KC_TAB),    KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F13,       LALT(KC_LEFT), KC_PGDN, KC_PGUP, LALT(KC_RGHT), XXXXXXX,      KC_DEL,
  _______,         KC_F5,   KC_F6,   KC_F7,   KC_F8,   LALT(KC_TAB), KC_LEFT,       KC_DOWN, KC_UP,   KC_RGHT,       LGUI(KC_TAB), XXXXXXX,
  LSFT_T(KC_CAPS), KC_F9,   KC_F10,  KC_F11,  KC_F12,  LALT(KC_GRV), KC_HOME,       XXXXXXX, XXXXXXX, KC_END,        LGUI(KC_GRV), _______,
  _______,         _______, _______, XXXXXXX, XXXXXXX,          _______,            XXXXXXX, XXXXXXX, _______,       _______,      _______
),

/* Nav
 * ,-----------------------------------------------------------------------------------.
 * | xxxx | xxxx | xxxx | xxxx | M(R) | xxxx | xxxx | xxxx | xxxx | xxxx | M(P) | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      | M(A) | M(S) | M(D) | M(F) | M(G) | A(H) | A(J) | A(K) | A(L) | M(;) | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      | xxxx | xxxx | xxxx | xxxx | xxxx | M(N) | xxxx | xxxx | xxxx | M(/) | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      | xxxx |     Meh     | xxxx | xxxx |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_NAV] = LAYOUT_planck_mit(
  XXXXXXX, XXXXXXX,   XXXXXXX,   XXXXXXX,   MEH(KC_R), XXXXXXX,   XXXXXXX,    XXXXXXX,    XXXXXXX,    XXXXXXX,    MEH(KC_P),    XXXXXXX,
  _______, MEH(KC_A), MEH(KC_S), MEH(KC_D), MEH(KC_F), MEH(KC_G), LALT(KC_H), LALT(KC_J), LALT(KC_K), LALT(KC_L), MEH(KC_SCLN), XXXXXXX,
  _______, XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,   MEH(KC_N),  XXXXXXX,    XXXXXXX,    XXXXXXX,    MEH(KC_SLSH), XXXXXXX,
  _______, _______,   _______,   _______,   XXXXXXX,          KC_MEH,         XXXXXXX,    XXXXXXX,    _______,    _______,      _______
),

/* Term
 * ,-----------------------------------------------------------------------------------.
 * | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      | xxxx | xxxx | xxxx | xxxx | xxxx |A(Lft)|A(Dwn)|A(Up) |A(Rgt)| xxxx | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      | xxxx | xxxx |     xxxx    | xxxx |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_TERM] = LAYOUT_planck_mit(
  XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,       XXXXXXX,       XXXXXXX,     XXXXXXX,       XXXXXXX, XXXXXXX,
  _______, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, LALT(KC_LEFT), LALT(KC_DOWN), LALT(KC_UP), LALT(KC_RGHT), XXXXXXX, XXXXXXX,
  _______, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,       XXXXXXX,       XXXXXXX,     XXXXXXX,       XXXXXXX, XXXXXXX,
  _______, _______, _______, XXXXXXX, XXXXXXX,      XXXXXXX,           XXXXXXX,       _______,     _______,       _______, _______
)

};

#ifdef AUDIO_ENABLE
  float plover_song[][2]     = SONG(PLOVER_SOUND);
  float plover_gb_song[][2]  = SONG(PLOVER_GOODBYE_SOUND);
#endif

layer_state_t layer_state_set_user(layer_state_t state) {
  return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_DOWN);
      #else
        tap_code(KC_PGDN);
      #endif
    } else {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_UP);
      #else
        tap_code(KC_PGUP);
      #endif
    }
  }
}

void dip_switch_update_user(uint8_t index, bool active) {
    switch (index) {
        case 0: {
#ifdef AUDIO_ENABLE
            static bool play_sound = false;
#endif
            if (active) {
#ifdef AUDIO_ENABLE
                if (play_sound) { PLAY_SONG(plover_song); }
#endif
                layer_on(_ADJUST);
            } else {
#ifdef AUDIO_ENABLE
                if (play_sound) { PLAY_SONG(plover_gb_song); }
#endif
                layer_off(_ADJUST);
            }
#ifdef AUDIO_ENABLE
            play_sound = true;
#endif
            break;
        }
        case 1:
            if (active) {
                muse_mode = true;
            } else {
                muse_mode = false;
            }
    }
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    } else {
        if (muse_counter) {
            stop_all_notes();
            muse_counter = 0;
        }
    }
#endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}
