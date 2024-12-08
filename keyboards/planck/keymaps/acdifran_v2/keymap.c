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
// #include "features/achordion.h"

// Left-hand home row mods
#define HOME_A LSFT_T(KC_A)
#define HOME_S LALT_T(KC_S)
#define HOME_D RCTL_T(KC_D)
#define HOME_F LGUI_T(KC_F)

// Right-hand home row mods
#define HOME_J RGUI_T(KC_J)
#define HOME_K RCTL_T(KC_K)
#define HOME_L LALT_T(KC_L)
#define HOME_SCLN RSFT_T(KC_SCLN)

#define CTRL_ESC LCTL_T(KC_ESC)

// symbol level mods
// Keys that are shifted, like parens, need to be handled differently
// handling them all this way because doing it the standard way causes a delay
// and I need to hold the layer key longer to send the symbol
// enumerate custom macro id above keymap
enum custom_macros {
  HOME_A_SYM = SAFE_RANGE,
  HOME_S_SYM,
  HOME_D_SYM,
  HOME_F_SYM,
  HOME_J_SYM,
  HOME_K_SYM,
  HOME_L_SYM,
  HOME_SCLN_SYM
};

// declare key_timer for use in macro
uint16_t key_timer;
// custom macro processor
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  // if (!process_achordion(keycode, record)) { return false; }
  switch(keycode) {
    case HOME_A_SYM:
      if (record->event.pressed) {
        key_timer = timer_read();
        register_mods(MOD_BIT(KC_LSFT));
      } else {
        unregister_mods(MOD_BIT(KC_LSFT));
        if (timer_elapsed(key_timer) < TAPPING_TERM) {
          tap_code16(S(KC_LPRN));
        }
      }
      return false;
    case HOME_S_SYM:
      if (record->event.pressed) {
        key_timer = timer_read();
        register_mods(MOD_BIT(KC_LALT));
      } else {
        unregister_mods(MOD_BIT(KC_LALT));
        if (timer_elapsed(key_timer) < TAPPING_TERM) {
          tap_code16(S(KC_LCBR));
        }
      }
      return false;
    case HOME_D_SYM:
      if (record->event.pressed) {
        key_timer = timer_read();
        register_mods(MOD_BIT(KC_LCTL));
      } else {
        unregister_mods(MOD_BIT(KC_LCTL));
        if (timer_elapsed(key_timer) < TAPPING_TERM) {
          tap_code(KC_LBRC);
        }
      }
      return false;
    case HOME_F_SYM:
      if (record->event.pressed) {
        key_timer = timer_read();
        register_mods(MOD_BIT(KC_LGUI));
      } else {
        unregister_mods(MOD_BIT(KC_LGUI));
        if (timer_elapsed(key_timer) < TAPPING_TERM) {
          tap_code(KC_PPLS);
        }
      }
      return false;
    case HOME_J_SYM:
      if (record->event.pressed) {
        key_timer = timer_read();
        register_mods(MOD_BIT(KC_LGUI));
      } else {
        unregister_mods(MOD_BIT(KC_LGUI));
        if (timer_elapsed(key_timer) < TAPPING_TERM) {
          tap_code(KC_PEQL);
        }
      }
      return false;
    case HOME_K_SYM:
      if (record->event.pressed) {
        key_timer = timer_read();
        register_mods(MOD_BIT(KC_LCTL));
      } else {
        unregister_mods(MOD_BIT(KC_LCTL));
        if (timer_elapsed(key_timer) < TAPPING_TERM) {
          tap_code(KC_RBRC);
        }
      }
      return false;
    case HOME_L_SYM:
      if (record->event.pressed) {
        key_timer = timer_read();
        register_mods(MOD_BIT(KC_LALT));
      } else {
        unregister_mods(MOD_BIT(KC_LALT));
        if (timer_elapsed(key_timer) < TAPPING_TERM) {
          tap_code16(S(KC_RCBR));
        }
      }
      return false;
    case HOME_SCLN_SYM:
      if (record->event.pressed) {
        key_timer = timer_read();
        register_mods(MOD_BIT(KC_RSFT));
      } else {
        unregister_mods(MOD_BIT(KC_RSFT));
        if (timer_elapsed(key_timer) < TAPPING_TERM) {
          tap_code16(S(KC_RPRN));
        }
      }
      return false;
  }
  return true;
}


enum planck_layers {
  _QWERTY,
  _GAME,
  _LOWER,
  _RAISE,
  _SPACEFN,
  _NAV,
  _TERM,
  _ADJUST,
  _FN
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
#define FN MO(_FN)
#define GAME TO(_GAME)
#define BASE TO(_QWERTY)
#define KC_LCA LCA(KC_NO)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

/* Qwerty
 * ,-----------------------------------------------------------------------------------.
 * | Tab  |   Q  |   W  |   E  |   R  |   T  |   Y  |   U  |   I  |   O  |   P  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |C/Esc |   A  |   S  |   D  |   F  |   G  |   H  |   J  |   K  |   L  |   ;  |  "   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Lsft |   Z  |   X  |   C  |   V  |   B  |   N  |   M  |   ,  |   .  |   /  |Rsft/E|
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | xxxx | Alt  | Lca  |  Nav |Lower |   SpaceFn   |Raise |  Fn  | Gui  | Ctrl | xxxx |
 * `-----------------------------------------------------------------------------------'
 */
[_QWERTY] = LAYOUT_planck_mit(
  KC_TAB,   KC_Q,    KC_W,    KC_E,   KC_R,    KC_T,    KC_Y,  KC_U,    KC_I,    KC_O,    KC_P,      KC_BSPC,
  CTRL_ESC, HOME_A,  HOME_S,  HOME_D, HOME_F,  KC_G,    KC_H,  HOME_J,  HOME_K,  HOME_L,  HOME_SCLN, KC_QUOT,
  KC_LSFT,  KC_Z,    KC_X,    KC_C,   KC_V,    KC_B,    KC_N,  KC_M,    KC_COMM, KC_DOT,  KC_SLSH,   RSFT_T(KC_ENTER),
  XXXXXXX,  KC_LALT, KC_LCA,  NAV,    LOWER,   SPACEFN, RAISE, FN,      KC_LGUI, KC_LCTL, XXXXXXX
),

/* Game
 * ,-----------------------------------------------------------------------------------.
 * | Tab  |   Q  |   W  |   E  |   R  |   T  |   Y  |   U  |   I  |   O  |   P  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Ctrl |   A  |   S  |   D  |   F  |   G  |   H  |   J  |   K  |   L  |   ;  |  "   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Lsft |   Z  |   X  |   C  |   V  |   B  |   N  |   M  |   ,  |   .  |   /  |Rsft/E|
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | xxxx | Alt  | Lca  |  Nav |Lower |   SpaceFn   |Raise |  Fn  | Gui  | Ctrl | xxxx |
 * `-----------------------------------------------------------------------------------'
 */
[_GAME] = LAYOUT_planck_mit(
  KC_TAB,   KC_Q,    KC_W,    KC_E,   KC_R,    KC_T,    KC_Y,  KC_U,    KC_I,    KC_O,    KC_P,      KC_BSPC,
  KC_LCTL,  KC_A,    KC_S,    KC_D,   KC_F,    KC_G,    KC_H,  HOME_J,  HOME_K,  HOME_L,  HOME_SCLN, KC_QUOT,
  KC_LSFT,  KC_Z,    KC_X,    KC_C,   KC_V,    KC_B,    KC_N,  KC_M,    KC_COMM, KC_DOT,  KC_SLSH,   RSFT_T(KC_ENTER),
  XXXXXXX,  KC_LALT, KC_LCA,  NAV,    LOWER,   SPACEFN, RAISE, FN,      KC_LGUI, KC_LCTL, XXXXXXX
),

/* Lower
 * ,-----------------------------------------------------------------------------------.
 * |   ~  |   !  |   @  |   #  |   $  |   %  |   ^  |   &  |   *  |   (  |   )  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   (  |   {  |   [  |   +  |   *  |   ~  |   =  |   ]  |   }  |   )  |  |   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   \  |   <  |   ^  |   -  |   /  |   `  |   _  |   $  |   >  |   /  |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_LOWER] = LAYOUT_planck_mit(
  KC_TILD, KC_EXLM,    KC_AT,      KC_HASH,    KC_DLR,     KC_PERC, KC_CIRC, KC_AMPR,    KC_ASTR,    KC_LPRN,    KC_RPRN,       KC_BSPC,
  KC_ESC , HOME_A_SYM, HOME_S_SYM, HOME_D_SYM, HOME_F_SYM, KC_PAST, KC_TILD, HOME_J_SYM, HOME_K_SYM, HOME_L_SYM, HOME_SCLN_SYM, KC_PIPE,
  _______, KC_BSLS,    KC_LT,      KC_CIRC,    KC_PMNS,    KC_PSLS, KC_GRV,  KC_UNDS,    KC_DLR,     KC_GT,      KC_SLSH,       _______,
  _______, _______,    _______,    _______,    _______,         _______,     _______,    _______,    _______,    _______,       _______
),

/* Raise
 * ,-----------------------------------------------------------------------------------.
 * |   `  |   1  |   2  |   3  |   4  |   5  |   6  |   7  |   8  |   9  |   0  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   (  |   {  |   [  |   +  |   *  |   ~  |   =  |   ]  |   }  |   )  |  |   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   \  |   <  |   ^  |   -  |   /  |   `  |   _  |   $  |   >  |   /  |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_RAISE] = LAYOUT_planck_mit(
  KC_GRV,  KC_1,       KC_2,       KC_3,       KC_4,       KC_5,    KC_6,    KC_7,       KC_8,       KC_9,       KC_0,          KC_BSPC,
  _______, HOME_A_SYM, HOME_S_SYM, HOME_D_SYM, HOME_F_SYM, KC_PAST, KC_TILD, HOME_J_SYM, HOME_K_SYM, HOME_L_SYM, HOME_SCLN_SYM, KC_PIPE,
  _______, KC_BSLS,    KC_LT,      KC_CIRC,    KC_PMNS,    KC_PSLS, KC_GRV,  KC_UNDS,    KC_DLR,     KC_GT,      KC_SLSH,       _______,
  _______, _______,    _______,    _______,    _______,         _______,     _______,    _______,    _______,    _______,       _______
),

/* Adjust (Lower + Raise)
 *                                           v---------------RGB CONTROL---------------v
 * ,-----------------------------------------------------------------------------------.
 * | RGB  |RGBMOD| xxxx | F14  | F15  | xxxx | HUE- | HUE+ | SAT- | SAT+ |BRGHT-|BRGHT+|
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | xxxx | xxxx | Vol- | Mute | Vol+ |Musoff|Mus on|MIDIof|MIDIon|MUSmod|Audoff|Aud on|
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | xxxx | xxxx | Prev | Play | Next |Voice-|Voice+| xxxx | xxxx | xxxx | xxxx | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Game | Base | xxxx | xxxx |      |     xxxx    |      | xxxx | xxxx |Debug |Reset |
 * `-----------------------------------------------------------------------------------'
 */
[_ADJUST] = LAYOUT_planck_mit(
  RGB_TOG, RGB_MOD, XXXXXXX, KC_F14,  KC_F15,  XXXXXXX, RGB_HUD, RGB_HUI, RGB_SAD, RGB_SAI, RGB_VAD, RGB_VAI,
  XXXXXXX, XXXXXXX, KC_VOLD, KC_MUTE, KC_VOLU, MU_OFF,  MU_ON,   MI_OFF,  MI_ON,   MU_NEXT,  AU_OFF,  AU_ON,
  XXXXXXX, XXXXXXX, KC_MRWD, KC_MPLY, KC_MFFD, AU_PREV, AU_NEXT, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
  GAME,    BASE,    XXXXXXX, XXXXXXX, _______,     XXXXXXX,      _______, XXXXXXX, XXXXXXX, DB_TOGG, QK_BOOT
),

/* SpaceFn
 * ,-----------------------------------------------------------------------------------.
 * |C(Tab)|CAG(1)|CAG(2)|CAG(3)| xxxx | xxxx |A(<-) | PgDn | PgUp |A(->) | xxxx | Del  |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |CAG(4)|CAG(5)|CAG(6)|CAG(0)|A(Tab)| Left | Down |  Up  |Right |G(Tab)| xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |S/Caps|CAG(7)|CAG(8)|CAG(9)| xxxx |A(Grv)| Home | xxxx | xxxx | End  |G(Grv)|      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_SPACEFN] = LAYOUT_planck_mit(
  LCTL(KC_TAB),    LCAG(KC_1), LCAG(KC_2), LCAG(KC_3), XXXXXXX,     XXXXXXX,      LALT(KC_LEFT), KC_PGDN, KC_PGUP, LALT(KC_RGHT), XXXXXXX,      KC_DEL,
  _______,         LCAG(KC_4), LCAG(KC_5), LCAG(KC_6), LCAG(KC_0),  LALT(KC_TAB), KC_LEFT,       KC_DOWN, KC_UP,   KC_RGHT,       LGUI(KC_TAB), XXXXXXX,
  LSFT_T(KC_CAPS), LCAG(KC_7), LCAG(KC_8), LCAG(KC_9), XXXXXXX,     LALT(KC_GRV), KC_HOME,       XXXXXXX, XXXXXXX, KC_END,        LGUI(KC_GRV), _______,
  _______,         _______,    _______,    _______,    _______,          _______,                _______, _______, _______,       _______,      _______
),

/* Nav
 * ,-----------------------------------------------------------------------------------.
 * | xxxx | M(Q) | M(W) | M(E) | M(R) | xxxx | xxxx | M(U) | M(I) | M(0) | M(P) | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      | M(A) | M(S) | M(D) | M(F) | xxxx | A(H) | A(J) | A(K) | A(L) | M(;) | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      | M(Z) | xxxx | xxxx | xxxx | xxxx | M(N) | M(M) | M(,) | M(.) | M(/) | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |     Meh     |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_NAV] = LAYOUT_planck_mit(
  XXXXXXX, MEH(KC_Q), MEH(KC_W), MEH(KC_E), MEH(KC_R), XXXXXXX, XXXXXXX,    MEH(KC_U),  MEH(KC_I),     MEH(KC_O),   MEH(KC_P),    XXXXXXX,
  _______, MEH(KC_A), MEH(KC_S), MEH(KC_D), MEH(KC_F), XXXXXXX, LALT(KC_H), LALT(KC_J), LALT(KC_K),    LALT(KC_L),  MEH(KC_SCLN), XXXXXXX,
  _______, MEH(KC_Z), XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX, MEH(KC_N),  MEH(KC_M),  MEH(KC_COMMA), MEH(KC_DOT), MEH(KC_SLSH), XXXXXXX,
  _______, _______,   _______,   _______,   _______,          KC_MEH,       _______,    _______,       _______,     _______,      _______
),

/* Term
 * ,-----------------------------------------------------------------------------------.
 * | xxxx | xxxx | xxxx |CAG(E)|CAG(R)| xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      | xxxx | xxxx |CAG(D)|CAG(F)| xxxx |A(Lft)|A(Dwn)|A(Up) |A(Rgt)| xxxx | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      | xxxx |CAG(X)| xxxx |CAG(V)| xxxx | xxxx |CAG(M)| xxxx | xxxx |CAG(/)| xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_TERM] = LAYOUT_planck_mit(
  XXXXXXX, XXXXXXX, XXXXXXX,    LCAG(KC_E), LCAG(KC_R), XXXXXXX, XXXXXXX,       XXXXXXX,       XXXXXXX,     XXXXXXX,       XXXXXXX,       XXXXXXX,
  _______, XXXXXXX, XXXXXXX,    LCAG(KC_D), LCAG(KC_F), XXXXXXX, LALT(KC_LEFT), LALT(KC_DOWN), LALT(KC_UP), LALT(KC_RGHT), XXXXXXX,       XXXXXXX,
  _______, XXXXXXX, LCAG(KC_X), XXXXXXX,    LCAG(KC_V), XXXXXXX, XXXXXXX,       LCAG(KC_M),    XXXXXXX,     XXXXXXX,       LCAG(KC_SLSH), XXXXXXX,
  _______, _______, _______,    _______,    _______,         _______,           _______,       _______,     _______,       _______,       _______
),

/* FN
 * ,-----------------------------------------------------------------------------------.
 * | xxxx |  F1  |  F2  |  F3  |  F4  |  F5  |  F6  |  F7  |  F8  |  F9  |  F10 | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |  F11 |  F12 |  F13 | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx | xxxx |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_FN] = LAYOUT_planck_mit(
  XXXXXXX, KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  XXXXXXX,
  _______, KC_F11,  KC_F12,  KC_F13,  XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
  _______, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
  _______, _______, _______, _______, _______,      _______,     _______, _______, _______, _______, _______
)

};

#ifdef AUDIO_ENABLE
  float plover_song[][2]     = SONG(PLOVER_SOUND);
  float plover_gb_song[][2]  = SONG(PLOVER_GOODBYE_SOUND);
#endif

layer_state_t layer_state_set_user(layer_state_t state) {
  state = update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
  state = update_tri_layer_state(state, _NAV, _RAISE, _TERM);
  return state;
}

uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case RSFT_T(KC_ENTER):
            return TAPPING_TERM - 60;
        default:
            return TAPPING_TERM;
    }
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

bool dip_switch_update_user(uint8_t index, bool active) {
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
  return true;
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
// achordion_task();
}

// bool achordion_eager_mod(uint8_t mod) {
//   switch (mod) {
//     default:
//       return false;
//   }
// }

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}
