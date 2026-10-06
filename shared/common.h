/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2017 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2017 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
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

#pragma once
#include <stdint.h>
#include "stmbl_talk.h"

#if __GNUC__ < 5
#error gcc to old (< 5.0)
#endif

#define DATABAUD 3000000  //baudrate used for communication

#pragma pack(push, 1)
typedef struct {
  stmbl_talk_header_t header;
  uint32_t addr;
  uint32_t value;
  enum {
    BOOTLOADER_OPCODE_NOP = 0,
    BOOTLOADER_OPCODE_READ,
    BOOTLOADER_OPCODE_WRITE,      // MEM[addr] = value
    BOOTLOADER_OPCODE_PAGEERASE,  // clear PAGE[addr]
    BOOTLOADER_OPCODE_CRCCHECK,   // CRC(APP) == 0
  } cmd : 8;
  enum {
    BOOTLOADER_STATE_OK = 0,
    BOOTLOADER_STATE_NAK,
  } state : 8;
  uint16_t padding;
} packet_bootloader_t;
_Static_assert(sizeof(packet_bootloader_t) == 24, "packet_bootloader_t size error");

//process data from f3 to f4
typedef struct {
  stmbl_talk_header_t header;
  float id_fb;
  float iq_fb;
  float ud_fb;
  float uq_fb;
  uint8_t fault;
  uint8_t buf;
  uint16_t padding;
} packet_from_hv_t;
_Static_assert(sizeof(packet_from_hv_t) == 32, "packet_from_hv_t size error");

//process data from f4 to f3
typedef struct {
  stmbl_talk_header_t header;
  float d_cmd;
  float q_cmd;
  float pos;
  float vel;
  union {
    struct {
      uint8_t buf;
      uint32_t enable : 1;
      enum packet_to_hv_cmd_type_t {
        VOLT_MODE = 0,
        CURRENT_MODE,
      } cmd_type : 1;
      enum packet_to_hv_phase_type_t {
        PHASE_90_3PH = 0,
        PHASE_90_4PH,
        PHASE_120_3PH,
        PHASE_180_2PH,
        PHASE_180_3PH,
      } phase_type : 3;
      uint32_t ignore_fault_pin : 1;
      // short-circuit braking (f3 io.c): sbrake asks for it now, sbrake_arm
      // lets the f3 brake on its own when packets stop
      uint32_t sbrake : 1;
      uint32_t sbrake_arm : 1;
    } flags;
    uint32_t padding;
  };
} packet_to_hv_t;
_Static_assert(sizeof(packet_to_hv_t) == 32, "packet_to_hv_t size error");

// config data for f3. Append only: between flashing the f4 and pushing the
// matching f3 image, the old f3 must still find its words where it expects.
typedef union {
  struct f3_config_data_temp {
    float r;
    float l;
    float psi;
    float cur_bw;
    float cur_ff;
    float cur_ind;
    float max_y;
    float max_cur;
    float dac;
    float drop_k;     // dead time compensation, fraction of the ideal
    float lq;         // q axis inductance, 0 = same as l
    float emf_run;    // emf0: 1 accumulate, 0 hold, -1 clear
    float emf_sel;    // emf0 result returned in emf_val
    float emf_pp;     // emf0 pole pairs
    float drop_knee;  // dead time compensation curve knee [A], 0 = latched sign
    float unused0;    // was the f3 observer's obs_mode, kept for the layout
    float unused1;    // was the f3 observer's obs_bw
  } pins;
  float data[sizeof(struct f3_config_data_temp) / 4];
} f3_config_data_t;

//status reply from f3
typedef union {
  struct f3_state_data_temp {
    float u_fb;
    float v_fb;
    float w_fb;
    float hv_temp;
    float mot_temp;
    float core_temp;
    float dc_volt;
    float pwm_volt;
    float y;
    float emf_val;  // emf0 result number emf_sel
    float unused0;  // was the f3 observer's obs_err, kept for the layout
    float unused1;  // was the f3 observer's obs_vel
    float pwm_freq;  // PWM and rt rate the image was built for [Hz]
    float link_to;  // f3 rt ticks spent in link timeout since its boot (ls0.timeout)
  } pins;
  float data[sizeof(struct f3_state_data_temp) / 4];
} f3_state_data_t;

#pragma pack(pop)


//fault state
typedef enum {
  DISABLED = 0,
  ENABLED,
  PHASING,
  SOFT_FAULT,
  HARD_FAULT,
  LED_TEST,
  DELAYED_ENABLED,
  DELAYED_DISABLED
} state_t;

//fault error codes
typedef enum {
  NO_ERROR = 0,
  CMD_ERROR,
  MOT_FB_ERROR,
  COM_FB_ERROR,
  JOINT_FB_ERROR,
  POS_ERROR,
  SAT_ERROR,
  MOT_TEMP_ERROR,
  HV_CRC_ERROR,
  HV_TIMEOUT_ERROR,
  HV_TEMP_ERROR,
  HV_VOLT_ERROR,
  HV_FAULT_ERROR,
  HV_CURRENT_OFFSET_FAULT,
  HV_OVERCURRENT_RMS,
  HV_OVERCURRENT_PEAK,
  HV_OVERCURRENT_HW,
  IPM_TEMP_ERROR,
} fault_t;

//check if structs can be send at 5kHz with DATABAUD
_Static_assert(sizeof(packet_to_hv_t) <= DATABAUD / 11 / 5000 - 1 - 5, "to_hv struct to large");
_Static_assert(sizeof(packet_from_hv_t) <= DATABAUD / 11 / 5000 - 1 - 5, "from_hv struct to large");
//check if structs are word aligned
_Static_assert(!(sizeof(packet_to_hv_t) % 4), "to_hv struct not word aligned");
_Static_assert(!(sizeof(packet_from_hv_t) % 4), "from_hv struct not word aligned");
