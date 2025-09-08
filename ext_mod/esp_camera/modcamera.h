/*
 * Copyright [2021] Mauro Riva <info@lemariva.com>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MICROPY_INCLUDED_ESP32_MODCAMERA_H
#define MICROPY_INCLUDED_ESP32_MODCAMERA_H

enum { OV2640, OV7725, OV5640};

#define TAG "camera"

//WROVER-KIT PIN Map
#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   -1 //software reset will be performed
#define CAM_PIN_XCLK     7
#define CAM_PIN_SIOD    47 // SDA
#define CAM_PIN_SIOC    48 // SCL

#define CAM_PIN_D7      6
#define CAM_PIN_D6      15
#define CAM_PIN_D5      16
#define CAM_PIN_D4      18
#define CAM_PIN_D3      9
#define CAM_PIN_D2      11
#define CAM_PIN_D1      10
#define CAM_PIN_D0       8
#define CAM_PIN_VSYNC   4
#define CAM_PIN_HREF    5
#define CAM_PIN_PCLK    17
#define XCLK_FREQ_10MHz    15000000//10000000
#define XCLK_FREQ_20MHz    20000000
#define XCLK_FREQ_15MHz    15000000

//White Balance
#define WB_NONE     0
#define WB_SUNNY    1
#define WB_CLOUDY   2
#define WB_OFFICE   3
#define WB_HOME     4

//Special Effect  
#define EFFECT_NONE    0
#define EFFECT_NEG     1
#define EFFECT_BW      2
#define EFFECT_RED     3
#define EFFECT_GREEN   4
#define EFFECT_BLUE    5
#define EFFECT_RETRO   6             


#endif
