/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <stdio.h>
#include "esp_err.h"

#include "py/nlr.h"
#include "py/runtime.h"
#include "modmachine.h"
#include "mphalport.h"
#include "soc/gpio_periph.h"

// Forward dec'l
const mp_obj_type_t machine_dffont_type;


#include "lvgl.h"

// 伪造字形数据（假设一个8x8字模，简单的矩阵）
static const uint8_t fake_glyph_bitmap[] = {
    0x3C, 0x42, 0x81, 0xA5, 0x81, 0x42, 0x3C, 0x00  // 一个简单的字模
};

// 获取字形描述
static bool my_get_glyph_dsc(const lv_font_t *font, lv_font_glyph_dsc_t *dsc, uint32_t unicode_letter, uint32_t unicode_letter_next) {
    // 伪造字形描述
    dsc->adv_w = 8 * 8;   // 字符宽度 (以 1/16 像素为单位)
    dsc->box_w = 8;       // 字模宽度
    dsc->box_h = 8;       // 字模高度
    dsc->ofs_x = 0;       // X 偏移
    dsc->ofs_y = 0;       // Y 偏移
    mp_printf(&mp_plat_print,"my_get_glyph_dsc\r\n");

    return true;  // 表示成功获取
}

// 获取字模位图
static const uint8_t* my_get_glyph_bitmap(const lv_font_t *font, uint32_t unicode_letter) {
    mp_printf(&mp_plat_print,"my_get_glyph_bitmap\r\n");
    return fake_glyph_bitmap;  // 返回伪造字模
}

// 创建字体对象
static lv_font_t my_custom_font = {
    .get_glyph_dsc = my_get_glyph_dsc,
    .get_glyph_bitmap = my_get_glyph_bitmap,
    .line_height = 16,  // 行高
    .base_line = 0
};

typedef struct {
    mp_obj_base_t base;
    const lv_font_t *font;  
} lv_font_obj_t;


static mp_obj_t lv_font_obj_get_font(mp_obj_t self_in) {
    lv_font_obj_t *self = MP_OBJ_TO_PTR(self_in);
    //return MP_OBJ_FROM_PTR(&self->font); 
    return MP_OBJ_NEW_SMALL_INT((intptr_t)(self->font));
    //return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(lv_font_obj_get_font_obj, lv_font_obj_get_font);

static mp_obj_t extern_font_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    lv_font_obj_t *self = m_new_obj(lv_font_obj_t);
    self->base.type = &machine_exfont_type; 
    self->font = &lv_font_montserrat_16;
    return MP_OBJ_FROM_PTR(self); 
}

static void extern_font_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    mp_printf(print, "<Exfont object>");
}

static const mp_rom_map_elem_t lv_font_obj_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_font), MP_ROM_PTR(&lv_font_obj_get_font_obj) },  
};

static MP_DEFINE_CONST_DICT(lv_font_obj_locals_dict, lv_font_obj_locals_dict_table);


MP_DEFINE_CONST_OBJ_TYPE(
    machine_exfont_type,
    MP_QSTR_Exfont,
    MP_TYPE_FLAG_NONE,
    print, extern_font_print,
    make_new, extern_font_make_new,
    locals_dict, &lv_font_obj_locals_dict
    );
