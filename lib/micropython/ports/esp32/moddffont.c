#include "py/obj.h"
#include "py/runtime.h"
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
    const lv_font_t *font;  // 包含 lv_font_t 指针
} lv_font_obj_t;

// Python接口：获取字体对象
static mp_obj_t moddffont_get_font(void) {
    lv_font_obj_t *font_obj = m_new_obj(lv_font_obj_t);
    font_obj->base.type = &mp_type_object;  // 自定义类型，确保类型正确
    font_obj->font = &lv_font_montserrat_16;
    mp_printf(&mp_plat_print,"Font pointer: %p\r\n", font_obj->font);
    return MP_OBJ_FROM_PTR(font_obj);  // 返回包装对象
}
static MP_DEFINE_CONST_FUN_OBJ_0(moddffont_get_font_obj, moddffont_get_font);

// 模块方法表
static const mp_rom_map_elem_t moddffont_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_moddffont) },
    { MP_ROM_QSTR(MP_QSTR_get_font), MP_ROM_PTR(&moddffont_get_font_obj) },
};

static MP_DEFINE_CONST_DICT(moddffont_module_globals, moddffont_module_globals_table);

// 定义模块
const mp_obj_module_t moddffont_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&moddffont_module_globals,
};

// 属性访问：获取字体
static mp_obj_t lv_font_obj_get_font(mp_obj_t self_in) {
    lv_font_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return MP_OBJ_FROM_PTR(self->font);
}

// Python接口：字体对象的属性定义
static const mp_rom_map_elem_t lv_font_obj_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_font), MP_ROM_PTR(&lv_font_obj_get_font) },  // 暴露 font 属性
};

static MP_DEFINE_CONST_DICT(lv_font_obj_locals_dict, lv_font_obj_locals_dict_table);



MP_DEFINE_CONST_OBJ_TYPE(
    lv_font_obj_type,
    MP_QSTR_LvFontObj,
    MP_TYPE_FLAG_NONE,
    make_new, NULL,
    locals_dict, &lv_font_obj_locals_dict
    );

// 注册模块
MP_REGISTER_MODULE(MP_QSTR_moddffont, moddffont_user_cmodule);