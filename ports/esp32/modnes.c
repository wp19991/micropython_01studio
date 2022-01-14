/**
	******************************************************************************
	* Copyright (C), 2021 -2023, 01studio Tech. Co., Ltd.https://www.01studio.cc/
	* File Name				:	modusbcam.c
	* Author				:	Folktale
	* Version				:	v1.0
	* date					:	2021/8/05
	* Description			:	
	******************************************************************************
**/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"

#include "lib/oofatfs/ff.h"
#include "extmod/vfs.h"
#include "extmod/vfs_fat.h"

#include "py/objstr.h"
#include "py/objlist.h"
#include "py/stream.h"

#if MICROPY_HW_NESEMU

#if MICROPY_HW_NESEMU
#include "app_nes_start.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task.h"

#if MICROPY_ENABLE_TFTLCD
#include "modtftlcd.h"
#endif

//===================================================================================================================
typedef struct _nesemu_obj_t {
  mp_obj_base_t base;
}nesemu_obj_t;

//=====================================================================================================================

//----------------------------------------------------------------------------------
STATIC mp_obj_t nesemu_deinit(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

#if MICROPY_HW_NESEMU
app_nes_start("/flash/test.nes");
#endif

vTaskDelay(2000 / portTICK_RATE_MS);

return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(nesemu_deinit_obj,0, nesemu_deinit);

//----------------------------------------------------------------------------------
STATIC mp_obj_t nesemu_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {

	mp_arg_check_num(n_args, n_kw, 0, MP_OBJ_FUN_ARGS_MAX, true);
	
	nesemu_obj_t *nesemu_obj;
	nesemu_obj = m_new_obj(nesemu_obj_t);
	nesemu_obj->base.type = type;

	return MP_OBJ_FROM_PTR(nesemu_obj);
}

/******************************************************************************/
STATIC const mp_rom_map_elem_t nesemu_locals_dict_table[] = {
	
	{ MP_ROM_QSTR(MP_QSTR__name__), MP_ROM_QSTR(MP_QSTR_game_nes) },
	{ MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&nesemu_deinit_obj) },

};
STATIC MP_DEFINE_CONST_DICT(nesemu_locals_dict,nesemu_locals_dict_table);

const mp_obj_type_t game_nesemu_type = {
    { &mp_type_type },
    .name = MP_QSTR_NES,
    .make_new = nesemu_make_new,
    .locals_dict = (mp_obj_dict_t *)&nesemu_locals_dict,
};

#endif

