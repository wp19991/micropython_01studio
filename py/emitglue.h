/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
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
#ifndef MICROPY_INCLUDED_PY_EMITGLUE_H
#define MICROPY_INCLUDED_PY_EMITGLUE_H

#include "py/obj.h"
#include "py/bc.h"

// These variables and functions glue the code emitters to the runtime.

// These must fit in 8 bits; see scope.h
enum {
    MP_EMIT_OPT_NONE,
    MP_EMIT_OPT_BYTECODE,
    MP_EMIT_OPT_NATIVE_PYTHON,
    MP_EMIT_OPT_VIPER,
    MP_EMIT_OPT_ASM,
};

typedef enum {
    MP_CODE_UNUSED,
    MP_CODE_RESERVED,
    MP_CODE_BYTECODE,
    MP_CODE_NATIVE_PY,
    MP_CODE_NATIVE_VIPER,
    MP_CODE_NATIVE_ASM,
} mp_raw_code_kind_t;

typedef struct _mp_qstr_link_entry_t {
    uint16_t off;
    uint16_t qst;
} mp_qstr_link_entry_t;

typedef struct _mp_raw_code_t {
    mp_uint_t kind: 3; // of type mp_raw_code_kind_t
    mp_uint_t scope_flags: 7;
    mp_uint_t n_pos_args: 11;
    const void *fun_data;
    const mp_uint_t *const_table;
#if MICROPY_PERSISTENT_CODE_SAVE
    size_t fun_data_len;
    uint16_t n_obj;
    uint16_t n_raw_code;
#if MICROPY_PY_SYS_SETTRACE
    mp_bytecode_prelude_t prelude;
    // line_of_definition is a Python source line where the raw_code was
    // created e.g. MP_BC_MAKE_FUNCTION. This is different from lineno info
    // stored in prelude, which provides line number for first statement of
    // a function. Required to properly implement "call" trace event.
    mp_uint_t line_of_definition;
#endif
#if MICROPY_EMIT_MACHINE_CODE
    uint16_t prelude_offset;
    uint16_t n_qstr;
    mp_qstr_link_entry_t *qstr_link;
#endif
#endif
#if MICROPY_EMIT_MACHINE_CODE
    mp_uint_t type_sig; // for viper, compressed as 2-bit types; ret is MSB, then arg0, arg1, etc
#endif
} mp_raw_code_t;

mp_raw_code_t *mp_emit_glue_new_raw_code(void);

void mp_emit_glue_assign_bytecode(mp_raw_code_t *rc, const byte *code,
#if MICROPY_PERSISTENT_CODE_SAVE || MICROPY_DEBUG_PRINTERS
        size_t len,
#endif
                                  const mp_uint_t *const_table,
#if MICROPY_PERSISTENT_CODE_SAVE
        uint16_t n_obj, uint16_t n_raw_code,
#endif
                                  mp_uint_t scope_flags);

void mp_emit_glue_assign_native(mp_raw_code_t *rc, mp_raw_code_kind_t kind, void *fun_data, mp_uint_t fun_len,
                                const mp_uint_t *const_table,
#if MICROPY_PERSISTENT_CODE_SAVE
        uint16_t prelude_offset,
        uint16_t n_obj, uint16_t n_raw_code,
        uint16_t n_qstr, mp_qstr_link_entry_t *qstr_link,
#endif
                                mp_uint_t n_pos_args, mp_uint_t scope_flags, mp_uint_t type_sig);

mp_obj_t mp_make_function_from_raw_code(const mp_raw_code_t *rc, mp_obj_t def_args, mp_obj_t def_kw_args);

mp_obj_t mp_make_closure_from_raw_code(const mp_raw_code_t *rc, mp_uint_t n_closed_over, const mp_obj_t *args);

#endif // MICROPY_INCLUDED_PY_EMITGLUE_H
