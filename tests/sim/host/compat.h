// Forced into every host compile (-include): what newlib's math.h has and
// glibc's lacks.
#pragma once
#include <math.h>
#ifndef M_SQRT3
#define M_SQRT3 1.73205080756887729353
#endif
#ifndef M_SQRT1_3
#define M_SQRT1_3 0.57735026918962576451
#endif

#ifdef SIDE_HOOKS
// the firmware's printf and sscanf go through host/side.c (32-bit long
// formats, output tagged per side). After stdio.h, so its own declarations
// keep their names.
#include <stdio.h>
int side_printf(const char *fmt, ...);
int side_sscanf(const char *str, const char *fmt, ...);
#define printf side_printf
#define sscanf side_sscanf
#endif
