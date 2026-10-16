#pragma once

float minus(float a, float b);
float mod(float a);
int modi(int v, int m);
int quadrant(float a);
void sincos_fast(float x, float *sin, float *cos);
float atan2_fast(float y, float x);
float err_filter(float *ctx, float max, float dens, float err);