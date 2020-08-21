clear
clc

syms f11 f12 f13 f21 f22 f23 f31 f32 f33
F=[f11, f12, f13; f21, f22, f23; f31 f32 f33];
F*transpose(F)
transpose(F)*F