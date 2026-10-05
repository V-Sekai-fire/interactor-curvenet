/**************************************************************************/
/*  df32.h                                                                */
/**************************************************************************/
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include <cmath>

// Double-float (hi + lo) scalar; error-free transforms follow anny_blend_backward.slang. Build with -ffp-contract=off.
namespace cassie_graph_port {

inline void two_sum(float a, float b, float &hi, float &lo) {
	float h = a + b;
	float bb = h - a;
	float ah = h - bb;
	float lo_a = a - ah;
	float lo_b = b - bb;
	hi = h;
	lo = lo_a + lo_b;
}

inline void quick_two_sum(float a, float b, float &hi, float &lo) {
	float h = a + b;
	float t = h - a;
	hi = h;
	lo = b - t;
}

inline void two_prod(float a, float b, float &hi, float &lo) {
	float h = a * b;
	hi = h;
	lo = std::fma(a, b, -h);
}

struct df32 {
	float hi = 0.0f;
	float lo = 0.0f;

	df32() {}
	df32(float p_hi) :
			hi(p_hi) {}
	df32(int p_v) :
			hi((float)p_v) {}
	df32(double p_v) {
		hi = (float)p_v;
		lo = (float)(p_v - (double)hi);
	}
	df32(float p_hi, float p_lo) :
			hi(p_hi), lo(p_lo) {}
	double to_double() const { return (double)hi + (double)lo; }
	explicit operator float() const { return hi + lo; }
};

inline df32 operator+(df32 x, df32 y) {
	float sh = 0.0f;
	float sl = 0.0f;
	two_sum(x.hi, y.hi, sh, sl);
	float sl2 = sl + (x.lo + y.lo);
	df32 z;
	quick_two_sum(sh, sl2, z.hi, z.lo);
	return z;
}
inline df32 operator-(df32 x) { return df32(-x.hi, -x.lo); }
inline df32 operator-(df32 x, df32 y) { return x + (-y); }
inline df32 operator*(df32 x, df32 y) {
	float ph = 0.0f;
	float pl = 0.0f;
	two_prod(x.hi, y.hi, ph, pl);
	pl = pl + (x.hi * y.lo + x.lo * y.hi);
	df32 z;
	quick_two_sum(ph, pl, z.hi, z.lo);
	return z;
}
inline df32 operator/(df32 x, df32 y) {
	float q1 = x.hi / y.hi;
	df32 r = x - y * df32(q1);
	float q2 = r.hi / y.hi;
	df32 z;
	quick_two_sum(q1, q2, z.hi, z.lo);
	return z;
}
inline df32 &operator+=(df32 &x, df32 y) { return x = x + y; }
inline df32 &operator-=(df32 &x, df32 y) { return x = x - y; }
inline bool operator<(df32 x, df32 y) { return x.hi < y.hi || (x.hi == y.hi && x.lo < y.lo); }
inline bool operator>(df32 x, df32 y) { return y < x; }
inline bool operator<=(df32 x, df32 y) { return !(y < x); }
inline bool operator>=(df32 x, df32 y) { return !(x < y); }
inline bool operator==(df32 x, df32 y) { return x.hi == y.hi && x.lo == y.lo; }
inline bool operator!=(df32 x, df32 y) { return !(x == y); }

inline df32 sqrt(df32 a) {
	if (!(a.hi > 0.0f)) {
		return df32(0.0f);
	}
	float s = std::sqrt(a.hi);
	float ph = 0.0f;
	float pl = 0.0f;
	two_prod(s, s, ph, pl);
	df32 r = a - df32(ph, pl);
	float c = r.hi / (2.0f * s);
	df32 z;
	quick_two_sum(s, c, z.hi, z.lo);
	return z;
}
inline df32 fabs(df32 a) { return a.hi < 0.0f || (a.hi == 0.0f && a.lo < 0.0f) ? -a : a; }
inline df32 floor(df32 a) {
	float fh = std::floor(a.hi);
	if (fh != a.hi) {
		return df32(fh);
	}
	df32 z;
	quick_two_sum(fh, std::floor(a.lo), z.hi, z.lo);
	return z;
}
inline double to_double(df32 a) { return a.to_double(); }
inline double to_double(float a) { return (double)a; }

} // namespace cassie_graph_port
