/**************************************************************************/
/*  cassie_graph_port.cpp                                                 */
/**************************************************************************/
/* Ported from CASSIE (Yu, Arora, Stanko, Baerentzen, Singh, Bousseau),   */
/* MIT. Copyright (c) 2021 Emilie Yu and the CASSIE authors.              */
/*                                                                        */
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

#include "cassie_graph_port.h"
#include "df32.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

namespace cassie_graph_port {

// One-line switch: define CASSIE_GRAPH_F32 to run the graph in plain float, as Unity did.
#ifdef CASSIE_GRAPH_F32
typedef float real;
real Sqrt(real x) { return (float)std::sqrt((double)x); }
real Fabs(real x) { return std::fabs(x); }
real Floor(real x) { return std::floor(x); }
#else
typedef df32 real;
real Sqrt(real x) { return sqrt(x); }
real Fabs(real x) { return fabs(x); }
real Floor(real x) { return floor(x); }
#endif
real Max(real a, real b) { return a < b ? b : a; }
real Min(real a, real b) { return b < a ? b : a; }
real Fmod(real a, real b) { return real(std::fmod(to_double(a), to_double(b))); }


namespace {

// Stands in for the C# exceptions (KeyNotFound, NullReference) that abort a Unity event mid-way.
struct CsException : std::runtime_error {
	explicit CsException(const char *p_what) :
			std::runtime_error(p_what) {}
};

template <class T>
T *nn(T *p) {
	if (p == nullptr) {
		throw CsException("NullReferenceException");
	}
	return p;
}

const real kEpsilon = real(std::numeric_limits<float>::denorm_min());
const real kInf = real(std::numeric_limits<float>::infinity());
const real kPi = real(3.14159274f);
const real kRad2Deg = 57.29578f;
const real kDeg2Rad = 0.0174532924f;

struct V3 {
	real x = 0.0f;
	real y = 0.0f;
	real z = 0.0f;
};

V3 v3(real x, real y, real z) {
	V3 r;
	r.x = x;
	r.y = y;
	r.z = z;
	return r;
}
V3 operator+(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
V3 operator-(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
V3 operator-(V3 a) { return v3(-a.x, -a.y, -a.z); }
V3 operator*(V3 a, real s) { return v3(a.x * s, a.y * s, a.z * s); }
V3 operator*(real s, V3 a) { return v3(a.x * s, a.y * s, a.z * s); }
V3 operator/(V3 a, real s) { return v3(a.x / s, a.y / s, a.z / s); }
real Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 Cross(V3 a, V3 b) { return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
real Mag(V3 a) { return Sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
V3 Normalized(V3 a) {
	real m = Mag(a);
	if (m > 1e-5f) {
		return a / m;
	}
	return V3();
}
real Distance(V3 a, V3 b) { return Mag(a - b); }
real Clamp(real v, real lo, real hi) { return v < lo ? lo : (v > hi ? hi : v); }
real Acos(real f) { return real(std::acos(to_double(f))); }
bool Approximately(real a, real b) {
	return Fabs(b - a) < Max(1e-6f * Max(Fabs(a), Fabs(b)), kEpsilon * 8.0f);
}

// Quaternion.AngleAxis(deg, axis) * v, with Unity's quaternion-vector product.
V3 RotateAngleAxis(real p_degrees, V3 p_axis, V3 v) {
	real mag = Mag(p_axis);
	real qx = 0.0f;
	real qy = 0.0f;
	real qz = 0.0f;
	real qw = 1.0f;
	if (mag > 1e-6f) {
		real half = (p_degrees * kDeg2Rad) * 0.5f;
		qw = real(std::cos(to_double(half)));
		real s = real(std::sin(to_double(half))) / mag;
		qx = s * p_axis.x;
		qy = s * p_axis.y;
		qz = s * p_axis.z;
	}
	real n1 = qx * 2.0f;
	real n2 = qy * 2.0f;
	real n3 = qz * 2.0f;
	real n4 = qx * n1;
	real n5 = qy * n2;
	real n6 = qz * n3;
	real n7 = qx * n2;
	real n8 = qx * n3;
	real n9 = qy * n3;
	real n10 = qw * n1;
	real n11 = qw * n2;
	real n12 = qw * n3;
	return v3((1.0f - (n5 + n6)) * v.x + (n7 - n12) * v.y + (n8 + n11) * v.z,
			(n7 + n12) * v.x + (1.0f - (n4 + n6)) * v.y + (n9 - n10) * v.z,
			(n8 - n11) * v.x + (n9 + n10) * v.y + (1.0f - (n4 + n5)) * v.z);
}

struct PointOnCurve {
	real t = 0.0f;
	V3 position;
};

struct CubicBezier {
	V3 c0, c1, c2, c3;
	CubicBezier(V3 P0, V3 P1, V3 P2, V3 P3) {
		c0 = P0;
		c1 = 3.0f * (P1 - P0);
		c2 = 3.0f * (P0 - 2.0f * P1 + P2);
		c3 = -P0 + 3.0f * P1 - 3.0f * P2 + P3;
	}
	V3 Calculate(real t) const {
		real t2 = t * t;
		real t3 = t2 * t;
		return c0 + c1 * t + c2 * t2 + c3 * t3;
	}
};

// Curve.cs + BezierCurve.cs + LineCurve.cs, the graph-facing members.
class Curve {
public:
	bool is_line = false;
	V3 A, B;
	std::vector<CubicBezier> beziers;
	std::vector<V3> ctrl;

	explicit Curve(const std::vector<V3> &p_ctrl) :
			ctrl(p_ctrl) {
		if (p_ctrl.size() == 2) {
			is_line = true;
			A = p_ctrl[0];
			B = p_ctrl[1];
			return;
		}
		int n = (int)p_ctrl.size() / 3;
		for (int i = 0; i < n; i++) {
			beziers.push_back(CubicBezier(p_ctrl[i * 3], p_ctrl[i * 3 + 1], p_ctrl[i * 3 + 2], p_ctrl[i * 3 + 3]));
		}
	}

	void Convert(real t, int &r_idx, real &r_u) const {
		int n = (int)beziers.size();
		if (Approximately(t, 1.0f)) {
			r_idx = n - 1;
			r_u = 1.0f;
			return;
		}
		int idx = (int)to_double(Floor(t * real(n)));
		r_u = t * real(n) - real(idx);
		if (idx < 0 || idx >= n) {
			throw CsException("IndexOutOfRangeException");
		}
		r_idx = idx;
	}

	V3 GetPoint(real t) const {
		if (is_line) {
			real c = Clamp(t, 0.0f, 1.0f);
			return v3(A.x + (B.x - A.x) * c, A.y + (B.y - A.y) * c, A.z + (B.z - A.z) * c);
		}
		int idx = 0;
		real u = 0.0f;
		Convert(t, idx, u);
		return beziers[idx].Calculate(u);
	}

	V3 GetTangent(real t) const {
		real delta = 0.001f;
		real t1 = t - delta;
		real t2 = t + delta;
		if (t1 < 0.0f) {
			t1 = 0.0f;
		}
		if (t2 > 1.0f) {
			t2 = 1.0f;
		}
		V3 pt1 = GetPoint(t1);
		V3 pt2 = GetPoint(t2);
		return Normalized(pt2 - pt1);
	}

	real GetClosestPointParameter(V3 point, int slices, real start, real end, int iterations) const {
		if (iterations <= 0) {
			return (start + end) / 2.0f;
		}
		real step = (end - start) / real(slices);
		if (step < 10e-6f) {
			return (start + end) / 2.0f;
		}
		real tMin = 0.0f;
		real t = start;
		real dMin = kInf;
		real d = 0.0f;
		while (t <= end) {
			d = Distance(point, GetPoint(t));
			if (d < dMin) {
				tMin = t;
				dMin = d;
			}
			t += step;
		}
		d = Distance(point, GetPoint(end));
		if (d < dMin) {
			tMin = t;
			dMin = d;
		}
		return GetClosestPointParameter(point, slices, Max(tMin - step, 0.0f), Min(tMin + step, 1.0f), iterations - 1);
	}

	PointOnCurve Project(V3 point) const {
		PointOnCurve r;
		if (is_line) {
			V3 direction = Normalized(B - A);
			V3 proj = A;
			if (Dot(point - A, direction) <= 0.0f) {
				proj = A;
			} else if (Dot(point - B, direction) >= 0.0f) {
				proj = B;
			} else {
				proj = A + Dot(point - A, direction) * direction;
			}
			r.t = Clamp(Distance(proj, A) / Distance(A, B), 0.0f, 1.0f);
			r.position = proj;
			return r;
		}
		int nSlices = 10 * (int)beziers.size();
		real tMin = GetClosestPointParameter(point, nSlices, 0.0f, 1.0f, 5);
		return GetPointOnCurve(tMin);
	}

	PointOnCurve GetPointOnCurve(real t) const {
		PointOnCurve r;
		r.t = t;
		r.position = GetPoint(t);
		return r;
	}

	int GetBezierCountBetween(real from, real to) const {
		int a = 0;
		int b = 0;
		real u = 0.0f;
		Convert(from, a, u);
		Convert(to, b, u);
		return std::abs(b - a) + 1;
	}

	V3 ParallelTransport(V3 v, real from, real to) const {
		if (is_line) {
			return v;
		}
		int n = 20 * GetBezierCountBetween(from, to);
		real dt = (to - from) / real(n);
		V3 prev_tangent = GetTangent(from);
		V3 v_t = v;
		for (int i = 1; i <= n; i++) {
			real t = from + real(i) * dt;
			V3 tangent = GetTangent(t);
			V3 axis = Cross(prev_tangent, tangent);
			if (Mag(axis) > kEpsilon) {
				axis = Normalized(axis);
				real dot = Dot(prev_tangent, tangent);
				real theta = Acos(Clamp(dot, -1.0f, 1.0f));
				v_t = RotateAngleAxis(theta * kRad2Deg, axis, v_t);
			}
			prev_tangent = tangent;
		}
		return v_t;
	}
};

struct Plane {
	V3 n, p0;
	bool valid = false;
	V3 Mirror(V3 p) const {
		real a = Dot(n, p - p0);
		return p - 2.0f * a * n;
	}
};

Plane FitPlanePoints(const std::vector<V3> &points) {
	Plane result;
	V3 sum;
	for (const V3 &p : points) {
		sum = sum + p;
	}
	V3 centroid = sum / real((int)points.size());
	real xx = 0.0f, xy = 0.0f, xz = 0.0f, yy = 0.0f, yz = 0.0f, zz = 0.0f;
	for (const V3 &p : points) {
		V3 r = p - centroid;
		xx += r.x * r.x;
		xy += r.x * r.y;
		xz += r.x * r.z;
		yy += r.y * r.y;
		yz += r.y * r.z;
		zz += r.z * r.z;
	}
	real det_x = yy * zz - yz * yz;
	real det_y = xx * zz - xz * xz;
	real det_z = xx * yy - xy * xy;
	real det_max = Max(det_x, Max(det_y, det_z));
	if (det_max <= 0.0f) {
		return result;
	}
	V3 n = det_max == det_x ? v3(det_x, xz * yz - xy * zz, xy * yz - xz * yy) : det_max == det_y ? v3(xz * yz - xy * zz, det_y, xy * xz - yz * xx)
																										: v3(xy * yz - xz * yy, xy * xz - yz * xx, det_z);
	result.n = Normalized(n);
	result.p0 = centroid;
	result.valid = true;
	return result;
}

// Utils.FitPlane(point, vectors): max |dot(n, t)| is the sharpness error.
Plane FitPlaneVectors(V3 point, const std::vector<V3> &vectors, real &r_err) {
	r_err = kInf;
	if (vectors.size() < 2) {
		return Plane();
	}
	real score = 0.0f;
	std::vector<V3> pts(vectors.size() + 1);
	for (size_t i = 0; i < vectors.size(); i++) {
		pts[i] = point + vectors[i];
		real nonCollinearity = Mag(Cross(vectors[i], vectors[(i + 1) % vectors.size()]));
		if (nonCollinearity > score) {
			score = nonCollinearity;
		}
	}
	pts[vectors.size()] = point;
	if (score < 0.1f) {
		return Plane();
	}
	Plane P = FitPlanePoints(pts);
	if (!P.valid) {
		throw CsException("NullReferenceException");
	}
	real maxError = 0.0f;
	for (const V3 &t : vectors) {
		real error = Fabs(Dot(P.n, t));
		if (error > maxError) {
			maxError = error;
		}
	}
	r_err = maxError;
	return P;
}

class Graph;
class FinalStroke;
class Segment;
class Cycle;

// .NET reference-source HashSet<T>: slot order, free list and stored hashes, so iteration and stale-hash lookups match.
template <class T>
class CsHashSet {
public:
	typedef int (*HashFn)(T);
	typedef bool (*EqFn)(T, T);
	struct Slot {
		int hash = 0;
		int next = 0;
		T value = nullptr;
	};
	std::vector<int> buckets;
	std::vector<Slot> slots;
	int last_index = 0;
	int count = 0;
	int free_list = -1;
	HashFn hash_fn;
	EqFn eq_fn;

	CsHashSet(HashFn p_hash, EqFn p_eq) :
			hash_fn(p_hash), eq_fn(p_eq) {}

	static int GetPrime(int min) {
		static const int primes[] = { 3, 7, 11, 17, 23, 29, 37, 47, 59, 71, 89, 107, 131, 163, 197, 239, 293, 353, 431, 521, 631, 761, 919,
			1103, 1327, 1597, 1931, 2333, 2801, 3371, 4049, 4861, 5839, 7013, 8419, 10103, 12143, 14591, 17519, 21023, 25229, 30293, 36353,
			43627, 52361, 62851, 75431, 90523, 108631, 130363, 156437, 187751, 225307, 270371, 324449, 389357, 467237, 560689, 672827, 807403,
			968897, 1162687, 1395263, 1674319, 2009191, 2411033, 2893249, 3471899, 4166287, 4999559, 5999471, 7199369 };
		for (int p : primes) {
			if (p >= min) {
				return p;
			}
		}
		return min | 1;
	}
	int Hash(T v) const { return hash_fn(v) & 0x7FFFFFFF; }
	void Initialize() {
		int size = GetPrime(0);
		buckets.assign(size, 0);
		slots.assign(size, Slot());
	}
	void SetCapacity(int new_size) {
		std::vector<Slot> new_slots(new_size);
		for (int i = 0; i < last_index; i++) {
			new_slots[i] = slots[i];
		}
		std::vector<int> new_buckets(new_size, 0);
		for (int i = 0; i < last_index; i++) {
			int bucket = new_slots[i].hash % new_size;
			new_slots[i].next = new_buckets[bucket] - 1;
			new_buckets[bucket] = i + 1;
		}
		slots = new_slots;
		buckets = new_buckets;
	}
	bool Contains(T v) const {
		if (buckets.empty()) {
			return false;
		}
		int h = Hash(v);
		for (int i = buckets[h % (int)buckets.size()] - 1; i >= 0; i = slots[i].next) {
			if (slots[i].hash == h && eq_fn(slots[i].value, v)) {
				return true;
			}
		}
		return false;
	}
	bool Add(T v) {
		if (buckets.empty()) {
			Initialize();
		}
		int h = Hash(v);
		int bucket = h % (int)buckets.size();
		for (int i = buckets[bucket] - 1; i >= 0; i = slots[i].next) {
			if (slots[i].hash == h && eq_fn(slots[i].value, v)) {
				return false;
			}
		}
		int index = 0;
		if (free_list >= 0) {
			index = free_list;
			free_list = slots[index].next;
		} else {
			if (last_index == (int)slots.size()) {
				SetCapacity(GetPrime(2 * count));
				bucket = h % (int)buckets.size();
			}
			index = last_index;
			last_index++;
		}
		slots[index].hash = h;
		slots[index].value = v;
		slots[index].next = buckets[bucket] - 1;
		buckets[bucket] = index + 1;
		count++;
		return true;
	}
	bool Remove(T v) {
		if (buckets.empty()) {
			return false;
		}
		int h = Hash(v);
		int bucket = h % (int)buckets.size();
		int last = -1;
		for (int i = buckets[bucket] - 1; i >= 0; last = i, i = slots[i].next) {
			if (slots[i].hash == h && eq_fn(slots[i].value, v)) {
				if (last < 0) {
					buckets[bucket] = slots[i].next + 1;
				} else {
					slots[last].next = slots[i].next;
				}
				slots[i].hash = -1;
				slots[i].value = nullptr;
				slots[i].next = free_list;
				count--;
				if (count == 0) {
					last_index = 0;
					free_list = -1;
				} else {
					free_list = i;
				}
				return true;
			}
		}
		return false;
	}
	void Clear() {
		if (last_index > 0) {
			std::fill(slots.begin(), slots.end(), Slot());
			std::fill(buckets.begin(), buckets.end(), 0);
			last_index = 0;
			count = 0;
			free_list = -1;
		}
	}
	std::vector<T> Items() const {
		std::vector<T> out;
		for (int i = 0; i < last_index; i++) {
			if (slots[i].hash >= 0) {
				out.push_back(slots[i].value);
			}
		}
		return out;
	}
};

// .NET Dictionary<int, T>: enumeration walks entries, removal pushes a LIFO free list.
template <class T>
class CsDict {
public:
	struct Entry {
		int key = 0;
		T value = nullptr;
		bool used = false;
		int next_free = -1;
	};
	std::vector<Entry> entries;
	std::map<int, int> index;
	int free_list = -1;
	int free_count = 0;
	void Add(int k, T v) {
		if (index.count(k)) {
			throw CsException("ArgumentException");
		}
		int i = 0;
		if (free_count > 0) {
			i = free_list;
			free_list = entries[i].next_free;
			free_count--;
		} else {
			i = (int)entries.size();
			entries.push_back(Entry());
		}
		entries[i].key = k;
		entries[i].value = v;
		entries[i].used = true;
		entries[i].next_free = -1;
		index[k] = i;
	}
	T Find(int k) const {
		std::map<int, int>::const_iterator it = index.find(k);
		return it == index.end() ? nullptr : entries[it->second].value;
	}
	void Remove(int k) {
		std::map<int, int>::iterator it = index.find(k);
		if (it == index.end()) {
			return;
		}
		int i = it->second;
		index.erase(it);
		entries[i] = Entry();
		entries[i].next_free = free_list;
		free_list = i;
		free_count++;
	}
	std::vector<T> Values() const {
		std::vector<T> out;
		for (const Entry &e : entries) {
			if (e.used) {
				out.push_back(e.value);
			}
		}
		return out;
	}
};

class Node {
public:
	int ID;
	std::list<Segment *> Neighbors;
	V3 Position;
	V3 Normal;
	bool IsSharp = false;

	Node(V3 p_position, int p_id) :
			ID(p_id), Position(p_position) {}
	int IncidentCount() const { return (int)Neighbors.size(); }

	std::vector<V3> GetNeighbors();
	Segment *GetNext(Segment *s);
	Segment *GetPrevious(Segment *s);
	Segment *GetInPlane(Segment *s, V3 N, bool next);
	void UpdateNormal();
	void AddSegment(Segment *s);
	void RemoveSegment(Segment *s);
	bool TryRemove();
	V3 ProjectOnPlane(V3 v) const {
		if (Mag(Normal) > 0.0f) {
			return Normalized(v - Dot(v, Normal) * Normal);
		}
		return v;
	}
	static V3 ProjectOnPlane(V3 v, V3 N, bool normalize) {
		return normalize ? Normalized(v - Dot(v, N) * N) : (v - Dot(v, N) * N);
	}
	void SortSegments();
};

class Segment {
public:
	FinalStroke *Stroke;
	Node *endpoints[2] = { nullptr, nullptr };
	real params[2] = { 0.0f, 0.0f };
	int ID;

	Segment(int p_id, FinalStroke *s, real start, real end, Node *startNode, Node *endNode) :
			Stroke(s), ID(p_id) {
		SetStart(start, startNode);
		SetEnd(end, endNode);
	}
	void SetStart(real param, Node *node) { Replace(0, param, node); }
	void SetStart(Node *node) { SetStart(params[0], node); }
	void SetEnd(real param, Node *node) { Replace(1, param, node); }
	void SetEnd(Node *node) { SetEnd(params[1], node); }
	Node *GetStartNode() const { return endpoints[0]; }
	Node *GetEndNode() const { return endpoints[1]; }
	real GetStartParam() const { return params[0]; }
	real GetEndParam() const { return params[1]; }
	V3 GetPointAt(real u) const;
	void Delete() {
		for (Node *node : endpoints) {
			nn(node)->RemoveSegment(this);
		}
	}
	int WhichEndpoint(const Node *node) const { return node == endpoints[0] ? 0 : 1; }
	static int Other(int idx) { return (idx + 1) % 2; }
	Node *GetOpposite(const Node *from) const { return endpoints[Other(WhichEndpoint(from))]; }
	real GetParam(const Node *from) const { return params[WhichEndpoint(from)]; }
	bool IsInReverse(const Node *from) const { return WhichEndpoint(from) == 1; }
	V3 GetTangentAt(const Node *endpoint) const;
	V3 ProjectInPlane(const Node *n, V3 normal) const;
	V3 Transport(V3 v, const Node *to) const;
	void Replace(int idx, real param, Node *node) {
		params[idx] = param;
		if (node != nullptr) {
			if (endpoints[idx] != nullptr) {
				endpoints[idx]->RemoveSegment(this);
			}
			endpoints[idx] = node;
			node->AddSegment(this);
		}
	}
};

struct HalfSegment {
	Segment *segment;
	bool IsReversed;
	int GetSegmentID() const { return segment->ID; }
};

class Cycle {
public:
	std::list<HalfSegment> HalfSegments;
	bool userCreated;
	int patchID = -1;
	int hashCode = 0;
	bool updateHashCode = true;

	Cycle(bool p_user, const std::list<HalfSegment> &p_segments) :
			HalfSegments(p_segments), userCreated(p_user) {}

	int HashCode() {
		if (updateHashCode) {
			std::vector<int> ids;
			for (const HalfSegment &hs : HalfSegments) {
				ids.push_back(hs.GetSegmentID());
			}
			std::sort(ids.begin(), ids.end());
			uint32_t hash = 0x2D2816FEu;
			for (int e : ids) {
				hash = 31u * hash + (uint32_t)e;
			}
			hashCode = (int)hash;
			updateHashCode = false;
		}
		return hashCode;
	}
	bool Equals(Cycle *other) { return other != nullptr && other->HashCode() == HashCode(); }

	std::list<HalfSegment>::iterator FindNode(const Segment *s) {
		for (std::list<HalfSegment>::iterator it = HalfSegments.begin(); it != HalfSegments.end(); ++it) {
			if (it->GetSegmentID() == nn(s)->ID) {
				return it;
			}
		}
		return HalfSegments.end();
	}
	void RepairAt(Segment *oldSegment, Segment *newSegment, bool inStrokeOrder) {
		std::list<HalfSegment>::iterator oldHS = FindNode(oldSegment);
		if (oldHS == HalfSegments.end()) {
			return;
		}
		HalfSegment newHS;
		newHS.segment = newSegment;
		newHS.IsReversed = oldHS->IsReversed;
		bool after = inStrokeOrder ? !oldHS->IsReversed : oldHS->IsReversed;
		if (after) {
			HalfSegments.insert(std::next(oldHS), newHS);
		} else {
			HalfSegments.insert(oldHS, newHS);
		}
		updateHashCode = true;
	}
	bool Contains(const Segment *s) {
		for (const HalfSegment &hs : HalfSegments) {
			if (hs.GetSegmentID() == s->ID) {
				return true;
			}
		}
		return false;
	}
	void Remove(Segment *s) {
		if (!Contains(s)) {
			return;
		}
		HalfSegments.erase(FindNode(s));
		updateHashCode = true;
	}
	bool Contains(Segment *s1, Segment *s2) {
		std::list<HalfSegment>::iterator hs1 = FindNode(s1);
		if (hs1 == HalfSegments.end()) {
			return false;
		}
		int id2 = nn(s2)->ID;
		std::list<HalfSegment>::iterator nx = std::next(hs1);
		bool hasNext = nx != HalfSegments.end();
		bool hasPrev = hs1 != HalfSegments.begin();
		if ((hasNext && nx->GetSegmentID() == id2) || (!hasNext && HalfSegments.front().GetSegmentID() == id2) || (hasPrev && std::prev(hs1)->GetSegmentID() == id2) || (!hasPrev && HalfSegments.back().GetSegmentID() == id2)) {
			return true;
		}
		return false;
	}
	std::vector<int> StrokeIDs() const;
};

int CycleHash(Cycle *c) { return c->HashCode(); }
bool CycleEq(Cycle *a, Cycle *b) { return a->Equals(b); }
int SegmentHash(Segment *s) { return s->ID; }
bool SegmentEq(Segment *a, Segment *b) { return a == b; }

class SegmentCycles {
public:
	std::vector<Cycle *> cycles;
	void Add(Cycle *c) { cycles.push_back(c); }
	int CyclesCount() const { return (int)cycles.size(); }
	void Remove(Cycle *c) {
		int idx = -1;
		for (int i = 0; i < (int)cycles.size(); i++) {
			if (cycles[i]->Equals(c)) {
				idx = i;
			}
		}
		if (idx != -1) {
			cycles.erase(cycles.begin() + idx);
		}
	}
};

class Graph {
public:
	bool surfacing = true;
	std::vector<Cycle *> _toAddCache;
	CsHashSet<Cycle *> _toRemoveCache{ CycleHash, CycleEq };
	CsHashSet<Segment *> _updatedSegmentsCache{ SegmentHash, SegmentEq };
	CsHashSet<Cycle *> _cyclesToCheckCache{ CycleHash, CycleEq };
	std::map<int, Node *> _nodes;
	CsDict<Segment *> _segments;
	std::map<int, SegmentCycles> _cyclesBySegment;
	CsHashSet<Cycle *> _cycles{ CycleHash, CycleEq };
	int _segmentID = 0;
	int _nodeID = 0;

	std::vector<std::unique_ptr<Node>> node_pool;
	std::vector<std::unique_ptr<Segment>> segment_pool;
	std::vector<std::unique_ptr<Cycle>> cycle_pool;

	Node *NewNode(V3 position) {
		int id = _nodeID++;
		node_pool.push_back(std::unique_ptr<Node>(new Node(position, id)));
		Node *n = node_pool.back().get();
		_nodes[id] = n;
		return n;
	}
	Segment *NewSegment(FinalStroke *s, real start, real end, Node *startNode, Node *endNode, bool onNewStroke) {
		int id = _segmentID++;
		segment_pool.push_back(std::unique_ptr<Segment>(new Segment(id, s, start, end, startNode, endNode)));
		Segment *seg = segment_pool.back().get();
		_segments.Add(id, seg);
		if (surfacing && onNewStroke) {
			_updatedSegmentsCache.Add(seg);
		}
		return seg;
	}
	bool TryAddCycle(Cycle *newCycle) {
		if (_cycles.Contains(newCycle)) {
			return false;
		}
		for (const HalfSegment &hs : newCycle->HalfSegments) {
			_cyclesBySegment[hs.GetSegmentID()].Add(newCycle);
		}
		if (_toRemoveCache.Contains(newCycle)) {
			for (Cycle *old : _toRemoveCache.Items()) {
				if (old->Equals(newCycle)) {
					newCycle->patchID = old->patchID;
					break;
				}
			}
			_toRemoveCache.Remove(newCycle);
		} else {
			_toAddCache.push_back(newCycle);
		}
		_cycles.Add(newCycle);
		return true;
	}
	void SetStart(Segment *s, Node *n, bool updateNeighbors) {
		s->SetStart(n);
		if (updateNeighbors) {
			UpdateNeighborsCycles(s, n);
		}
	}
	void SetEnd(Segment *s, Node *n, bool updateNeighbors) {
		s->SetEnd(n);
		if (updateNeighbors) {
			UpdateNeighborsCycles(s, n);
		}
	}
	void SetEnd(Segment *s, real param, Node *n, bool updateNeighbors) {
		s->SetEnd(param, n);
		if (updateNeighbors) {
			UpdateNeighborsCycles(s, n);
		}
	}
	void RepairCycles(Segment *oldS, Segment *newS, bool inStrokeOrder = true) {
		std::map<int, SegmentCycles>::iterator it = _cyclesBySegment.find(oldS->ID);
		if (it == _cyclesBySegment.end()) {
			return;
		}
		std::vector<Cycle *> list = it->second.cycles;
		for (Cycle *cycle : list) {
			_cyclesToCheckCache.Remove(cycle);
			_cycles.Remove(cycle);
			cycle->RepairAt(oldS, newS, inStrokeOrder);
			_cyclesBySegment[newS->ID].Add(cycle);
			_cyclesToCheckCache.Add(cycle);
			_cycles.Add(cycle);
		}
	}
	void RemoveNode(Node *node) { _nodes.erase(node->ID); }
	void RemoveSegment(Segment *s) {
		Node *start = nn(s->GetStartNode());
		Node *end = nn(s->GetEndNode());
		UpdateNeighbors(s, start);
		UpdateNeighbors(s, end);
		s->Delete();
		if (start->TryRemove()) {
			RemoveNode(start);
		}
		if (end->TryRemove()) {
			RemoveNode(end);
		}
		_segments.Remove(s->ID);
		DeleteCycles(s);
		_updatedSegmentsCache.Remove(s);
	}
	void MendSegments(Segment *sKeep, Segment *sRemove) {
		std::map<int, SegmentCycles>::iterator it = _cyclesBySegment.find(sRemove->ID);
		if (it != _cyclesBySegment.end()) {
			for (Cycle *c : it->second.cycles) {
				_cycles.Remove(c);
				c->Remove(sRemove);
				_cycles.Add(c);
			}
		}
		_cyclesBySegment.erase(sRemove->ID);
		SetEnd(sKeep, sRemove->GetEndParam(), sRemove->GetEndNode(), false);
		_segments.Remove(sRemove->ID);
		sRemove->Delete();
		_updatedSegmentsCache.Remove(sRemove);
	}
	int ExistingCyclesCount(const Segment *s) const {
		std::map<int, SegmentCycles>::const_iterator it = _cyclesBySegment.find(s->ID);
		return it == _cyclesBySegment.end() ? 0 : it->second.CyclesCount();
	}
	void Update(std::vector<Cycle *> &r_add, std::vector<Cycle *> &r_remove) {
		r_add = _toAddCache;
		r_remove = _toRemoveCache.Items();
		_toAddCache.clear();
		_toRemoveCache.Clear();
	}
	bool ManualDeletePatch(int patchID) {
		for (Cycle *c : _cycles.Items()) {
			if (c->patchID == patchID) {
				RemoveCycle(c, true);
				return true;
			}
		}
		return false;
	}
	Segment *FindClosestSegment(V3 pos, bool lookAtNonManifold) {
		Segment *closest = nullptr;
		real minDist = 10.0f;
		for (Segment *s : _segments.Values()) {
			if (!lookAtNonManifold) {
				std::map<int, SegmentCycles>::iterator it = _cyclesBySegment.find(s->ID);
				if (it != _cyclesBySegment.end() && it->second.CyclesCount() >= 2) {
					continue;
				}
			}
			if (s->GetStartNode()->IncidentCount() < 2 || s->GetEndNode()->IncidentCount() < 2) {
				continue;
			}
			real avg = (Distance(s->GetStartNode()->Position, pos) + Distance(s->GetEndNode()->Position, pos) + Distance(s->GetPointAt(0.5f), pos)) / 3.0f;
			if (avg < minDist) {
				closest = s;
				minDist = avg;
			}
		}
		return closest;
	}
	Segment *FindClosestAmongNeighbors(V3 pos, Node *node, Segment *current, bool lookAtNonManifold) {
		Segment *closest = nullptr;
		real minDistNext = 10.0f;
		std::list<Segment *> copy = node->Neighbors;
		for (Segment *s : copy) {
			if (s == current) {
				continue;
			}
			if (!lookAtNonManifold && ExistingCyclesCount(s) >= 2) {
				break;
			}
			if (s->GetOpposite(node)->IncidentCount() < 2) {
				continue;
			}
			real dt = 1.0f / 5.0f;
			real dist = kInf;
			for (int i = 1; i <= 5; i++) {
				real d = Distance(s->GetPointAt(real(i) * dt), pos);
				if (d < dist) {
					dist = d;
				}
			}
			if (dist < minDistNext) {
				closest = s;
				minDistNext = dist;
			}
		}
		return closest;
	}
	bool TryFindCycleAt(V3 pos, bool lookAtNonManifold);
	void TryFindAllCycles();
	void UpdateNeighbors(Segment *s, Node *n) {
		if (!surfacing) {
			return;
		}
		if (n->Neighbors.size() == 2) {
			Segment *next = n->GetNext(s);
			if (next != nullptr) {
				_updatedSegmentsCache.Add(next);
			}
		}
		if (n->Neighbors.size() > 2) {
			Segment *next = n->GetNext(s);
			Segment *previous = n->GetPrevious(s);
			if (previous != nullptr) {
				_updatedSegmentsCache.Add(previous);
			}
			if (next != nullptr) {
				_updatedSegmentsCache.Add(next);
			}
		}
	}
	void UpdateNeighborsCycles(Segment *s, Node *n) {
		Segment *next = nn(n->GetNext(s));
		Segment *prev = n->GetPrevious(s);
		std::map<int, SegmentCycles>::iterator it = _cyclesBySegment.find(next->ID);
		if (it != _cyclesBySegment.end()) {
			std::vector<Cycle *> list = it->second.cycles;
			for (Cycle *c : list) {
				if (c->Contains(next, prev)) {
					_cyclesToCheckCache.Add(c);
				}
			}
		}
	}
	void RemoveCycle(Cycle *c, bool userTriggered = false) {
		for (const HalfSegment &hs : c->HalfSegments) {
			std::map<int, SegmentCycles>::iterator it = _cyclesBySegment.find(hs.GetSegmentID());
			if (it != _cyclesBySegment.end()) {
				it->second.Remove(c);
			}
		}
		_cycles.Remove(c);
		if (!userTriggered) {
			_toRemoveCache.Add(c);
		}
	}
	void DeleteCycles(Segment *s) {
		std::map<int, SegmentCycles>::iterator it = _cyclesBySegment.find(s->ID);
		if (it != _cyclesBySegment.end()) {
			std::vector<Cycle *> toDelete = it->second.cycles;
			for (Cycle *c : toDelete) {
				RemoveCycle(c);
			}
		}
		_cyclesBySegment.erase(s->ID);
	}
	bool TryReachCycle(Cycle *c, Segment *startSegment, Node *startNode) {
		int limit = 5;
		int i = 0;
		Segment *s1 = startNode->GetNext(startSegment);
		Segment *s2 = startNode->GetPrevious(startSegment);
		Node *node = startNode;
		while (node->Neighbors.size() > 1 && i < limit) {
			if (c->Contains(s1, s2)) {
				return true;
			}
			if (node->Neighbors.size() > 2) {
				return false;
			}
			node = nn(nn(s1)->GetOpposite(node));
			s2 = node->GetPrevious(s1);
			s1 = node->GetNext(s1);
			i++;
		}
		return false;
	}
	Segment *TryRemove(Cycle *c) {
		const HalfSegment &hs = c->HalfSegments.front();
		Segment *seed = _segments.Find(hs.GetSegmentID());
		if (seed == nullptr) {
			throw CsException("KeyNotFoundException");
		}
		RemoveCycle(c);
		return seed;
	}
	void TryFindCycle(Segment *startSegment) {
		Node *startNode = nn(startSegment->GetStartNode());
		if (startNode->IsSharp && !nn(startSegment->GetEndNode())->IsSharp) {
			startNode = startSegment->GetEndNode();
		}
		TryFindCycle(startSegment, startNode, false);
		TryFindCycle(startSegment, startNode, true);
	}
	void TryFindCycle(Segment *startSegment, Node *startNode, bool reversed);
};

class FinalStroke {
public:
	int ID = -1;
	std::unique_ptr<Curve> curve;
	std::list<Segment *> segments;
	Graph *_graph;

	explicit FinalStroke(Graph *g) :
			_graph(g) {}

	void SetCurve(const std::vector<V3> &ctrl, bool closedLoop) {
		curve.reset(new Curve(ctrl));
		if (!closedLoop) {
			Node *startNode = _graph->NewNode(curve->GetPoint(0.0f));
			Node *endNode = _graph->NewNode(curve->GetPoint(1.0f));
			segments.push_front(_graph->NewSegment(this, 0.0f, 1.0f, startNode, endNode, true));
		} else {
			Node *closingNode = _graph->NewNode(curve->GetPoint(0.0f));
			segments.push_front(_graph->NewSegment(this, 0.0f, 1.0f, closingNode, closingNode, true));
		}
	}
	void Destroy() {
		std::vector<Segment *> copy(segments.begin(), segments.end());
		for (Segment *s : copy) {
			_graph->RemoveSegment(s);
		}
	}
	std::list<Segment *>::iterator GetSegmentInListContaining(real param) {
		if (segments.empty()) {
			throw CsException("NullReferenceException");
		}
		std::list<Segment *>::iterator it = segments.begin();
		while (std::next(it) != segments.end() && (*it)->GetEndParam() < param) {
			++it;
		}
		return it;
	}
	PointOnCurve GetConstraint(V3 position, real snap) {
		PointOnCurve onCurve = curve->Project(position);
		if (!segments.empty()) {
			Segment *s = *GetSegmentInListContaining(onCurve.t);
			Node *closest = nullptr;
			real param = 0.0f;
			if (Distance(s->endpoints[0]->Position, onCurve.position) < Distance(s->endpoints[1]->Position, onCurve.position)) {
				closest = s->endpoints[0];
				param = s->params[0];
			} else {
				closest = s->endpoints[1];
				param = s->params[1];
			}
			if (Distance(closest->Position, onCurve.position) < snap) {
				onCurve = curve->GetPointOnCurve(param);
			}
		}
		return onCurve;
	}
	Node *GetClosest(Segment *segment, V3 position) {
		Node *l = nn(segment->GetStartNode());
		Node *r = nn(segment->GetEndNode());
		return Distance(l->Position, position) < Distance(r->Position, position) ? l : r;
	}
	Node *AddIntersectionOldStroke(PointOnCurve point, real threshold) {
		std::list<Segment *>::iterator seg = GetSegmentInListContaining(point.t);
		Node *newNode = _graph->NewNode(point.position);
		Node *closest = GetClosest(*seg, point.position);
		if (Distance(closest->Position, point.position) < threshold) {
			_graph->RemoveNode(newNode);
			return closest;
		}
		AddNode(newNode, seg, point, false);
		return newNode;
	}
	void AddIntersectionNewStroke(Node *node, PointOnCurve point, real threshold) {
		std::list<Segment *>::iterator seg = GetSegmentInListContaining(point.t);
		Node *closest = GetClosest(*seg, point.position);
		if (Distance(closest->Position, point.position) < threshold) {
			_graph->RemoveNode(closest);
			if ((*seg)->GetStartNode() == closest) {
				_graph->SetStart(*seg, node, true);
			}
			if ((*seg)->GetEndNode() == closest) {
				_graph->SetEnd(*seg, node, true);
			}
			return;
		}
		AddNode(node, seg, point, true);
	}
	void AddNode(Node *node, std::list<Segment *>::iterator seg, PointOnCurve point, bool onNewStroke) {
		Segment *old = *seg;
		Segment *newSegment = _graph->NewSegment(this, point.t, old->GetEndParam(), node, old->GetEndNode(), onNewStroke);
		segments.insert(std::next(seg), newSegment);
		_graph->SetEnd(old, point.t, node, false);
		if (!onNewStroke) {
			_graph->RepairCycles(old, newSegment);
		}
	}
	bool MendSegments(Node *mendAt, Segment *sA, Segment *sB) {
		std::list<Segment *>::iterator a = std::find(segments.begin(), segments.end(), sA);
		if (a == segments.end()) {
			throw CsException("NullReferenceException");
		}
		std::list<Segment *>::iterator left;
		std::list<Segment *>::iterator right;
		std::list<Segment *>::iterator an = std::next(a);
		if (an != segments.end() && *an == sB && sA->GetEndNode() == mendAt && sB->GetStartNode() == mendAt) {
			left = a;
			right = an;
		} else if (a != segments.begin() && *std::prev(a) == sB && sA->GetStartNode() == mendAt && sB->GetEndNode() == mendAt) {
			left = std::prev(a);
			right = a;
		} else {
			return false;
		}
		_graph->MendSegments(*left, *right);
		segments.erase(right);
		return true;
	}
};

V3 Segment::GetPointAt(real u) const {
	real t = params[0] + (params[1] - params[0]) * Clamp(u, 0.0f, 1.0f);
	return Stroke->curve->GetPoint(t);
}
V3 Segment::GetTangentAt(const Node *endpoint) const {
	int end = WhichEndpoint(endpoint);
	real u = params[end];
	real orientation = end == 0 ? 1.0f : -1.0f;
	return orientation * Stroke->curve->GetTangent(u);
}
V3 Segment::ProjectInPlane(const Node *n, V3 normal) const {
	V3 v = GetTangentAt(n);
	V3 proj = v - Dot(v, normal) * normal;
	if (Mag(proj) < 0.7f) {
		int nodeIdx = WhichEndpoint(n);
		V3 v2 = Normalized(nn(endpoints[Other(nodeIdx)])->Position - n->Position);
		V3 proj2 = v2 - Dot(v2, normal) * normal;
		if (Mag(proj2) > Mag(proj)) {
			return Normalized(proj2);
		}
	}
	return Normalized(proj);
}
V3 Segment::Transport(V3 v, const Node *to) const {
	int toIdx = WhichEndpoint(to);
	int fromIdx = Other(toIdx);
	return Stroke->curve->ParallelTransport(v, params[fromIdx], params[toIdx]);
}

std::vector<int> Cycle::StrokeIDs() const {
	std::vector<int> ids;
	for (const HalfSegment &hs : HalfSegments) {
		ids.push_back(hs.segment->Stroke->ID);
	}
	return ids;
}

std::vector<V3> Node::GetNeighbors() {
	std::vector<V3> t;
	for (Segment *s : Neighbors) {
		t.push_back(s->GetTangentAt(this));
	}
	return t;
}
Segment *Node::GetNext(Segment *s) {
	std::list<Segment *>::iterator it = std::find(Neighbors.begin(), Neighbors.end(), s);
	if (it == Neighbors.end()) {
		return nullptr;
	}
	std::list<Segment *>::iterator nx = std::next(it);
	return nx != Neighbors.end() ? *nx : Neighbors.front();
}
Segment *Node::GetPrevious(Segment *s) {
	std::list<Segment *>::iterator it = std::find(Neighbors.begin(), Neighbors.end(), s);
	if (it == Neighbors.end()) {
		return nullptr;
	}
	return it != Neighbors.begin() ? *std::prev(it) : Neighbors.back();
}
Segment *Node::GetInPlane(Segment *s, V3 N, bool next) {
	Segment *nextSegment = nullptr;
	real projMax = 0.0f;
	Segment *bestInPlane = nullptr;
	for (Segment *other : Neighbors) {
		if (other == s) {
			continue;
		}
		V3 x0 = ProjectOnPlane(s->GetTangentAt(this), N, true);
		V3 y0 = Cross(x0, N);
		V3 tangent_s = ProjectOnPlane(other->GetTangentAt(this), N, false);
		if (Mag(tangent_s) < 0.7f) {
			if (Mag(tangent_s) > projMax) {
				bestInPlane = other;
				projMax = Mag(tangent_s);
			}
			continue;
		}
		tangent_s = Normalized(tangent_s);
		real y_s = Dot(tangent_s, y0);
		real x_s = Dot(tangent_s, x0);
		if (nextSegment == nullptr) {
			nextSegment = other;
			continue;
		}
		V3 next_tangent = nextSegment->GetTangentAt(this);
		if (Dot(next_tangent, y0) >= 0.0f) {
			if (next) {
				if (y_s > 0.0f && Dot(next_tangent, x0) < x_s) {
					nextSegment = other;
				}
			} else {
				if (y_s <= 0.0f || Dot(next_tangent, x0) > x_s) {
					nextSegment = other;
				}
			}
		} else {
			if (next) {
				if (y_s >= 0.0f || Dot(next_tangent, x0) > x_s) {
					nextSegment = other;
				}
			} else {
				if (y_s < 0.0f && Dot(next_tangent, x0) < x_s) {
					nextSegment = other;
				}
			}
		}
	}
	if (nextSegment == nullptr) {
		nextSegment = bestInPlane;
	}
	return nextSegment;
}
void Node::UpdateNormal() {
	if (Neighbors.size() < 2) {
		Normal = V3();
		return;
	}
	std::vector<V3> tangents = GetNeighbors();
	V3 newNormal;
	if (tangents.size() > 2 || Mag(Cross(tangents[0], tangents[1])) > 0.1f) {
		real err = 0.0f;
		Plane bestPlane = FitPlaneVectors(Position, tangents, err);
		if (!bestPlane.valid) {
			std::vector<V3> pseudo;
			for (Segment *s : Neighbors) {
				Node *opp = s->GetOpposite(this);
				V3 oppositeEndpoint = opp != nullptr ? opp->Position : s->GetPointAt(Fmod(s->GetParam(this) + 0.5f, 1.0f));
				pseudo.push_back(Normalized(oppositeEndpoint - Position));
			}
			bestPlane = FitPlaneVectors(Position, pseudo, err);
		}
		IsSharp = bestPlane.valid && err > 0.5f;
		if (bestPlane.valid) {
			newNormal = bestPlane.n;
		}
	}
	Normal = newNormal;
}
void Node::AddSegment(Segment *s) {
	Neighbors.push_back(s);
	UpdateNormal();
	if (Mag(Normal) > 0.0f) {
		SortSegments();
	}
}
void Node::RemoveSegment(Segment *s) {
	std::list<Segment *>::iterator it = std::find(Neighbors.begin(), Neighbors.end(), s);
	if (it != Neighbors.end()) {
		Neighbors.erase(it);
	}
	UpdateNormal();
	if (Mag(Normal) > 0.0f) {
		SortSegments();
	}
}
bool Node::TryRemove() {
	if (Neighbors.size() == 2 && Neighbors.front()->Stroke == Neighbors.back()->Stroke) {
		Segment *s = Neighbors.front();
		bool canMend = s->Stroke->MendSegments(this, Neighbors.front(), Neighbors.back());
		if (canMend) {
			Neighbors.clear();
			return true;
		}
		return false;
	} else if (Neighbors.empty()) {
		return true;
	}
	return false;
}
void Node::SortSegments() {
	if (Neighbors.size() <= 2) {
		return;
	}
	std::list<Segment *> sorted;
	sorted.push_front(Neighbors.front());
	Neighbors.pop_front();
	for (Segment *s : Neighbors) {
		std::list<Segment *>::iterator neighbor = sorted.begin();
		V3 x0 = ProjectOnPlane((*neighbor)->GetTangentAt(this));
		V3 y0 = Cross(x0, Normal);
		V3 tangent_s = ProjectOnPlane(s->GetTangentAt(this));
		real y_s = Dot(tangent_s, y0);
		real x_s = Dot(tangent_s, x0);
		bool isAfterCurrent = true;
		while (std::next(neighbor) != sorted.end() && isAfterCurrent) {
			++neighbor;
			V3 tangent_neighbor = ProjectOnPlane((*neighbor)->GetTangentAt(this));
			real x_c = x_s;
			real y_c = y_s;
			if (Dot(tangent_neighbor, tangent_s) > 0.99f) {
				tangent_neighbor = Normalized(nn((*neighbor)->GetOpposite(this))->Position - Position);
				Node *opp = s->GetOpposite(this);
				V3 s_opp = opp != nullptr ? opp->Position : s->GetPointAt(Fmod(s->GetParam(this) + 0.5f, 1.0f));
				V3 t_corr = Normalized(s_opp - Position);
				x_c = Dot(t_corr, x0);
				y_c = Dot(t_corr, y0);
			}
			if (Dot(tangent_neighbor, y0) >= 0.0f) {
				if (y_c > 0.0f && Dot(tangent_neighbor, x0) < x_c) {
					isAfterCurrent = false;
				}
			} else {
				if (y_c >= 0.0f || Dot(tangent_neighbor, x0) > x_c) {
					isAfterCurrent = false;
				}
			}
		}
		if (!isAfterCurrent) {
			sorted.insert(neighbor, s);
		} else {
			sorted.push_back(s);
		}
	}
	Neighbors = sorted;
}

// CycleDetection.cs
bool ListContains(const std::list<HalfSegment> &cycle, const Segment *s) {
	for (const HalfSegment &hs : cycle) {
		if (hs.GetSegmentID() == s->ID) {
			return true;
		}
	}
	return false;
}

V3 TransportAcrossNode(const Node *node, const Segment *prevSegment, const Segment *nextSegment, V3 normal) {
	V3 prev_tangent = -prevSegment->GetTangentAt(node);
	V3 tangent = nextSegment->GetTangentAt(node);
	V3 axis = Cross(prev_tangent, tangent);
	if (Mag(axis) > kEpsilon) {
		axis = Normalized(axis);
		real dot = Dot(prev_tangent, tangent);
		real theta = Acos(Clamp(dot, -1.0f, 1.0f));
		return RotateAngleAxis(theta * kRad2Deg, axis, normal);
	}
	return normal;
}

void ShouldReverse(bool &reversed, V3 currentNormal, Node *nextNode, Segment *segment) {
	V3 transported = segment->Transport(currentNormal, nextNode);
	real endpointsAngle = Dot(transported, nextNode->Normal);
	if (Fabs(endpointsAngle) < 0.5f) {
		Segment *nextAtNode = nn(nextNode->GetNext(segment));
		Segment *prevAtNode = nn(nextNode->GetPrevious(segment));
		V3 projNext = nextAtNode->ProjectInPlane(nextNode, transported);
		V3 projPrev = prevAtNode->ProjectInPlane(nextNode, transported);
		V3 x0 = segment->ProjectInPlane(nextNode, transported);
		V3 y0 = Cross(x0, transported);
		real twoPi = 2.0f * kPi;
		real thetaNext = Fmod(real(std::atan2(to_double(Dot(projNext, y0)), to_double(Dot(projNext, x0)))) + twoPi, twoPi);
		real thetaPrev = Fmod(real(std::atan2(to_double(Dot(projPrev, y0)), to_double(Dot(projPrev, x0)))) + twoPi, twoPi);
		if (thetaNext > thetaPrev) {
			reversed = !reversed;
		}
	} else if (endpointsAngle < 0.0f) {
		reversed = !reversed;
	}
}

bool DetectCycle(Graph *g, Segment *startSegment, Node *startNode, std::list<HalfSegment> &cycle, bool reversed) {
	Node *currentNode = startNode;
	Segment *currentSegment = startSegment;
	Node *nextNode = nullptr;
	Segment *nextSegment = nullptr;
	Node *oppositeNode = startSegment->GetOpposite(startNode);
	if (oppositeNode == startNode) {
		HalfSegment hs;
		hs.segment = currentSegment;
		hs.IsReversed = false;
		cycle.push_back(hs);
		return true;
	}
	if (currentNode->IncidentCount() < 2 || nn(oppositeNode)->IncidentCount() < 2) {
		return false;
	}
	V3 currentNormal;
	while (currentNode->IncidentCount() > 1) {
		if (ListContains(cycle, currentSegment)) {
			break;
		}
		if (g->ExistingCyclesCount(currentSegment) >= 2) {
			break;
		}
		if (currentNode->IncidentCount() > 2) {
			V3 transportedCurrentNormal = currentSegment->Transport(currentNormal, currentNode);
			if (currentNode->IsSharp && Mag(currentNormal) > 0.9f) {
				nextSegment = currentNode->GetInPlane(currentSegment, transportedCurrentNormal, reversed);
			} else {
				nextSegment = reversed ? currentNode->GetNext(currentSegment) : currentNode->GetPrevious(currentSegment);
			}
			nextNode = nn(nextSegment)->GetOpposite(currentNode);
			V3 nodeNormal;
			if (currentNode->IsSharp) {
				if (Mag(transportedCurrentNormal) < 0.1f) {
					nodeNormal = Normalized(Cross(currentSegment->GetTangentAt(currentNode), nextSegment->GetTangentAt(currentNode)));
					if (reversed) {
						nodeNormal = -nodeNormal;
					}
				} else {
					nodeNormal = TransportAcrossNode(currentNode, currentSegment, nextSegment, transportedCurrentNormal);
				}
			} else {
				nodeNormal = currentNode->Normal;
			}
			currentNormal = nodeNormal;
			if (nn(nextNode)->IncidentCount() > 2 && !nextNode->IsSharp) {
				ShouldReverse(reversed, currentNormal, nextNode, nextSegment);
			}
		} else {
			nextSegment = nn(currentNode->GetNext(currentSegment));
			nextNode = nextSegment->GetOpposite(currentNode);
			if (Mag(currentNormal) > 0.9f) {
				currentNormal = currentSegment->Transport(currentNormal, currentNode);
				currentNormal = TransportAcrossNode(currentNode, currentSegment, nextSegment, currentNormal);
			} else {
				V3 nTrivial = Cross(currentSegment->GetTangentAt(currentNode), nextSegment->GetTangentAt(currentNode));
				if (Mag(nTrivial) > 0.5f) {
					currentNormal = Normalized(nTrivial);
				}
			}
			if (Mag(currentNormal) > 0.9f) {
				if (nn(nextNode)->IncidentCount() > 2 && !nextNode->IsSharp) {
					ShouldReverse(reversed, currentNormal, nextNode, nextSegment);
				}
			}
		}
		HalfSegment hs;
		hs.segment = currentSegment;
		hs.IsReversed = currentSegment->IsInReverse(currentNode);
		cycle.push_back(hs);
		currentSegment = nextSegment;
		currentNode = nn(nextNode);
	}
	Node *finalNode = currentSegment->GetOpposite(currentNode);
	return finalNode == oppositeNode && currentSegment == startSegment && cycle.size() > 1;
}

bool DetectCycleAt(Graph *g, V3 inputPos, bool lookAtNonManifold, std::list<HalfSegment> &cycle) {
	Segment *closest = g->FindClosestSegment(inputPos, lookAtNonManifold);
	if (closest == nullptr) {
		return false;
	}
	if (closest->GetStartNode() == closest->GetEndNode()) {
		HalfSegment hs;
		hs.segment = closest;
		hs.IsReversed = false;
		cycle.push_back(hs);
		return true;
	}
	Node *startNode = closest->GetStartNode();
	Node *currentNode = startNode;
	Segment *currentSegment = closest;
	int counter = 0;
	while (counter <= 10) {
		if (ListContains(cycle, currentSegment)) {
			break;
		}
		HalfSegment hs;
		hs.segment = currentSegment;
		hs.IsReversed = !currentSegment->IsInReverse(currentNode);
		cycle.push_back(hs);
		currentNode = currentSegment->GetOpposite(currentNode);
		Segment *nextSegment = g->FindClosestAmongNeighbors(inputPos, currentNode, currentSegment, lookAtNonManifold);
		if (nextSegment == nullptr) {
			break;
		}
		if (Dot(currentSegment->GetTangentAt(currentNode), nextSegment->GetTangentAt(currentNode)) < 0.8f) {
			counter++;
		}
		currentSegment = nextSegment;
	}
	return currentNode == startNode && currentSegment == closest && cycle.size() > 1;
}

bool Graph::TryFindCycleAt(V3 pos, bool lookAtNonManifold) {
	if (!surfacing) {
		return false;
	}
	std::list<HalfSegment> segs;
	if (DetectCycleAt(this, pos, lookAtNonManifold, segs)) {
		cycle_pool.push_back(std::unique_ptr<Cycle>(new Cycle(true, segs)));
		return TryAddCycle(cycle_pool.back().get());
	}
	return false;
}

void Graph::TryFindCycle(Segment *startSegment, Node *startNode, bool reversed) {
	std::list<HalfSegment> segs;
	if (DetectCycle(this, startSegment, startNode, segs, reversed)) {
		cycle_pool.push_back(std::unique_ptr<Cycle>(new Cycle(false, segs)));
		TryAddCycle(cycle_pool.back().get());
	}
}

void Graph::TryFindAllCycles() {
	if (!surfacing) {
		return;
	}
	CsHashSet<Segment *> seeds(SegmentHash, SegmentEq);
	std::vector<Cycle *> toCheck = _cyclesToCheckCache.Items();
	std::vector<Segment *> updated = _updatedSegmentsCache.Items();
	for (Cycle *cycle : toCheck) {
		for (Segment *s : updated) {
			Node *endpointA = nn(s->GetStartNode());
			Segment *sA1 = endpointA->GetNext(s);
			Segment *sA2 = endpointA->GetPrevious(s);
			Node *endpointB = nn(s->GetEndNode());
			Segment *sB1 = endpointB->GetNext(s);
			Segment *sB2 = endpointB->GetPrevious(s);
			if (cycle->Contains(sA1, sA2)) {
				if (TryReachCycle(cycle, s, endpointB)) {
					seeds.Add(TryRemove(cycle));
					break;
				}
			} else if (cycle->Contains(sB1, sB2)) {
				if (TryReachCycle(cycle, s, endpointA)) {
					seeds.Add(TryRemove(cycle));
					break;
				}
			}
		}
	}
	for (Segment *segment : seeds.Items()) {
		TryFindCycle(segment);
	}
	_cyclesToCheckCache.Clear();
	for (Segment *segment : _updatedSegmentsCache.Items()) {
		TryFindCycle(segment);
	}
	_updatedSegmentsCache.Clear();
}

} // namespace

struct Replay::Impl {
	WorldPoint origin;
	Plane plane;
	Graph graph;
	std::vector<std::unique_ptr<FinalStroke>> pool;
	std::vector<FinalStroke *> strokes;
	std::map<int, FinalStroke *> mirrored;
	std::vector<LoggedPatch> pending;
	std::map<int, Cycle *> alive_patch;
	int fresh = 1000000;
	ReplayStats stats;

	V3 Local(const WorldPoint &p) const {
		return v3(real(p.x - origin.x), real(p.y - origin.y), real(p.z - origin.z));
	}
	FinalStroke *NewStroke(int id) {
		pool.push_back(std::unique_ptr<FinalStroke>(new FinalStroke(&graph)));
		pool.back()->ID = id;
		return pool.back().get();
	}
	FinalStroke *Find(int id) {
		for (FinalStroke *s : strokes) {
			if (s->ID == id) {
				return s;
			}
		}
		return nullptr;
	}
	static std::vector<int> Key(const std::vector<int> &ids) {
		std::vector<int> k = ids;
		std::sort(k.begin(), k.end());
		return k;
	}
	// Assigns logged patch ids to new cycles, as the session's surface manager did.
	void Sink(const std::vector<Cycle *> &to_add, const std::vector<Cycle *> &to_remove) {
		for (Cycle *c : to_remove) {
			alive_patch.erase(c->patchID);
		}
		for (Cycle *c : to_add) {
			std::vector<int> k = Key(c->StrokeIDs());
			int pid = -1;
			for (size_t i = 0; i < pending.size(); i++) {
				if (Key(pending[i].strokes) == k && pending[i].found_by_algo == !c->userCreated) {
					pid = pending[i].id;
					pending.erase(pending.begin() + i);
					break;
				}
			}
			if (pid < 0) {
				pid = fresh++;
			}
			c->patchID = pid;
			alive_patch[pid] = c;
		}
	}
	void GraphUpdate() {
		graph.TryFindAllCycles();
		std::vector<Cycle *> to_add;
		std::vector<Cycle *> to_remove;
		graph.Update(to_add, to_remove);
		Sink(to_add, to_remove);
	}
	// Float64 Bernstein samples, used only to shortlist strokes before the float32 projection.
	static std::vector<std::array<double, 3>> Dense(const Curve &c, int per) {
		std::vector<std::array<double, 3>> out;
		double step = 1.0 / (double)(per - 1);
		int nb = c.is_line ? 1 : (int)c.beziers.size();
		for (int b = 0; b < nb; b++) {
			for (int i = 0; i < per; i++) {
				double u = i == per - 1 ? 1.0 : (double)i * step;
				double w = 1.0 - u;
				std::array<double, 3> r;
				for (int k = 0; k < 3; k++) {
					if (c.is_line) {
						double a = to_double(k == 0 ? c.A.x : (k == 1 ? c.A.y : c.A.z));
						double e = to_double(k == 0 ? c.B.x : (k == 1 ? c.B.y : c.B.z));
						r[k] = w * a + u * e;
					} else {
						const V3 *P = &c.ctrl[3 * b];
						double p0 = to_double(k == 0 ? P[0].x : (k == 1 ? P[0].y : P[0].z));
						double p1 = to_double(k == 0 ? P[1].x : (k == 1 ? P[1].y : P[1].z));
						double p2 = to_double(k == 0 ? P[2].x : (k == 1 ? P[2].y : P[2].z));
						double p3 = to_double(k == 0 ? P[3].x : (k == 1 ? P[3].y : P[3].z));
						r[k] = (w * w * w) * p0 + 3.0 * (w * w) * u * p1 + 3.0 * w * u * u * p2 + (u * u * u) * p3;
					}
				}
				out.push_back(r);
			}
		}
		return out;
	}
	FinalStroke *ResolveStroke(V3 pos, PointOnCurve &r_old, real &r_dist) {
		std::vector<std::pair<double, int>> best;
		for (FinalStroke *st : strokes) {
			double m = std::numeric_limits<double>::infinity();
			for (const std::array<double, 3> &q : Dense(*st->curve, 96)) {
				double dx = q[0] - to_double(pos.x);
				double dy = q[1] - to_double(pos.y);
				double dz = q[2] - to_double(pos.z);
				m = std::min(m, std::sqrt(dx * dx + dy * dy + dz * dz));
			}
			best.push_back(std::make_pair(m, st->ID));
		}
		if (best.empty()) {
			return nullptr;
		}
		std::sort(best.begin(), best.end());
		FinalStroke *win = nullptr;
		real win_d = 0.0f;
		for (const std::pair<double, int> &b : best) {
			if (!(b.first < best[0].first + 2e-3)) {
				continue;
			}
			FinalStroke *st = Find(b.second);
			PointOnCurve poc = st->curve->Project(pos);
			real d = Distance(poc.position, pos);
			if (win == nullptr || d < win_d || (d == win_d && st->ID < win->ID)) {
				win = st;
				win_d = d;
				r_old = poc;
			}
		}
		r_dist = win_d;
		return win;
	}
};

Replay::Replay(WorldPoint p_canvas_origin, WorldPoint p_mirror_point, WorldPoint p_mirror_normal) :
		impl(new Impl) {
	impl->origin = p_canvas_origin;
	impl->plane.p0 = impl->Local(p_mirror_point);
	impl->plane.n = v3(real(p_mirror_normal.x), real(p_mirror_normal.y), real(p_mirror_normal.z));
	impl->plane.valid = true;
}

Replay::~Replay() {
	delete impl;
}

struct Intersection {
	FinalStroke *old_stroke;
	PointOnCurve old_data;
	PointOnCurve new_data;
};

void Replay::SetSurfacing(bool p_surfacing) {
	impl->graph.surfacing = p_surfacing;
}

void Replay::SetPendingPatches(const std::vector<LoggedPatch> &p_patches) {
	impl->pending = p_patches;
}

void Replay::AddStroke(int p_id, const std::vector<WorldPoint> &p_ctrl_points, const std::vector<RecordedConstraint> &p_constraints, bool p_closed_loop, bool p_mirroring, float p_canvas_scale) {
	Impl &m = *impl;
	real sd = real(0.02f) / real(p_canvas_scale);
	real snap = sd;
	real merge = sd * 0.5f;
	real prox = sd * 2.0f;
	std::vector<V3> ctrl;
	for (const WorldPoint &p : p_ctrl_points) {
		ctrl.push_back(m.Local(p));
	}
	Curve new_curve(ctrl);
	bool on_mirror = true;
	for (const V3 &p : ctrl) {
		if (!(Fabs(Dot(m.plane.n, p - m.plane.p0)) < 1e-6f)) {
			on_mirror = false;
		}
	}
	// Constraints were applied at the new curve's anchors (a line takes its projection), sorted by t on a line.
	std::vector<std::pair<PointOnCurve, const RecordedConstraint *>> items;
	for (const RecordedConstraint &c : p_constraints) {
		V3 pos = m.Local(c.position);
		PointOnCurve nd;
		if (new_curve.is_line) {
			nd = new_curve.Project(pos);
			if (Distance(nd.position, pos) >= prox * 0.1f) {
				continue;
			}
		} else {
			int nb = (int)new_curve.beziers.size();
			int best_i = 0;
			real best_a = kInf;
			for (int i = 0; i <= nb; i++) {
				real d = Distance(new_curve.ctrl[i * 3], pos);
				if (d < best_a) {
					best_a = d;
					best_i = i;
				}
			}
			nd.t = best_i == nb ? real(1.0f) : (real(0.0f) + real(best_i)) / real(nb);
			nd.position = new_curve.ctrl[best_i * 3];
		}
		items.push_back(std::make_pair(nd, &c));
	}
	if (new_curve.is_line) {
		std::stable_sort(items.begin(), items.end(), [](const std::pair<PointOnCurve, const RecordedConstraint *> &a, const std::pair<PointOnCurve, const RecordedConstraint *> &b) { return a.first.t < b.first.t; });
	}
	// The recording omits which stroke a constraint hit; it lies on that stroke's curve.
	std::vector<Intersection> intersections;
	std::vector<PointOnCurve> seams;
	for (const std::pair<PointOnCurve, const RecordedConstraint *> &item : items) {
		const RecordedConstraint &c = *item.second;
		V3 pos = m.Local(c.position);
		if (c.is_intersection) {
			PointOnCurve old;
			real dist = 0.0f;
			FinalStroke *best = m.ResolveStroke(pos, old, dist);
			if (best == nullptr || dist > 1e-3f) {
				m.stats.unresolved_constraints++;
				continue;
			}
			Intersection in;
			in.old_stroke = best;
			in.old_data = old;
			in.new_data = item.first;
			intersections.push_back(in);
		} else if (std::fabs(to_double(Dot(m.plane.n, pos - m.plane.p0))) < 1e-5) {
			seams.push_back(item.first);
			m.stats.mirror_seam_constraints++;
		}
	}
	try {
		FinalStroke *fs = m.NewStroke(p_id);
		fs->SetCurve(ctrl, p_closed_loop);
		for (const Intersection &in : intersections) {
			Node *node = in.old_stroke->AddIntersectionOldStroke(in.old_data, snap);
			fs->AddIntersectionNewStroke(node, in.new_data, merge);
		}
		if (p_mirroring) {
			if (on_mirror) {
				m.mirrored[fs->ID] = fs;
				m.stats.on_mirror_strokes++;
			} else {
				std::vector<V3> mctrl;
				for (const V3 &p : ctrl) {
					mctrl.push_back(m.plane.Mirror(p));
				}
				FinalStroke *ms = m.NewStroke(p_id + 1);
				ms->SetCurve(mctrl, p_closed_loop);
				m.mirrored[fs->ID] = ms;
				m.mirrored[ms->ID] = fs;
				for (const Intersection &in : intersections) {
					std::map<int, FinalStroke *>::iterator it = m.mirrored.find(in.old_stroke->ID);
					if (it != m.mirrored.end()) {
						PointOnCurve mo = in.old_data;
						mo.position = m.plane.Mirror(mo.position);
						Node *node = it->second->AddIntersectionOldStroke(mo, snap);
						PointOnCurve mn = in.new_data;
						mn.position = m.plane.Mirror(mn.position);
						ms->AddIntersectionNewStroke(node, mn, merge);
					}
				}
				for (const PointOnCurve &seam : seams) {
					Node *node = fs->AddIntersectionOldStroke(seam, snap);
					ms->AddIntersectionNewStroke(node, seam, merge);
				}
				m.strokes.push_back(ms);
				m.GraphUpdate();
			}
		}
		m.strokes.push_back(fs);
		m.GraphUpdate();
	} catch (const CsException &) {
		m.stats.caught_exceptions++;
	}
}

void Replay::DeleteStroke(int p_id, bool p_mirroring) {
	Impl &m = *impl;
	FinalStroke *s = m.Find(p_id);
	if (s == nullptr) {
		return;
	}
	try {
		std::map<int, FinalStroke *>::iterator it = m.mirrored.find(s->ID);
		if (it != m.mirrored.end()) {
			FinalStroke *ms = it->second;
			m.mirrored.erase(s->ID);
			m.mirrored.erase(ms->ID);
			if (p_mirroring && ms->ID != s->ID) {
				m.strokes.erase(std::remove(m.strokes.begin(), m.strokes.end(), ms), m.strokes.end());
				ms->Destroy();
			}
		}
		m.strokes.erase(std::remove(m.strokes.begin(), m.strokes.end(), s), m.strokes.end());
		s->Destroy();
		m.GraphUpdate();
	} catch (const CsException &) {
		m.stats.caught_exceptions++;
	}
}

// DrawingCanvas.TryAddPatchAt: the tap, in canvas space, then its mirror image with the same manifold choice.
bool Replay::AddUserPatches(const std::vector<LoggedPatch> &p_group, WorldPoint p_tap, bool p_mirroring) {
	Impl &m = *impl;
	m.pending = p_group;
	bool ok = false;
	try {
		V3 pos = m.Local(p_tap);
		bool lookAtNonManifold = false;
		ok = m.graph.TryFindCycleAt(pos, lookAtNonManifold);
		if (!ok) {
			lookAtNonManifold = true;
			ok = m.graph.TryFindCycleAt(pos, lookAtNonManifold);
		}
		if (ok) {
			if (p_mirroring) {
				m.graph.TryFindCycleAt(m.plane.Mirror(pos), lookAtNonManifold);
			}
			m.GraphUpdate();
		} else {
			m.stats.tap_misses++;
		}
	} catch (const CsException &) {
		m.stats.caught_exceptions++;
	}
	return ok;
}

bool Replay::DeletePatch(int p_id) {
	Impl &m = *impl;
	std::map<int, Cycle *>::iterator it = m.alive_patch.find(p_id);
	if (it == m.alive_patch.end()) {
		m.stats.missing_patch_deletes++;
		return false;
	}
	m.graph.ManualDeletePatch(p_id);
	m.alive_patch.erase(p_id);
	return true;
}

std::vector<std::vector<int>> Replay::Cycles(bool p_include_user) const {
	std::vector<std::vector<int>> out;
	for (Cycle *c : impl->graph._cycles.Items()) {
		if (c->userCreated && !p_include_user) {
			continue;
		}
		std::vector<int> ids = c->StrokeIDs();
		std::sort(ids.begin(), ids.end());
		ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
		out.push_back(ids);
	}
	std::sort(out.begin(), out.end());
	return out;
}

std::vector<CycleBoundary> Replay::Boundaries(bool p_include_user) const {
	std::vector<CycleBoundary> out;
	for (Cycle *c : impl->graph._cycles.Items()) {
		if (c->userCreated && !p_include_user) {
			continue;
		}
		CycleBoundary b;
		b.user_created = c->userCreated;
		b.strokes = c->StrokeIDs();
		std::sort(b.strokes.begin(), b.strokes.end());
		b.strokes.erase(std::unique(b.strokes.begin(), b.strokes.end()), b.strokes.end());
		// The half-segment list is not a walk after repairs; walk the segments by their nodes.
		std::vector<const Segment *> segs;
		for (const HalfSegment &hs : c->HalfSegments) {
			segs.push_back(hs.segment);
		}
		std::vector<bool> used(segs.size(), false);
		const Node *at = segs.empty() ? nullptr : segs[0]->GetStartNode();
		for (size_t step = 0; step < segs.size(); step++) {
			size_t pick = segs.size();
			for (size_t i = 0; i < segs.size() && pick == segs.size(); i++) {
				if (!used[i] && (segs[i]->GetStartNode() == at || segs[i]->GetEndNode() == at)) {
					pick = i;
				}
			}
			if (pick == segs.size()) {
				b.broken_walk = true;
				for (size_t i = 0; i < segs.size() && pick == segs.size(); i++) {
					if (!used[i]) {
						pick = i;
					}
				}
			}
			used[pick] = true;
			const Segment *s = segs[pick];
			bool forward = s->GetStartNode() == at || pick == segs.size();
			BoundarySpan sp;
			sp.stroke = s->Stroke->ID;
			double t0 = to_double(s->GetStartParam());
			double t1 = to_double(s->GetEndParam());
			sp.t_from = forward ? t0 : t1;
			sp.t_to = forward ? t1 : t0;
			b.spans.push_back(sp);
			at = forward ? s->GetEndNode() : s->GetStartNode();
		}
		out.push_back(b);
	}
	std::sort(out.begin(), out.end(), [](const CycleBoundary &a, const CycleBoundary &b) { return a.strokes < b.strokes; });
	return out;
}

ReplayStats Replay::Stats() const {
	return impl->stats;
}

} // namespace cassie_graph_port
