#include "collision.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include "math.h"

bool AABB(const body& a, const body& b) {
	Vector2 aIntervalX = { std::numeric_limits<float>::max(),-std::numeric_limits<float>::max() };
	Vector2 aIntervalY = {std::numeric_limits<float>::max(),-std::numeric_limits<float>::max() };
	Vector2 bIntervalX = {std::numeric_limits<float>::max() ,-std::numeric_limits<float>::max() };
	Vector2 bIntervalY = {std::numeric_limits<float>::max() ,-std::numeric_limits<float>::max() };

	for (const Vector2& point : a.accuratePoints) {
		aIntervalX = { std::min(aIntervalX.x,point.x),std::max(aIntervalX.y,point.x) };
		aIntervalY = { std::min(aIntervalY.x,point.y),std::max(aIntervalY.y,point.y) };
	}
	for (const Vector2& point : b.accuratePoints) {
		bIntervalX = { std::min(bIntervalX.x,point.x),std::max(bIntervalX.y,point.x) };
		bIntervalY = { std::min(bIntervalY.x,point.y),std::max(bIntervalY.y,point.y) };
	}
	return (aIntervalX.x <= bIntervalX.y && aIntervalX.y >= bIntervalX.x && aIntervalY.x <= bIntervalY.y && aIntervalY.y >= bIntervalY.x);

}
void project(const body& b, const Vector2& axis, float& min, float& max) {
	min = std::numeric_limits<float>::max();
	max = -std::numeric_limits<float>::max();
	for (const Vector2& point : b.accuratePoints) {
		float product = dotProduct(point, axis);
		min = std::min(min, product);
		max = std::max(max, product);
	}
}


bool testAxes(const body& a, const body& B, collisionInfo& info) {
	size_t n = a.accuratePoints.size();
	for (size_t i = 0; i < n; i++) {
		Vector2 axis = normalizeVector(edgeVector(a.accuratePoints[i], a.accuratePoints[(i + 1) % n]));
		float minA, maxA, minB, maxB;
		project(a, axis, minA, maxA);
		project(B, axis, minB, maxB);
		float overlap = std::min(maxA, maxB) - std::max(minA, minB);
		if (overlap < 0.0f) {
			return false;
		}
		if (overlap < info.depth) {
			info.depth = overlap;
			info.normal = axis;
		}
	}
	return true;
}
Vector2 getSupport(const body& s, const Vector2& dir) {
	float best = -std::numeric_limits<float>::max();
	Vector2 out = s.accuratePoints[0];
	for (const auto& p : s.accuratePoints) {
		float d = dotProduct(p, dir);
		if (d > best) { best = d; out = p; }
	}
	return out;
}

Vector2 findContactPoint(const body& a, const body& b, const Vector2& normal) {
	auto inside = [](const Vector2& p, const std::vector<Vector2>& poly) {
		for (size_t i = 0; i < poly.size(); i++) {
			const Vector2& v0 = poly[i];
			const Vector2& v1 = poly[(i + 1) % poly.size()];
			if (perpDot({ v1.x - v0.x, v1.y - v0.y }, { p.x - v0.x, p.y - v0.y }) < 0.0f) return false;
		}
		return true;
		};

	Vector2 sum = { 0, 0 };
	int n = 0;
	for (const Vector2& p : a.accuratePoints)
		if (inside(p, b.accuratePoints)) { sum.x += p.x; sum.y += p.y; n++; }
	for (const Vector2& p : b.accuratePoints)
		if (inside(p, a.accuratePoints)) { sum.x += p.x; sum.y += p.y; n++; }

	if (n > 0) return { sum.x / n, sum.y / n };

	Vector2 pa = getSupport(a, normal);
	Vector2 pb = getSupport(b, { -normal.x, -normal.y });
	return { (pa.x + pb.x) * 0.5f, (pa.y + pb.y) * 0.5f };
}
collisionInfo collide(const body& b1, const body& b2) {
	collisionInfo info;
	if (!AABB(b1, b2)) {
		return {};
	}
	if (!testAxes(b1, b2, info) || !testAxes(b2, b1, info)) {
		return {};
	}
	info.hit = true;
	Vector2 c1 = centroid(b1);
	Vector2 c2 = centroid(b2);
	Vector2 d = { b2.centerOfMass.x - b1.centerOfMass.x,
			  b2.centerOfMass.y - b1.centerOfMass.y };
	if (dotProduct(d, info.normal) < 0.0f) {
		info.normal = { -info.normal.x, -info.normal.y };
	}
	info.contactPoint = findContactPoint(b1, b2, info.normal);
	return info;
}

