#pragma once
#include "body.h"
#include <limits>
bool AABB(const body& a, const body& b);
struct collisionInfo {
	bool hit = false;
	Vector2 normal = { 0,0 };
	float depth = std::numeric_limits<float>::max();
	Vector2 contactPoint;
};
collisionInfo collide(const body& b1, const body& b2);
bool testAxes(const body& a, const body& B, collisionInfo& info);
