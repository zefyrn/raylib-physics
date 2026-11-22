#pragma once
#include "raylib.h"
#include <vector>

struct body {
	std::vector<Vector2> points;
	std::vector<Vector2> accuratePoints;
	float width = 0.0f;
	float height = 0.0f;

	Vector2 position = { 0.0f, 0.0f };
	Vector2 speed = { 0.0f, 0.0f };
	Vector2 centerOfMass = { 0.0f, 0.0f };
	Vector2 force = { 0.0f, 0.0f };
	float   invMass = 1.0f;
	float bodyArea = 0.0f;

	float angle = 0;
	float angVel = 0.0f;
	float torque = 0.0f;
	float invInertia = 0.0f;

	float restitution = 0.3;
	Color color = WHITE;
	
	std::vector<Vector2> updatedPoints();
	void calculateArea();
	void calculateMomentOfInertia();
};
struct triangle {
	Vector2 a, b, c;
};
std::vector<triangle> triangleFan(const std::vector<Vector2>& points);
Vector2 centroid(const body& a);