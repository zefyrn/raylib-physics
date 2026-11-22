#include "body.h"
#include "math.h"
#include "raymath.h"
#include <cmath>
std::vector<triangle> triangleFan(const std::vector<Vector2>& points) {
	std::vector<triangle> triangles;
	size_t n = points.size();
	if (n < 3) {
		return triangles;
	}

	for (size_t i = 1; i + 1 < n; i++) {
		triangles.push_back({points[0],points[i],points[i+1]});
	}

	return triangles;
}
std::vector<Vector2> body::updatedPoints() {
	std::vector<Vector2> newPoints;
	for (const Vector2& p : points) {
		float new_x = p.x * cos(angle) - p.y * sin(angle);
		float new_y = p.x * sin(angle) + p.y * cos(angle);

		newPoints.push_back({ new_x + centerOfMass.x,new_y + centerOfMass.y });
	}
	return newPoints;
}
Vector2 centroid(const body& a) {
	float sum_x=0;
	float sum_y=0;
	for (const Vector2& point : a.points) {
		sum_x += point.x;
		sum_y += point.y;
	}
	return { sum_x / a.points.size(),sum_y / a.points.size() };
}
void body::calculateArea() {
	float areasSum = 0;
	for (const triangle& triang : triangleFan(points)) {
		Vector2 e1 = { triang.a.x - triang.b.x, triang.a.y - triang.b.y };
		Vector2 e2 = { triang.a.x - triang.c.x, triang.a.y - triang.c.y };
		float area= perpDot(e1, e2) / 2;
		areasSum += area;
	}
	bodyArea = areasSum;
}
void body::calculateMomentOfInertia() {
	if (invMass == 0) {
		invInertia = 0;
	}
	else {
		std::vector<triangle> triangles = triangleFan(points);
		float areasSum = 0;
		float sumInertia = 0;
		Vector2 massCenter = { 0, 0 };
		for (const triangle& triang : triangles) {
			Vector2 e1 = { triang.a.x - triang.b.x, triang.a.y - triang.b.y };
			Vector2 e2 = { triang.a.x - triang.c.x, triang.a.y - triang.c.y };
			float area = perpDot(e1, e2) / 2;
			areasSum += area;
			Vector2 triangleCenter = { (e1.x + e2.x) / 3, (e1.y + e2.y) / 3 };
			massCenter += triangleCenter * area;
			float inertiaOfTriangle = area / 6 * (dotProduct(e1, e1) + dotProduct(e1, e2) + dotProduct(e2, e2));
			sumInertia += inertiaOfTriangle;
		}
		Vector2 c = massCenter / areasSum;
		float k = sumInertia / areasSum - dotProduct(c, c);
		invInertia = invMass / k;
	}
	           
}