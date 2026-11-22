#include "math.h"
#include "cmath"
Vector2 edgeVector(const Vector2& a, const Vector2& b) {
	float dx = b.x - a.x;
	float dy = b.y - a.y;
	return { -dy,dx };
}

Vector2 normalizeVector(const Vector2& vectorEdge) {
	float length = std::sqrt(vectorEdge.x * vectorEdge.x + vectorEdge.y * vectorEdge.y);
	if (length == 0.0f) return { 0.0f,0.0f };
	return { vectorEdge.x / length,vectorEdge.y / length };
}

float dotProduct(Vector2 p, Vector2 n) {
	float projection = p.x * n.x + p.y * n.y;
	return projection;
}
float perpDot(Vector2 a, Vector2 b) {
	return  a.x * b.y - a.y * b.x ;
}
Vector2 perpScale(float s, Vector2 v) {
	return { -s * v.y, s * v.x };
}