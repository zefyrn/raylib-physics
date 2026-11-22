#pragma once
#include "raylib.h"
Vector2 edgeVector(const Vector2& a, const Vector2& b);

Vector2 normalizeVector(const Vector2& vectorEdge);

float dotProduct(Vector2 p, Vector2 n);

float perpDot(Vector2 a, Vector2 b);
Vector2 perpScale(float s, Vector2 v);