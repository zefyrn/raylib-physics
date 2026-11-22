#pragma once
#include "collision.h"
#include "body.h"
#include <unordered_map>

struct world {
	std::vector<body> bodyList;
	std::vector<int> bigBodyList;
	std::unordered_map<int64_t, std::vector<int>> grid;
	void createBody(Vector2 center, Vector2 speed, float mass, float WIDTH, float HEIGHT, Color color);
	void step(float dt, float gravity);
	void resolveCollisions(const collisionInfo& info, body& a, body& b);
	void buildGrid(float cellWidth,float cellHeight);
};