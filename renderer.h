#pragma once
#include "raylib.h"
#include <vector>
#include "body.h"
struct renderer {
	void init(int width, int height, const char* title);
	void draw(const std::vector<body>& renderQueue, const Color& backgroundColor);
	void shutdown();
};