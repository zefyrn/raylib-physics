#include "renderer.h"

void renderer::init(int width, int height, const char* title) {
	InitWindow(width, height, title);
	SetTargetFPS(60);
}
void renderer::draw(const std::vector<body>& renderQueue, const Color& backgroundColor) {
	BeginDrawing();
	ClearBackground(backgroundColor);
	for (const body& object : renderQueue) {
		for (const triangle& triang : triangleFan(object.accuratePoints)) {
			DrawTriangle(triang.c, triang.b, triang.a, object.color);
		}

	}
	DrawFPS(10,10);
	EndDrawing();
}
void renderer::shutdown() {
	CloseWindow();
}