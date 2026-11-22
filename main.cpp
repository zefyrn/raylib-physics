#include "renderer.h"
#include "collision.h"
#include "world.h"
#include <algorithm>

float screenWidth = 1800;
float screenHeight = 1200;
int main(void)
{
	renderer rend1;
	rend1.init(screenWidth, screenHeight, "Physics Engine");
	world scene;
	
	scene.createBody({ screenWidth / 2, screenHeight - 200 }, { 0,0 }, 0, 1000, 100, BLUE);
	scene.createBody({ screenWidth / 2, screenHeight-1200}, { 0,0 }, 0, 1000, 100, BLUE);
	scene.createBody({ screenWidth-1800, screenHeight/2 }, { 0,0 }, 0, 100, 1000, BLUE);
	scene.createBody({ screenWidth , screenHeight / 2 }, { 0,0 }, 0, 100, 1000, BLUE);
	while (!WindowShouldClose())
	{
		if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
			Vector2 delta = GetMouseDelta();
			float deltatime = GetFrameTime();
			Vector2 pos = GetMousePosition();
			scene.createBody({ pos.x, pos.y }, { std::clamp(delta.x / deltatime,-100.0f,100.0f),std::clamp(delta.y / deltatime,-100.0f,100.0f) }, 1, 50, 50, { unsigned char(GetRandomValue(0,255)),unsigned char(GetRandomValue(0,255)) ,unsigned char(GetRandomValue(0,255)) ,255 });
		}
		rend1.draw(scene.bodyList, BLACK);
		scene.step(0.1f, 8.0f);
	}

	rend1.shutdown();

	return 0;
}