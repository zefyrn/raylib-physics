#include "world.h"
#include "math.h"
#include <algorithm>
#include "raymath.h"

void world::buildGrid(float cellWidth, float cellHeight) {
	grid.clear();
	bigBodyList.clear();
	for (int i = 0; i < (int)bodyList.size(); i++) {
		const body& b = bodyList[i];
		if (b.bodyArea > cellWidth * cellHeight) {
			bigBodyList.push_back(i);
		}
		else {
			int gx = b.centerOfMass.x / cellWidth;
			int gy = b.centerOfMass.y / cellHeight;
			int64_t key = (static_cast<int64_t>(gx) << 32) | static_cast<uint32_t>(gy);
			grid[key].push_back(i);
		}
	}

}
void world::resolveCollisions(const collisionInfo& info, body& a, body& b) {
	float total = a.invMass + b.invMass;
	if (total == 0.0f) return;

	float ca = info.depth * a.invMass / total;
	float cb = info.depth * b.invMass / total;
	a.centerOfMass.x -= ca * info.normal.x;
	a.centerOfMass.y -= ca * info.normal.y;
	b.centerOfMass.x += cb * info.normal.x;
	b.centerOfMass.y += cb * info.normal.y;

	Vector2 rA = { info.contactPoint.x - a.centerOfMass.x,info.contactPoint.y - a.centerOfMass.y };
	Vector2 rB = { info.contactPoint.x - b.centerOfMass.x,info.contactPoint.y - b.centerOfMass.y };

	Vector2 vAp = { a.speed.x + perpScale(a.angVel,rA).x,a.speed.y + perpScale(a.angVel,rA).y };
	Vector2 vBp = { b.speed.x + perpScale(b.angVel,rB).x,b.speed.y + perpScale(b.angVel,rB).y };
	Vector2 rel = { vBp.x - vAp.x, vBp.y - vAp.y };
	float vn = dotProduct(rel, info.normal);
	if (vn < 0.0f) {
		Vector2 friction_impulse;
		float e = std::min(a.restitution, b.restitution);
		float raN = perpDot(rA, info.normal);   // cross(rA, n)
		float rbN = perpDot(rB, info.normal);
		float denom = a.invMass + b.invMass
			+ raN * raN * a.invInertia
			+ rbN * rbN * b.invInertia;

		float j = -(1.0f + e) * vn / denom;


		Vector2 impulse = { j * info.normal.x, j * info.normal.y };
		a.speed.x -= j * a.invMass * info.normal.x;
		a.speed.y -= j * a.invMass * info.normal.y;
		b.speed.x += j * b.invMass * info.normal.x;
		b.speed.y += j * b.invMass * info.normal.y;
		a.angVel -= perpDot(rA, impulse) * a.invInertia;
		b.angVel += perpDot(rB, impulse) * b.invInertia;

		Vector2 rv = (b.speed + perpScale(b.angVel, rB)) - (a.speed + perpScale(a.angVel, rA));
		Vector2 t = rv - info.normal * dotProduct(rv, info.normal);
		float tLen = sqrt(t.x * t.x + t.y * t.y);
		if (tLen > 1e-6f) {
			t.x /= tLen;
			t.y /= tLen;
			float raT = perpDot(rA, t);
			float rbT = perpDot(rB, t);
			float denomT = a.invMass + b.invMass + raT * raT * a.invInertia + rbT * rbT * b.invInertia;
			float jt = -dotProduct(rv, t) / denomT;
			float mu_s = 0.3f;
			float mu_d = 0.2f;
			Vector2 frictionImpulse;
			if (fabs(jt) <= j * mu_s) {
				frictionImpulse = { jt * t.x, jt * t.y };
			}
			else {
				frictionImpulse = { -j * mu_d * t.x, -j * mu_d * t.y };
			}
			a.speed.x -= frictionImpulse.x * a.invMass;
			a.speed.y -= frictionImpulse.y * a.invMass;
			b.speed.x += frictionImpulse.x * b.invMass;
			b.speed.y += frictionImpulse.y * b.invMass;
			a.angVel -= perpDot(rA, frictionImpulse) * a.invInertia;
			b.angVel += perpDot(rB, frictionImpulse) * b.invInertia;
		}
	}

	a.accuratePoints = a.updatedPoints();
	b.accuratePoints = b.updatedPoints();
}
void world::createBody(Vector2 center, Vector2 speed, float mass, float WIDTH, float HEIGHT, Color color) {
	std::vector<Vector2> points;
	points.push_back({ -WIDTH,-HEIGHT });
	points.push_back({ +WIDTH,-HEIGHT });
	points.push_back({ +WIDTH,+HEIGHT });
	points.push_back({ -WIDTH,+HEIGHT });
	body b;
	b.speed = speed;
	b.invMass = mass;
	b.width = WIDTH;
	b.height = HEIGHT;
	b.invMass = mass;
	b.points = points;
	b.calculateArea();
	b.calculateMomentOfInertia();
	b.centerOfMass = center;
	b.accuratePoints = b.updatedPoints();
	b.color = color;
	bodyList.push_back(b);
}
void world::step(float dt, float gravity) {
	for (body& b : bodyList) {
		if (b.invMass != 0.0f) {
			Vector2 acc = { b.force.x * b.invMass,b.force.y * b.invMass + gravity };
			b.speed.x += acc.x * dt;
			b.speed.y += acc.y * dt;



			Vector2 d = { b.speed.x * dt, b.speed.y * dt };
			b.centerOfMass.x += d.x;
			b.centerOfMass.y += d.y;
			b.angVel += b.torque * b.invInertia * dt;
			b.angle += b.angVel * dt;

			b.accuratePoints = b.updatedPoints();

		}
		b.force = { 0,0 };
		b.torque = 0.0f;
	}
	buildGrid(200, 200);
	for (int k = 0; k < 32; k++) {
		

		for (size_t i = 0; i < bodyList.size(); i++) {
			for (int ox : {-1, 0, 1}) {
				for (int oy : {-1, 0, 1}) {
					int gx = bodyList[i].centerOfMass.x / 200;
					int gy = bodyList[i].centerOfMass.y / 200;
					auto it = grid.find((static_cast<int64_t>(gx + ox) << 32) | static_cast<uint32_t>(gy + oy));
					if (it == grid.end()) continue;
					for (int j : it->second) {
						if (j <= (int)i) continue;
						collisionInfo info = collide(bodyList[i], bodyList[j]);
						if (info.hit) {

							resolveCollisions(info, bodyList[i], bodyList[j]);
						}
					}
				}
			}
			for (int j : bigBodyList) {
				if (j == (int)i) continue;
				if (bodyList[i].bodyArea > 200 * 200 && j < (int)i) continue;
				collisionInfo info = collide(bodyList[i], bodyList[j]);
				if (info.hit) resolveCollisions(info, bodyList[i], bodyList[j]);
			}
		}
	}

}