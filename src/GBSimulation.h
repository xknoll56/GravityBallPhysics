#include "GBInclude.h"


#include <typeinfo>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <unordered_map>

struct GBTerrain
{
	float minHeight = FLT_MAX;
	float maxHeight = -FLT_MAX;
	int cellsX;
	int cellsY;
	int cellsZ;
	std::vector<std::vector<GBTriangle*>> triangles;
	GBGrid* pGrid;
	float spacing = 1.0f;
	

	GBTerrain()
	{

	}
};

struct GBSimulation
{
	//GBGrid grid;
	GBGridMap gridMap;
	std::vector<GBCollider*> colliders;
	std::vector<std::unique_ptr<GBSphereCollider>> sphereColliders;
	std::vector<std::unique_ptr<GBBoxCollider>> boxColliders;
	std::vector<std::unique_ptr<GBCapsuleCollider>> capsuleColliders;
	std::vector<std::unique_ptr<GBBody>> rigidBodies;
	std::vector<std::unique_ptr<GBCloth>> cloths;
	std::vector< std::unique_ptr<GBTriangle>> triangles;
	std::vector< std::unique_ptr<GBTerrain>> terrains;
	uint32_t idCount;
	std::unordered_map<uint32_t, std::vector<std::function<void(const GBManifold& manifold, GBBody* pOther)>>> enterListeners;
	std::unordered_map<uint32_t, std::vector<std::function<void(const GBManifold& manifold, GBBody* pOther)>>> stayListeners;
	std::unordered_map<uint32_t, std::vector<std::function<void(GBBody* pOther)>>> exitListeners;

	//: gridMap(GBGridMap(GBVector3(-50,-50, -25), 1.0f, 100, 100, 50, 1, 1, 1))
	GBSimulation()
		: gridMap()
	{
		idCount = 0;
	}

	void clearSimulation()
	{
		while (!rigidBodies.empty())
			deleteBody(rigidBodies.back().get());

		while (!terrains.empty())
			deleteTerrain(terrains.back().get());

		while (!triangles.empty())
			deleteTriangle(triangles.back().get());
	}

	~GBSimulation()
	{
		clearSimulation();
	}

	void dispatchEnterListeners(uint32_t id, const GBManifold& manifold, GBBody* pOther)
	{
		auto it = enterListeners.find(id);
		if (it == enterListeners.end()) return;

		for (auto& fn : it->second)
			fn(manifold, pOther);
	}

	void dispatchStayListeners(uint32_t id, const GBManifold& manifold, GBBody* pOther)
	{
		auto it = stayListeners.find(id);
		if (it == stayListeners.end()) return;

		for (auto& fn : it->second)
			fn(manifold, pOther);
	}

	void dispatchExitListeners(uint32_t id,  GBBody* pOther)
	{
		auto it = exitListeners.find(id);
		if (it == exitListeners.end()) return;

		for (auto& fn : it->second)
			fn(pOther);
	}

	void addEnterListener(uint32_t id, std::function<void(const GBManifold&, GBBody*)> fn)
	{
		enterListeners[id].push_back(std::move(fn));
	}

	void addStayListener(uint32_t id, std::function<void(const GBManifold&, GBBody*)> fn)
	{
		stayListeners[id].push_back(std::move(fn));
	}

	void addExitListener(uint32_t id, std::function<void(GBBody*)> fn)
	{
		exitListeners[id].push_back(std::move(fn));
	}


	static GBSimulation simulationWithSingleGrid(GBVector3 anchor = GBVector3(-50, -50, -25), float cellSize = 1.0f, int cellsX = 100, int cellsY = 100, int cellsZ = 50)
	{
		GBSimulation sim;
		sim.gridMap = GBGridMap(anchor, cellSize, cellsX, cellsY, cellsZ, 1, 1, 1);
	}

	uint32_t getId()
	{
		uint32_t id = idCount;
		idCount++;
		return id;
	}

	GBBody* createBody(float mass = 1.0f, bool isStatic = false)
	{
		rigidBodies.push_back(std::make_unique<GBBody>());
		GBBody* body = rigidBodies.back().get();
		body->id = getId();
		body->setMass(mass);
		body->isStatic = isStatic;
		return body;
	}


	GBTriangle* createTriangle(GBVector3 a, GBVector3 b, GBVector3 c, bool insertToGrid = true)
	{
		triangles.push_back(std::make_unique<GBTriangle>(a, b, c));
		GBTriangle* pTriangle = triangles.back().get();
		pTriangle->id = getId();
		GBStaticGeometry* pStatic = (GBStaticGeometry*)pTriangle;
		if (insertToGrid)
			gridMap.insertStaticGeometry(*pStatic);
		return pTriangle;
	}

	// trianglesArr is a 2D vector of triangles.  The inner vector contains 2 triangles in a strip each creating a
	// quad to the next spacing
	GBTerrain* createTerrain(const std::vector<std::vector<GBTriangle>>& trianglesArr, float spacing = 1.0f)
	{
		terrains.push_back(std::make_unique<GBTerrain>());
		GBTerrain* pTerrain = terrains.back().get();
		GBVector3 origin = { FLT_MAX, FLT_MAX, FLT_MAX };
		GBVector3 max = { 0,0,0 };
		pTerrain->triangles.reserve(trianglesArr.size());
		for (const std::vector<GBTriangle> tris : trianglesArr)
		{
			pTerrain->triangles.push_back(std::vector<GBTriangle*>());
			for (const GBTriangle tri : tris)
			{
				pTerrain->triangles.back().push_back(createTriangle(tri.vertices[0], tri.vertices[1], tri.vertices[2], false));
				for (int i = 0; i < 3; i++)
				{
					GBVector3 vert = tri.vertices[i];
					pTerrain->minHeight = GBMin(pTerrain->minHeight, vert.z);
					pTerrain->maxHeight = GBMax(pTerrain->maxHeight, vert.z);
					origin = GBMin(origin, vert);
					max = GBMax(max, vert);
				}
			}
		}

		GBVector3 sizes = max - origin;

		pTerrain->spacing = spacing;
		pTerrain->cellsX = sizes.x / spacing;
		pTerrain->cellsY = sizes.y / spacing;
		pTerrain->cellsZ = std::ceil((pTerrain->maxHeight - pTerrain->minHeight) / spacing) + 1;

		pTerrain->pGrid = new GBGrid(origin, spacing, pTerrain->cellsX, pTerrain->cellsY, pTerrain->cellsZ, getId(), GBGridType::TERRAIN);

		for (const std::vector<GBTriangle*> tris : pTerrain->triangles)
		{
			for (const GBTriangle* pTri : tris)
			{
				GBAABB triAABB = pTri->toAABB();
				triAABB.grow(-0.01f * spacing);
				triAABB.halfExtents = GBMax(triAABB.halfExtents, GBVector3{ 0.01f,0.01f,0.01f });
				pTerrain->pGrid->insertStaticGeometry(triAABB, *(GBStaticGeometry*)pTri);
			}
		}

		gridMap.insertTerrainGrid(*pTerrain->pGrid);
		return pTerrain;
	}



	GBTriangle* getTriangle(int index)
	{
		return (index < triangles.size()) ? triangles[index].get() : nullptr;
	}

	GBSphereCollider* getSphereCollider(int index)
	{
		return (index < sphereColliders.size()) ? sphereColliders[index].get() : nullptr;
	}

	GBBoxCollider* getBoxCollider(int index)
	{
		return (index < boxColliders.size()) ? boxColliders[index].get() : nullptr;
	}

	GBCapsuleCollider* getCapsuleCollider(int index)
	{
		return (index < capsuleColliders.size()) ? capsuleColliders[index].get() : nullptr;
	}

	GBBody* getBody(int index)
	{
		return (index < rigidBodies.size()) ? rigidBodies[index].get() : nullptr;
	}

	bool deleteCollider(GBCollider* pCollider)
	{
		if (!pCollider) return false;

		// remove from flat collider list
		colliders.erase(std::remove(colliders.begin(), colliders.end(), pCollider), colliders.end());


		switch (pCollider->type)
		{
		case ColliderType::Box:
		{
			auto it = std::find_if(boxColliders.begin(), boxColliders.end(),
				[&](const auto& up) { return up.get() == pCollider; });
			if (it == boxColliders.end()) return false;
			boxColliders.erase(it);
			return true;
		}
		case ColliderType::Sphere:
		{
			auto it = std::find_if(sphereColliders.begin(), sphereColliders.end(),
				[&](const auto& up) { return up.get() == pCollider; });
			if (it == sphereColliders.end()) return false;
			sphereColliders.erase(it);
			return true;
		}
		default:
			return false;
		}
	}

	bool deleteTriangle(GBTriangle* triangle)
	{
		bool successful = true;
		gridMap.removeStaticGeometry(*(GBStaticGeometry*)triangle);


		auto it = std::find_if(triangles.begin(), triangles.end(),
			[&](const auto& up) { return up.get() == triangle; });
		if (it == triangles.end())
			successful = false;
		triangles.erase(it);

		return successful;
	}

	bool deleteTerrain(GBTerrain* pTerrain)
	{
		bool success = true;
		for (int i = 0; i < pTerrain->triangles.size(); i++)
		{
			for (int j = 0; j < pTerrain->triangles[i].size(); j++)
			{
				success = deleteTriangle(pTerrain->triangles[i][j]);
			}
		}

		gridMap.removeTerrainGrid(*pTerrain->pGrid);

		auto it = std::find_if(terrains.begin(), terrains.end(),
			[&](const auto& up) { return up.get() == pTerrain; });
		if (it == terrains.end())
			success = false;
		terrains.erase(it);

		return success;
	}


	bool deleteBody(GBBody* pBody)
	{
		if (!pBody) return false;

		gridMap.removeBody(*pBody);

		enterListeners.erase(pBody->id);
		stayListeners.erase(pBody->id);
		exitListeners.erase(pBody->id);

		auto collidersCopy = pBody->colliders;
		for (GBCollider* c : collidersCopy)
			deleteCollider(c);

		auto it = std::find_if(rigidBodies.begin(), rigidBodies.end(),
			[&](const auto& up) { return up.get() == pBody; });

		if (it == rigidBodies.end()) return false;
		rigidBodies.erase(it);
		return true;
	}

	GBSphereCollider* attachSphereCollider(GBBody* pBody, float radius, GBTransform localTransform = GBTransform(), bool insertToGrid = true)
	{
		sphereColliders.push_back(std::make_unique<GBSphereCollider>());
		GBSphereCollider* col = sphereColliders.back().get();
		col->localTransform = localTransform;
		col->radius = radius;
		col->id = getId();
		col->pBody = pBody;
		colliders.push_back(col);
		pBody->colliders.push_back(col);
		pBody->updateColliders();
		if (insertToGrid)
			gridMap.insertBody(*pBody);
		recenterMass(pBody);
		return col;
	}

	GBCloth* createCloth(int width = 10, int height = 10, float spacing = 0.25f, float stiffness = 1000.0f, float radius = 0.1f)
	{
		cloths.push_back(std::make_unique<GBCloth>());
		GBCloth* cloth = cloths.back().get();
		cloth->height = height;
		cloth->width = width;
		cloth->spacing = spacing;
		cloth->particleRadius = radius;
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				cloth->particles.push_back(GBParticle(GBVector3(0, j * spacing, i * spacing)));
			}
		}

		for (int i = 0; i < height - 1; i++)
		{
			for (int j = 0; j < width; j++)
			{
				cloth->springs.push_back(GBSpring(cloth->index(j, i), cloth->index(j, i + 1), spacing, stiffness));
			}
		}

		for (int i = 0; i < width - 1; i++)
		{
			for (int j = 0; j < height; j++)
			{
				cloth->springs.push_back(GBSpring(cloth->index(i, j), cloth->index(i + 1, j), spacing, stiffness));
			}
		}

		return cloth;
	}

	GBBoxCollider* attachBoxCollider(GBBody* pBody, GBVector3 halfExtents, GBTransform localTransform = GBTransform(), bool insertToGrid = true)
	{
		boxColliders.push_back(std::make_unique<GBBoxCollider>());
		GBBoxCollider* col = boxColliders.back().get();
		col->localTransform = localTransform;
		col->halfExtents = halfExtents;
		col->id = getId();
		col->pBody = pBody;
		colliders.push_back(col);
		pBody->colliders.push_back(col);
		pBody->updateColliders();
		if (insertToGrid)
			gridMap.insertBody(*pBody);
		recenterMass(pBody);
		return col;
	}

	GBCapsuleCollider* attachCapsuleCollider(GBBody* pBody, float radius, float height, GBTransform localTransform = GBTransform(), bool insertToGrid = true)
	{
		// For simplicity, we can treat the capsule as a box for now
		//boxColliders.push_back(std::make_unique<GBBoxCollider>());
		capsuleColliders.push_back(std::make_unique<GBCapsuleCollider>());
		GBCapsuleCollider* col = capsuleColliders.back().get();
		col->localTransform = localTransform;
		col->radius = radius;
		col->height = height;
		col->id = getId();
		col->pBody = pBody;
		colliders.push_back(col);
		pBody->colliders.push_back(col);
		pBody->updateColliders();
		if (insertToGrid)
			gridMap.insertBody(*pBody);
		recenterMass(pBody);
		return col;
	}

	void recenterMass(GBBody* body)
	{
		GBVector3 com = GBVector3::zero();
		float totalVolume = 0.0f;

		for (auto* c : body->colliders)
		{
			float vol = c->volume();           // assume each collider has a volume() method
			com += c->localTransform.position * vol;
			totalVolume += vol;
		}

		if (totalVolume > 0.0f)
			com /= totalVolume;                // weighted average by volume

		// shift colliders so COM becomes origin
		for (auto* c : body->colliders)
			c->localTransform.position -= com;

		// move body so world-space stays the same
		body->transform.position +=
			body->transform.rotation.rotate(com);
	}

	void setBodyTransform(GBBody& body, GBTransform& transform)
	{
		body.transform = transform;
		body.updateColliders();
		gridMap.moveBody(body);
	}

	void setBodyPosition(GBBody& body, GBVector3 position)
	{
		body.transform.position = position;
		body.updateColliders();
		gridMap.moveBody(body);
	}

	int solverIterations = 6;   // 6–12 typical
	static constexpr float maxDeltaTime = 1.0f / 60.0f;
	float timeScale = 1.0f;


	void solveDynamicManifold(const GBManifold& m, GBBody& A, GBBody& B, float dt, bool canTreatAsStatic = true)
	{
		//if (A.isKinematic || B.isKinematic)
		//	return;

		const float restitution = 0.05f;
		const float percent = 0.3f;
		const float forceCap = 250.0f;
		const float slopeRequirement = 0.9f;
		const float staticManifoldThreshold = 0.05f;

		int count = m.numContacts;

		GBVector3 dv = B.velocity - A.velocity;
		float vRelN = GBDot(dv, m.normal);
		//if (!m.isEdge && GBDot(GBVector3::up(), m.normal) > 0.90f && m.numContacts > 1 && GBAbs(vRelN) < 1.0f)


		bool ignoreA = A.isStatic || A.isKinematic;
		bool ignoreB = B.isStatic || B.isKinematic;
		float aModifier = 1.0f;
		float bModifier = 1.0f;



		for (int i = 0; i < count; i++)
		{
			const GBContact& c = m.contacts[i];
			GBVector3 n = m.getFeatureBasedNormal(i);

			GBVector3 rA = c.position - A.transform.position;
			GBVector3 rB = c.position - B.transform.position;

			GBVector3 vA = A.velocity + GBCross(A.angularVelocity, rA);
			if (A.isKinematic)
			{
				vA = A.realVelocity(dt) + GBCross(A.realAngularVelocity(dt), rA);
				B.wakeIsland();
			}

			GBVector3 vB = B.velocity + GBCross(B.angularVelocity, rB);
			if (B.isKinematic)
			{
				vB = B.realVelocity(dt) + GBCross(B.realAngularVelocity(dt), rB);
				A.wakeIsland();
			}
			GBVector3 vRel = vB - vA;
			float vn = GBDot(vRel, n);

			
			if (vn < 0.0f)
				continue;

			if (vn > wakeThreshold)
			{
				if (A.isMovable())
					A.wakeIsland();
				if (B.isMovable())
					B.wakeIsland();
			}

			if (canTreatAsStatic && (!A.isKinematic || !B.isKinematic))
			{
				float upness = GBDot(GBVector3::up(), m.normal);
				const static float stackModifier = 1.0f;
				if (GBAbs(GBAbs(vn) < staticManifoldThreshold * stackModifier && upness > slopeRequirement))
				{
					if (bodyIsPureColliderType(*m.pIncident, ColliderType::Sphere))
					{
						solveStaticSphereManifold(m, *m.pIncident, dt);
						return;
					}

					//if (m.pIncident->awakeTimer > 0.25)
					{
						solveStaticManifold(m, *m.pIncident);
						return;
					}
				}
			}


			float raCn = GBDot(GBCross(rA, n), GBCross(rA, n));
			float rbCn = GBDot(GBCross(rB, n), GBCross(rB, n));

			float invMassSum =
				A.invMass + B.invMass +
				raCn * A.invInertia.dot(n) +
				rbCn * B.invInertia.dot(n);

			float j = -(1.0f + restitution) * vn;
			j /= invMassSum;
			j /= (float)count;
			j = GBClamp(j, -forceCap * (A.mass + B.mass) * dt, 0.0f);

			GBVector3 impulse = n * j;

			if (!ignoreA)
				A.velocity -= impulse * A.invMass * aModifier;
			if (!ignoreB)
				B.velocity += impulse * B.invMass * bModifier;

			if (A.isKinematic)
			{
				float slope = GBDot(GBVector3::up(), m.normal);
				float vnA = GBDot(A.velocity, m.normal);

				static const float minGroundVelocity = 0.0f;
				if (slope >= A.playerSlopeValue && vnA < minGroundVelocity)
				{
					// Grounded on a walkable surface
					// clamp the verticle velocity to  the minimum ground velocity.
					// If we clamp it to zero, running down slopes doesn't work
					A.velocity.z = minGroundVelocity;
				}
			}

			if (B.isKinematic)
			{
				float slope = GBDot(GBVector3::up(), m.normal);
				float vnB = GBDot(B.velocity, m.normal);

				static const float minGroundVelocity = 0.0f;
				if (slope >= B.playerSlopeValue && vnB < minGroundVelocity)
				{
					// Grounded on a walkable surface
					// clamp the verticle velocity to  the minimum ground velocity.
					// If we clamp it to zero, running down slopes doesn't work
					A.velocity.z = minGroundVelocity;
				}
			}

			if (!ignoreA)
				A.angularVelocity -= GBCross(rA, impulse) * A.invInertia * aModifier;
			if (!ignoreB)
				B.angularVelocity += GBCross(rB, impulse) * B.invInertia * bModifier;

			// --- FRICTION ---
			if (GBDot(n, GBVector3::up()) > 0.05f)
			{
				GBVector3 vRelT = vRel - n * GBDot(vRel, n);
				float tLen = vRelT.length();
				if (tLen > 0.0001f)
				{
					GBVector3 t = vRelT / tLen;

					float jt = GBDot(vRel, t);
					jt /= invMassSum;

					const float mu = 0.4f; // friction coefficient
					float maxFriction = j * mu;

					jt = GBClamp(jt, -maxFriction, maxFriction);

					GBVector3 fImpulse = t * jt;

					if (!ignoreA)
						A.velocity -= fImpulse * A.invMass;
					if (!ignoreB)
						B.velocity += fImpulse * B.invMass;

					if (!ignoreA)
						A.angularVelocity -= GBCross(rA, fImpulse) * A.invInertia;
					if (!ignoreB)
						B.angularVelocity += GBCross(rB, fImpulse) * B.invInertia;
				}
			}
		}
	}

	void solveStaticSphereManifold(const GBManifold& manifold, GBBody& body, float dt)
	{
		if (body.isKinematic)
			return;

		const float restitution = body.restitution;
		const static float restingThreshold = 1.0f;
		const float rollingThreshold = 1.0f; // threshold for rolling without slip
		const float cornerPushStrength = 0.05f;
		const float minBounceVel = 0.001f;

		const GBContact& c = manifold.contacts[0];

		float flatness = GBDot(c.normal, GBVector3::up());
		const static float minFlat = 0.75f;
		bool notRestable = false;
		if (flatness < minFlat)
			notRestable = true;

		GBVector3 n = c.normal;                     // contact normal
		GBVector3 r = c.position - body.transform.position; // contact relative position
		//GBVector3 vRel = body.velocity;
		GBVector3 vRel = body.velocity;
		//GBVector3 vRel = body.velocity + GBCross(body.angularVelocity, r); // contact velocity

		float vn = GBDot(vRel, n);

		// --- Collision impulse ---
		if (vn < -restingThreshold || notRestable) // only if approaching
		{
			// Compute normal impulse
			float jn = -(1.0f + restitution) * vn;

			// Guarantee minimum bounce velocity
			float minJn = minBounceVel;
			if (jn < minJn)
				jn = minJn;

			// Apply normal impulse
			body.velocity += n * (jn);
			body.angularVelocity += body.invInertia * GBCross(r, n * jn);
			return; // collision handled, skip rolling
		}




		// --- Rolling without slip ---
		if (fabs(vn) < rollingThreshold) // sphere essentially resting
		{
			GBVector3 vTangent = vRel - n * vn; // remove normal component
			float vTangentLen = vTangent.length();

			//if (vTangentLen > 1e-6f)
			{
				// enforce rolling: v + ω × r = 0 → ω = r × (-v) / |r|^2
				GBVector3 omegaDesired = GBCross(r, -vTangent) / (r.lengthSquared());

				// optionally blend for stability
				float blend = 1.0f; // 1.0 = fully enforce, <1 = smooth
				body.angularVelocity = body.angularVelocity * (1.0f - blend) + omegaDesired * blend;
			}

			float frictionCoeff = body.dynamicFriction;
			body.velocity -= vTangent * frictionCoeff * dt;
			body.frameManifold.hasGroundedManifold = true;
		}
	}

	void solveStaticManifold(
		const GBManifold& manifold,
		GBBody& body)
	{
		if (manifold.numContacts == 0 || body.isKinematic)
		{
			if (body.useGravity)
			{
				float slope = GBDot(GBVector3::up(), manifold.normal);
				float vn = GBDot(body.velocity, manifold.normal);

				static const float minGroundVelocity = 0.0f;
				if (slope >= body.playerSlopeValue && vn < minGroundVelocity)
				{
					// Grounded on a walkable surface
					// clamp the verticle velocity to  the minimum ground velocity.
					// If we clamp it to zero, running down slopes doesn't work
					body.velocity.z = minGroundVelocity;
				}
			}
			return;
		}

		const float restitution = body.restitution;

		for (int i = 0; i < manifold.numContacts; i++)
		{
			const GBContact& c = manifold.contacts[i];

			GBVector3 n = c.normal;
			GBVector3 r = c.position - body.transform.position;

			// Relative velocity at contact
			GBVector3 vRel = body.velocity + GBCross(body.angularVelocity, r);

			float vn = GBDot(vRel, n);
			if (vn >= 0.0f)
				continue;

			// --- Effective mass ---
			GBVector3 rn = GBCross(r, n);
			float invMassEff =
				body.invMass +
				GBDot(rn, body.invInertia * rn);

			if (invMassEff < 1e-8f)
				continue;

			// --- Normal impulse ---
			float jn = -(1.0f + restitution) * vn;
			jn /= invMassEff;

			// Clamp for stability (like box solver)
			jn = GBClamp(jn, 0.0f, 20.0f * body.mass);

			GBVector3 impulse = n * jn;

			body.velocity += impulse * body.invMass;
			body.angularVelocity += body.invInertia * GBCross(r, impulse);

			// -------- Friction --------
			vRel = body.velocity + GBCross(body.angularVelocity, r);

			GBVector3 t = vRel - n * GBDot(vRel, n);
			float tLen = t.length();

			if (tLen > 1e-6f)
			{
				t /= tLen;

				GBVector3 rt = GBCross(r, t);

				float invMassT =
					body.invMass +
					GBDot(rt, body.invInertia * rt);


				if (GBDot(n, GBVector3::up()) > 0.5f)
				{
					if (invMassT > 1e-8f)
					{
						float vt = GBDot(vRel, t);
						float jt = -vt / invMassT;

						float maxFriction = body.dynamicFriction * jn;
						jt = GBClamp(jt, -maxFriction, maxFriction);

						GBVector3 frictionImpulse = t * jt;

						body.velocity += frictionImpulse * body.invMass;
						body.angularVelocity += body.invInertia * GBCross(r, frictionImpulse);
					}
				}
			}
		}

	}

	bool overlapTest(GBCollider* colA, GBCollider* colB, GBSATCollisionData& outData, GBManifold& outManifold)
	{
		GBContact c;
		bool retValue = false;
		switch (colA->type)
		{
		case ColliderType::Box:
			switch (colB->type)
			{
			case ColliderType::Box:
				retValue = GBManifoldGeneration::GBCollisionBoxBoxSAT(*(GBBoxCollider*)colA, *(GBBoxCollider*)colB, outData);
				break;
			case ColliderType::Sphere:
				retValue = GBManifoldGeneration::GBManifoldSphereBox(*(GBSphereCollider*)colB, *(GBBoxCollider*)colA, outManifold);
				break;
			case ColliderType::Capsule:
				retValue = GBManifoldGeneration::GBManifoldCapsuleBox(*(GBCapsuleCollider*)colB, *(GBBoxCollider*)colA, outManifold);
				break;
			}
			break;

		case ColliderType::Sphere:
			switch (colB->type)
			{
			case ColliderType::Box:
				retValue = GBManifoldGeneration::GBManifoldSphereBox(*(GBSphereCollider*)colA, *(GBBoxCollider*)colB, outManifold);
				break;
			case ColliderType::Sphere:
				retValue = GBManifoldGeneration::GBContactSphereSphere(*(GBSphereCollider*)colA, *(GBSphereCollider*)colB, c);
				break;
			case ColliderType::Capsule:
				retValue = GBManifoldGeneration::GBManifoldCapsuleSphere(*(GBCapsuleCollider*)colB, *(GBSphereCollider*)colA, outManifold);
				break;
			}
			break;

		case ColliderType::Capsule:
			switch (colB->type)
			{
			case ColliderType::Box:
				retValue = GBManifoldGeneration::GBManifoldCapsuleBox(*(GBCapsuleCollider*)colA, *(GBBoxCollider*)colB, outManifold);
				break;
			case ColliderType::Sphere:
				retValue = GBManifoldGeneration::GBManifoldCapsuleSphere(*(GBCapsuleCollider*)colA, *(GBSphereCollider*)colB, outManifold);
				break;
			case ColliderType::Capsule:
				if (colB->pBody->isKinematic)
					retValue = GBManifoldGeneration::GBManifoldCapsuleCapsule(*(GBCapsuleCollider*)colA, *(GBCapsuleCollider*)colB, outManifold);
				else
					retValue = GBManifoldGeneration::GBManifoldCapsuleCapsule(*(GBCapsuleCollider*)colB, *(GBCapsuleCollider*)colA, outManifold);
				break;
			}
			break;
		}
		return retValue;
	}

	bool generateManifold(GBCollider* colA, GBCollider* colB, const GBSATCollisionData& data, GBManifold& outManifold)
	{
		GBContact c;
		switch (colA->type)
		{
		case ColliderType::Box:
			switch (colB->type)
			{
			case ColliderType::Box:   return GBManifoldGeneration::GBManifoldBoxBox(*(GBBoxCollider*)colA,
				*(GBBoxCollider*)colB, data, outManifold);
			case ColliderType::Sphere:return false;
			case ColliderType::Capsule: return false;
			}
			break;

		case ColliderType::Sphere:
			switch (colB->type)
			{
			case ColliderType::Box:   return false;
			case ColliderType::Sphere:return false;
			case ColliderType::Capsule: return false;
			}
			break;

		case ColliderType::Capsule:
			switch (colB->type)
			{
			case ColliderType::Box:   return false;
			case ColliderType::Sphere:return false;
			case ColliderType::Capsule: return false;
			}
			break;
		}

		return false;
	}


	bool generateCapsuleManifold(GBCapsuleCollider* pCapsule, GBCollider* pOther, GBManifold& outManifold)
	{
		switch (pOther->type)
		{
		case ColliderType::Sphere:
		{
			GBSphereCollider* pSphere = (GBSphereCollider*)pOther;
			GBManifoldGeneration::GBManifoldCapsuleSphere(*pCapsule, *pSphere, outManifold);
		}
			break;
		case ColliderType::Box:
		{
			GBBoxCollider* pBox = (GBBoxCollider*)pOther;
			GBManifoldGeneration::GBManifoldCapsuleBox(*pCapsule, *pBox, outManifold);
		}
			break;
		case ColliderType::Capsule:
		{
			GBCapsuleCollider* pOtherCapsule = (GBCapsuleCollider*)pOther;
			// We want kinematic/dynamic bodies to be incident....
			if (pCapsule->pBody->isDynamic() || pCapsule->pBody->isKinematic)
				 GBManifoldGeneration::GBManifoldCapsuleCapsule(*pOtherCapsule, *pCapsule, outManifold);
			else
				GBManifoldGeneration::GBManifoldCapsuleCapsule(*pCapsule, *pOtherCapsule, outManifold);
		}
			break;
		}

		return outManifold.numContacts > 0;
	}

	bool generateBoxManifold(GBBoxCollider* pBox, GBCollider* pOther, GBSATCollisionData& data, GBManifold& outManifold)
	{
		switch (pOther->type)
		{
		case ColliderType::Sphere:
		{
			GBSphereCollider* pSphere = (GBSphereCollider*)pOther;
			GBManifoldGeneration::GBManifoldSphereBox(*pSphere, *pBox, outManifold);
		}
		break;
		case ColliderType::Box:
		{
			GBBoxCollider* pOtherBox = (GBBoxCollider*)pOther;
			if (GBManifoldGeneration::GBCollisionBoxBoxSAT(*pBox, *pOtherBox, data))
			{
				GBManifoldGeneration::GBManifoldBoxBox(*pBox, *pOtherBox, data, outManifold);
			}
		}
		break;
		case ColliderType::Capsule:
		{
			GBCapsuleCollider* pCapsule = (GBCapsuleCollider*)pOther;
			GBManifoldGeneration::GBManifoldCapsuleBox(*pCapsule, *pBox, outManifold);
		}
		break;
		}

		return outManifold.numContacts > 0;
	}

	bool generateSphereManifold(GBSphereCollider* pSphere, GBCollider* pOther, GBManifold& outManifold)
	{
		switch (pOther->type)
		{
		case ColliderType::Sphere:
		{
			GBSphereCollider* pOtherSphere = (GBSphereCollider*)pOther;
			GBManifoldGeneration::GBManifoldSphereSphere(*pSphere, *pOtherSphere, outManifold);
		}
		break;
		case ColliderType::Box:
		{
			GBBoxCollider* pBox = (GBBoxCollider*)pOther;
			GBManifoldGeneration::GBManifoldSphereBox(*pSphere, *pBox, outManifold);
		}
		break;
		case ColliderType::Capsule:
		{
			GBCapsuleCollider* pCapsule = (GBCapsuleCollider*)pOther;
			GBManifoldGeneration::GBManifoldCapsuleSphere(*pCapsule, *pSphere, outManifold);
		}
		break;
		}

		return outManifold.numContacts > 0;
	}

	bool generateColliderManifold(GBCollider* col, GBCollider* pOther, GBSATCollisionData& data, GBManifold& outManifold)
	{
		outManifold.clear();

		if (col && pOther)
		{
			if (col == pOther)
				return false;

			switch (col->type)
			{
			case ColliderType::Sphere:
				generateSphereManifold((GBSphereCollider*)col, pOther, outManifold);
				break;
			case ColliderType::Box:
				generateBoxManifold((GBBoxCollider*)col, pOther, data, outManifold);
				break;
			case ColliderType::Capsule:
				generateCapsuleManifold((GBCapsuleCollider*)col, pOther, outManifold);
				break;
			}
		}

		return outManifold.numContacts > 0;
	}

	bool generateBodyManifold(GBBody* a, GBBody* b, GBSATCollisionData& data)
	{
		manifoldStack.clear();
		for (int i = 0; i < a->colliders.size(); i++)
		{
			GBCollider* pCol = a->colliders[i];
			if(pCol)
			{ 
				for (int j = 0; j < b->colliders.size(); j++)
				{
					GBManifold manifold;
					GBCollider* pOtherCol = b->colliders[j];
					if (pOtherCol)
					{
						switch (pCol->type)
						{
						case ColliderType::Sphere:
						{
							GBSphereCollider* pSphere = (GBSphereCollider*)pCol;
							if (pSphere)
							{
								if (generateSphereManifold(pSphere, pOtherCol, manifold))
								{
									manifoldStack.push_back(manifold);
								}
							}
						}
						break;
						case ColliderType::Box:
						{
							GBBoxCollider* pBox = (GBBoxCollider*)pCol;
							if (pBox)
							{
								if (generateBoxManifold(pBox, pOtherCol, data, manifold))
								{
									manifoldStack.push_back(manifold);
								}
							}
						}
						break;
						case ColliderType::Capsule:
						{
							GBCapsuleCollider* pCapsule = (GBCapsuleCollider*)pCol;
							if (pCapsule)
							{
								if (generateCapsuleManifold(pCapsule, pOtherCol, manifold))
								{
									manifoldStack.push_back(manifold);
								}
							}
						}
						break;
						}
					}
				}
			}
		}
		return manifoldStack.size() > 0;
	}

	std::vector<GBBody*> sortBodiesByHeight(const std::vector<std::unique_ptr<GBBody>>& bodies)
	{
		std::vector<GBBody*> sorted;
		sorted.reserve(bodies.size());

		// Fill raw pointers
		for (auto& b : bodies)
			sorted.push_back(b.get());

		// Sort by height (Z-axis in Unreal)
		std::sort(sorted.begin(), sorted.end(),
			[](GBBody* a, GBBody* b) {
				return a->transform.position.z > b->transform.position.z;
			});

		return sorted;
	}

	std::vector<GBBody*> sortBodiesByHeight(const std::vector<GBBody*>& bodies)
	{
		std::vector<GBBody*> sorted;
		sorted.reserve(bodies.size());

		// Fill raw pointers
		for (auto& b : bodies)
			sorted.push_back(b);

		// Sort by height (Z-axis in Unreal)
		std::sort(sorted.begin(), sorted.end(),
			[](GBBody* a, GBBody* b) {
				return a->transform.position.z > b->transform.position.z;
			});

		return sorted;
	}


	bool containsPair(const std::vector<std::pair<GBBody*, GBBody*>>& handledPairs,
		const GBBody* a, const GBBody* b)
	{
		for (const auto& p : handledPairs)
		{
			if ((p.first == a && p.second == b) ||
				(p.first == b && p.second == a))
				return true;
		}
		return false;
	}


	enum BroadPhaseType
	{
		NONE = 0,
		UNIFORM_GRID = 1
	};

	BroadPhaseType broadPhaseType = BroadPhaseType::UNIFORM_GRID;

	void extractBroadPhaseBodies(int bodyIndex, std::vector<GBBody*>& bodies, std::vector<GBBody*>& outBodies)
	{
	//	switch (broadPhaseType)
	//	{
	//	case BroadPhaseType::NONE:
	//		for (int i = bodyIndex + 1; i < bodies.size(); i++)
	//			outBodies.push_back(bodies[i]);
	//		break;
	//	case BroadPhaseType::UNIFORM_GRID:
	//		GBBody* pBody = bodies[bodyIndex];
	//		gridMap.sampleBodies(pBody->aabb, outBodies);
	//		break;
	//	}
	}

	void extractBroadPhaseColliders(GBCollider* pCol, std::vector<GBCollider*>& outCols, BroadPhaseType sampleType = BroadPhaseType::UNIFORM_GRID)
	{
		switch (broadPhaseType)
		{
		case BroadPhaseType::NONE:
			for (int i = 0; i < colliders.size(); i++)
			{	
				outCols.push_back(colliders[i]);
			}
			break;
		case BroadPhaseType::UNIFORM_GRID:
			gridMap.sampleColliders(pCol->aabb, outCols);
			break;
		}
	}


	std::vector<GBAABB> getOccupiedCellAABBs(const GBBody& body)
	{
		std::vector<GBAABB> cellAABBs;
		if (broadPhaseType == BroadPhaseType::NONE)
			return cellAABBs;

		std::vector<GBCell*> occupiedCells;
		gridMap.sampleCells(body.aabb, occupiedCells, true);
		for (GBCell* pCell : occupiedCells)
			cellAABBs.push_back(pCell->toAABB());
		return cellAABBs;
	}


	std::vector<GBManifold> frameStaticManifolds;
	void handleStaticGeometry(GBBody& body, float deltaTime)
	{
		GBBoxCollider* pBox = nullptr;
		GBSphereCollider* pSphere = nullptr;
		GBCapsuleCollider* pCapsule = nullptr;


		// Now handle the sampled static geometry
		std::vector<GBStaticGeometry*> sampledStaticGeometry;
		gridMap.sampleStaticGeometry(body.aabb, sampledStaticGeometry);
		gridMap.sampleTerrainGridStaticGeometry(body.aabb, sampledStaticGeometry);

		for (GBStaticGeometry* pGeometry : sampledStaticGeometry)
		{
			GBTriangle* pTriangle = (GBTriangle*)pGeometry;
			if (pTriangle)
			{
				// Right now just handle single colliders
				for(int colIt = 0; colIt<body.colliders.size(); colIt++)
				{
					pBox = nullptr;
					pSphere = nullptr;
					pCapsule = nullptr;
					

					GBManifold manifold;
					GBSATCollisionData data;
					GBCollider* pCollider = body.colliders[colIt];
					switch (pCollider->type)
					{
					case ColliderType::Sphere:
						pSphere = (GBSphereCollider*)pCollider;
						if (pSphere)
						{
							if (GBManifoldGeneration::GBManifoldSphereTriangle(*pSphere, *pTriangle, manifold))
							{
								if (manifold.numContacts > 0)
								{
									body.addStaticGeometry(pGeometry);
								}
							}
						}
						break;
					case ColliderType::Box:
						pBox = (GBBoxCollider*)pCollider;
						if (pBox)
						{
							if (GBManifoldGeneration::GBCollisionBoxTriangleSAT(*pBox, *pTriangle, data))
							{
								
								if (GBManifoldGeneration::GBManifoldBoxTriangle(*pTriangle, *pBox, data, manifold))
								{
									body.addStaticGeometry(pGeometry);
								}
							}
						}
						break;
					case ColliderType::Capsule:
						pCapsule = (GBCapsuleCollider*)pCollider;
						if (pCapsule)
						{
							if (GBManifoldGeneration::GBManifoldCapsuleTriangle(*pCapsule, *pTriangle, manifold))
							{
								if (manifold.numContacts > 0)
								{
									body.addStaticGeometry(pGeometry);
								}
							}

						}
						break;
					}

					if (pBox)
					{
						if (manifold.numContacts > 0)
						{
							solveDynamicPenetration(manifold, *manifold.pIncident);
							solveStaticManifold(manifold, *manifold.pIncident);
							pBox->pBody->frameManifold.combine(manifold);
						}
					}
					else if (pCapsule)
					{
						if (manifold.numContacts > 0)
						{
							solveDynamicPenetration(manifold, *manifold.pIncident);
							solveStaticManifold(manifold, *manifold.pIncident);
							pCapsule->pBody->frameManifold.combine(manifold);
						}
					}
					else if (pSphere)
					{
						if (manifold.numContacts > 0)
						{
							solveDynamicPenetration(manifold, *manifold.pIncident);
							solveStaticManifold(manifold, *manifold.pIncident);
							pSphere->pBody->frameManifold.combine(manifold);
						}
					}
				}
			}
		}


	}

	std::vector<GBBody*> bodiesAsVector() const
	{
		std::vector<GBBody*> bodies;
		for (const auto& bodyIt : rigidBodies)
			bodies.push_back(bodyIt.get());
		return bodies;
	}


	struct ColliderPair
	{
		GBCollider* a;
		GBCollider* b;

		ColliderPair(GBCollider* x, GBCollider* y)
		{
			if (x < y) { a = x; b = y; }
			else { a = y; b = x; }
		}

		bool operator==(const ColliderPair& other) const
		{
			return a == other.a && b == other.b;
		}
	};

	struct BodyPair
	{
		GBBody* a;
		GBBody* b;

		BodyPair(GBBody* x, GBBody* y)
		{
			if (x < y) { a = x; b = y; }
			else { a = y; b = x; }
		}

		bool operator==(const BodyPair& other) const
		{
			return a == other.a && b == other.b;
		}
	};

	struct BodyPairHash
	{
		size_t operator()(const BodyPair& p) const noexcept
		{
			size_t h1 = std::hash<GBBody*>{}(p.a);
			size_t h2 = std::hash<GBBody*>{}(p.b);

			// boost-style hash combine
			return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
		}
	};


	struct ColliderPairHash
	{
		size_t operator()(const ColliderPair& p) const noexcept
		{
			size_t h1 = std::hash<GBCollider*>{}(p.a);
			size_t h2 = std::hash<GBCollider*>{}(p.b);

			// boost-style hash combine
			return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
		}
	};

	bool containsPair(const std::unordered_map<BodyPair, GBManifold, BodyPairHash>& map, const GBBody* a, const GBBody* b)
	{
		return map.find(BodyPair((GBBody*)a, (GBBody*)b)) != map.end();
	}


	GBVector3 gravity = GBVector3(0, 0, -10.0f);
	uint64_t frame = 0;
	static constexpr float maxPenetration = 0.05f;
	static constexpr float wakeThreshold = 0.3f;
    static constexpr float sleepThreshold = 0.1f; // linear+angular speed below which sleep is considered
	static constexpr float sleepTime = 0.4f;       // time to accumulate before sleeping
	static constexpr float sleepDampingRatio = 0.25f;  // The awake time needed out of the sleep timer that heavier contact based damping occurs 
	static constexpr float slop = 0.005f;
	std::unordered_map<ColliderPair, GBManifold, ColliderPairHash> pairManifolds;
	std::vector<GBManifold> manifoldStack;

	// ------------------------------------------------------------
	// SIMULATION STEP
	// ------------------------------------------------------------
	void step(float deltaTime)
	{

		if (deltaTime > maxDeltaTime)
			deltaTime = maxDeltaTime;
		deltaTime = timeScale * deltaTime;
		float interDeltaTime = deltaTime / (float)solverIterations;

		//We only need to reset is grounded every frame
		for (auto& rb : rigidBodies)
		{
			GBBody* body = rb.get();
			body->isGrounded = false;
			body->isGroundedCount = 0;
			body->frameManifold.clear();
			body->prevFrameVelocity = body->velocity;
			body->prevFrameAngularVelocity = body->angularVelocity;
			body->ignoreKinematicVelocityClamp = false;
		}

		frameStaticManifolds.clear();
		std::unordered_map<ColliderPair, GBManifold, ColliderPairHash> curPairManifolds;

		for (int iter = 0; iter < solverIterations; iter++)
		{
			for (auto& rb : rigidBodies)
			{
				GBBody* body = rb.get();

				if (body->ignoreSample)
					continue;

				if (body->isDirty)
				{
					body->updateColliders();
					gridMap.moveBody(*body);
					body->isDirty = false;
				}

				if (!body->isStatic)
				{
					gridMap.moveBody(*body);
				}

				if (body->isSleeping)
				{
					body->clearForces();
					body->velocity = { 0,0,0 };
					body->angularVelocity = { 0,0,0 };
				}
				else
					body->clearContactedBodies();

				if (!body->isSleeping && (body->useGravity))
					body->addForce(gravity * body->mass);
				body->update(interDeltaTime);
			}


			std::vector<GBBody*> sortedBodies = sortBodiesByHeight(rigidBodies);
			//std::unordered_set<BodyPair, BodyPairHash> dynamicPairs;
			std::unordered_set<ColliderPair, ColliderPairHash> dynamicPairs;

			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			// DYNAMIC SOLVER
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			for (int i = 0; i < sortedBodies.size(); i++)
			{
				GBBody* bodyA = sortedBodies[i];
				if (bodyA->ignoreSample || bodyA->isStatic || bodyA->isSleeping || bodyA->isTrigger)
					continue;
				bool didColide = false;

				for (GBCollider* colA: bodyA->colliders)
				{
					std::vector<GBCollider*> checkColliders;
					extractBroadPhaseColliders(colA, checkColliders);
					for (GBCollider* colB: checkColliders)
					{
						GBBody* bodyB = colB->pBody;
						if (bodyA == bodyB || !bodyA->sharesLayer(*bodyB))
							continue;

						ColliderPair pair(colA, colB);
						if (!dynamicPairs.insert(pair).second)
							continue;


						if (!bodyA->isStatic && !bodyB->isStatic)
						{
							// Skip triggers and handle in static solver
							if (bodyB->isTrigger)
								continue;

							GBSATCollisionData data;
							GBManifold manifold;
							if (generateColliderManifold(colA, colB,data, manifold))
							{
									if (manifold.numContacts == 0)
										continue;

									curPairManifolds[pair] = manifold;

									manifold.flipAndSwapIfContactOnTop(manifold.normal);

									if (!manifold.pIncident->isKinematic)
										solveDynamicPenetration(manifold, *manifold.pIncident);
									else
										solveDynamicPenetrationEqually(manifold);

									solveDynamicManifold(manifold, *manifold.pIncident, *manifold.pReference, interDeltaTime, !manifold.pIncident->isKinematic);


									if (manifold.pIncident)
									{
										manifold.pIncident->frameManifold.combine(manifold, true);
									}
									if (manifold.pReference)
									{
										GBManifold refManifold = GBManifold::asReference(manifold);
										manifold.pReference->frameManifold.combine(refManifold, true);
									}
							}

						}
					}

				}

			}


			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			// STATIC SOLVER
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			std::unordered_set<ColliderPair, ColliderPairHash> staticPairs;
			for (int i = 0; i < sortedBodies.size(); i++)
			{
				GBBody* bodyA = sortedBodies[i];
				if (bodyA->ignoreSample || bodyA->isSleeping || !bodyA->isMovable())
					continue;
				handleStaticGeometry(*bodyA, interDeltaTime);

				for (GBCollider* colA : bodyA->colliders)
				{
					std::vector<GBCollider*> checkColliders;
					extractBroadPhaseColliders(colA, checkColliders);
					for (GBCollider* colB : checkColliders)
					{
						GBBody* bodyB = colB->pBody;
						if (bodyA == bodyB || !bodyA->sharesLayer(*bodyB))
							continue;

						ColliderPair pair(colA, colB);
						if (!staticPairs.insert(pair).second)
							continue; // already processed this pair

						if (bodyA->isStatic && bodyB->isStatic)
							continue;

						GBSATCollisionData data;
						GBManifold manifold;
						if (generateColliderManifold(colA, colB, data, manifold))
						{
							if (manifold.numContacts == 0)
								continue;

							if (!bodyA->isMovable() || !bodyB->isMovable())
							{

								if (manifold.numContacts > 0)
								{

									curPairManifolds[ColliderPair(colA,colB)] = manifold;

									// If B is a trigger, just add manifold for callback and don't resolve
									if (bodyB->isTrigger)
										continue;

									if (manifold.pReference)
									{
										manifold.pIncident->addDynamicContact(manifold.pReference);
										manifold.pReference->addDynamicContact(manifold.pIncident);
									}

									solveDynamicPenetration(manifold, *manifold.pIncident);

									if (bodyIsPureColliderType(*manifold.pIncident, ColliderType::Sphere))
										solveStaticSphereManifold(manifold, *manifold.pIncident, interDeltaTime);
									else
										solveStaticManifold(manifold, *manifold.pIncident);


									if (manifold.pIncident)
									{
										manifold.pIncident->frameManifold.combine(manifold, true);
									}
									if (manifold.pReference)
									{
										GBManifold refManifold = GBManifold::asReference(manifold);
										manifold.pReference->frameManifold.combine(refManifold, true);
									}
								}
							}
							else
							{
								bodyA->addStaticContact(bodyB);
								bodyB->addStaticContact(bodyA);
							}
						}
					}
				}
			}


			for (auto& rb : rigidBodies)
			{
				GBBody* body = rb.get();

				if (body->frameManifold.hasGroundedManifold)
				{
					body->isGroundedCount++;
				}

				if (!body->isStatic)
					body->updateTransform(interDeltaTime);


				float speed = body->velocity.length();
				float angSpeed = body->angularVelocity.length();

				body->awakeTimer += interDeltaTime;

				bool hasStatic = body->hasStaticAttachment();
				bool bodyIsBox = bodyIsPureColliderType(*body, ColliderType::Box);
				if (hasStatic)
				{
					if (bodyIsBox &&
						body->frameManifold.numContacts > 2
						&& body->awakeTimer >  sleepTime*sleepDampingRatio)
					{

						GBContact contact;
						const static float minUp = 0.15f;
						float upness = GBMax(minUp, body->frameManifold.maxUpContact(contact));

						float damping = 0.999f - 0.015 * body->frameManifold.numContacts * upness;

						float t = 1.0f - exp(-0.8f * body->frameManifold.numContacts);
						damping = GBLerp(0.9998f, 0.965f,t);
						float verticleDampFactor = GBLerp(0.98, 0.85, t);

						GBVector3 velxy = body->velocity.xyComponent();
						velxy *= damping;



						body->velocity = GBVector3(velxy.x, velxy.y, body->velocity.z*verticleDampFactor*upness);
						body->angularVelocity *= damping*upness;
					}
					else
					{
						float damping = 0.99998f;

						body->velocity *= damping;
						body->angularVelocity *= damping;
					}
				}

				bool manifoldSleepCond = true;
				if (bodyIsBox &&
					body->frameManifold.numContacts < 3)
					manifoldSleepCond = false;

				if (!body->isKinematic && speed <  sleepThreshold 
					&& angSpeed < sleepThreshold && hasStatic  &&
					manifoldSleepCond)
				{
					body->sleepTimer += interDeltaTime;


					if (body->sleepTimer >= sleepTime)
					{
						body->isSleeping = true;
						body->velocity = GBVector3::zero();
						body->angularVelocity = GBVector3::zero();
					}
				}
				else if(speed > wakeThreshold && angSpeed > wakeThreshold)
				{
					body->wakeIsland();
				}
				
			}
			frame++;
		}

		for (auto& [pair, manifold] : curPairManifolds)
		{
			auto it = pairManifolds.find(pair);

			if (it == pairManifolds.end())
			{
				dispatchEnterListeners(pair.a->pBody->id, manifold, pair.b->pBody);
				dispatchEnterListeners(pair.b->pBody->id, manifold, pair.a->pBody);
			}
			else
			{
				dispatchStayListeners(pair.a->pBody->id, manifold, pair.b->pBody);
				dispatchStayListeners(pair.b->pBody->id, manifold, pair.a->pBody);
			}
		}

		for (auto& [pair, manifold] : pairManifolds)
		{
			if (curPairManifolds.find(pair)==curPairManifolds.end())
			{
				dispatchExitListeners(pair.a->pBody->id, pair.b->pBody);
				dispatchExitListeners(pair.b->pBody->id, pair.a->pBody);
			}
		}

		pairManifolds = curPairManifolds;
		curPairManifolds.clear();

		for (auto& rb : rigidBodies)
		{
			GBBody* body = rb.get();
			if (body->isGroundedCount > 0)
			{
				body->isGrounded = true;
			}

			if (body->isKinematic && !body->ignoreKinematicVelocityClamp)
			{

				float verticleSpeed = body->velocity.z;
				body->velocity = body->prevFrameVelocity;
				body->angularVelocity = body->prevFrameAngularVelocity;
				if (body->useGravity)
				{
					body->velocity.z = verticleSpeed;
				}
			}
		}
	}

	void wakeSurroundingBodies(GBAABB sample, float tolerance = 0.0f)
	{
		sample.grow(tolerance);
		std::vector<GBCollider*> cols;
		gridMap.sampleColliders(sample, cols);
		for (GBCollider* pCol : cols)
		{
			pCol->pBody->wakeIsland(2);
		}
	}

	void init(int iters = 10)
	{
		const float fixedDt = 1.0f / 60.0f;

		for (int i = 0; i < iters; i++)
		{
			step(fixedDt);
		}
	}

	std::vector<GBManifold> generateBodiesManifolds(std::vector<GBBody*> bodies, BroadPhaseType sampleType = BroadPhaseType::NONE)
	{
		std::vector<GBManifold> generatedManifolds;
		for (GBBody* bodyA: bodies)
		{
			for (int k = 0; k < bodyA->colliders.size(); k++)
			{
				GBCollider* colA = bodyA->colliders[k];

				GBAABB sample = colA->aabb;
				std::vector<GBCollider*> cols;
				extractBroadPhaseColliders(colA, cols, sampleType);
				for (GBCollider* colB : cols)
				{
					GBBody* bodyB = colB->pBody;

					if (bodyA->isMovable() && bodyB->isMovable())
					{
						GBSATCollisionData data;
						GBManifold manifold;
						if (generateColliderManifold(colA, colB, data, manifold))
						{
							generatedManifolds.push_back(manifold);
						}
					}
				}
			}
		}

		return generatedManifolds;
	}

	std::vector<GBManifold> generateStaticManifolds()
	{
		std::vector<GBManifold> generatedManifolds;
		for (int i = 0; i < rigidBodies.size(); i++)
		{
			for (int j = 0; j < triangles.size(); j++)
			{
				GBBody* body = rigidBodies[i].get();

				if (body->isSleeping) continue;

				if (body->isDynamic())
				{
					for (int k = 0; k < body->colliders.size(); k++)
					{
						if (body->colliders[k]->type == ColliderType::Box)
						{
							GBBoxCollider* pBox = (GBBoxCollider*)body->colliders[k];
							GBSATCollisionData data;
							GBManifold manifold;
							if (GBManifoldGeneration::GBCollisionBoxTriangleSAT(*pBox, *triangles[j].get(), data))
							{
								if (GBManifoldGeneration::GBManifoldBoxTriangle(*triangles[j].get(), *pBox, data, manifold))
								{
									generatedManifolds.push_back(manifold);
								}
							}
						}
					}
				}
			}
		}

		return generatedManifolds;
	}

	bool bodyIsPureColliderType(const GBBody& body, const ColliderType type) const
	{
		if (body.colliders.size() == 1)
		{
			switch (type)
			{
			case ColliderType::Capsule:
				return body.colliders[0]->type == ColliderType::Capsule;
				break;
			case ColliderType::Box:
				return body.colliders[0]->type == ColliderType::Box;
				break;
			case ColliderType::Sphere:
				return body.colliders[0]->type == ColliderType::Sphere;
				break;
			}
		}
		return false;
	}



	// Wrapper to call recursion
	void solveDynamicPenetration(GBManifold& manifold, GBBody& root)
	{
		const float percent = 1.0f - (1.0f / solverIterations);
		float penetration = manifold.separation - slop;
		float adjustedSeparation =
			GBMin(GBMax(penetration, 0.0f), maxPenetration);
		if (root.isMovable())
		{

			root.transform.position +=
				manifold.normal * adjustedSeparation * percent;
		}
	}

	void solveDynamicPenetrationEqually(GBManifold& manifold)
	{
		bool iDynamic = manifold.pIncident && manifold.pIncident->isMovable() ? true : false;
		bool rDynamic = manifold.pReference && manifold.pReference->isMovable() ? true : false;
		float adjustedSeparation = GBMin(manifold.separation*(1.0f- slop), maxPenetration);
		if (iDynamic && rDynamic)
		{
			manifold.pReference->transform.position -= adjustedSeparation * manifold.normal * 0.5f;
			manifold.pIncident->transform.position += adjustedSeparation * manifold.normal * 0.5f;
		}
		else if (iDynamic && !manifold.pIncident->isStatic)
		{
			manifold.pIncident->transform.position += adjustedSeparation * manifold.normal;
		}
		else if (rDynamic)
		{
			manifold.pReference->transform.position -= adjustedSeparation * manifold.normal;

		}
	}

	bool raycast(
		const GBVector3& rayOrigin,
		const GBVector3& rayDir,
		GBContact& outContact,
		float maxDistance = FLT_MAX,
		unsigned int mask = 0xFFFFFFFF)
	{
		return gridMap.raycast(rayOrigin, rayDir, outContact, maxDistance, mask);
	}
};