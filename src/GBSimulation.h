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
	std::vector< std::unique_ptr<GBQuad>> quads;
	std::vector< std::unique_ptr<GBTriangle>> triangles;
	std::vector< std::unique_ptr<GBTerrain>> terrains;
	GBBoxCollider* pStaticBox;
	uint32_t idCount;
	std::unordered_map<uint32_t, std::vector<std::function<void(const GBManifold& manifold, GBBody* pOther)>>> enterListeners;
	std::unordered_map<uint32_t, std::vector<std::function<void(const GBManifold& manifold, GBBody* pOther)>>> stayListeners;
	std::unordered_map<uint32_t, std::vector<std::function<void(GBBody* pOther)>>> exitListeners;

	//: gridMap(GBGridMap(GBVector3(-50,-50, -25), 1.0f, 100, 100, 50, 1, 1, 1))
	GBSimulation()
		: gridMap()
	{
		idCount = 0;
		setupStaticBox();
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

	void setupStaticBox()
	{
		GBBody* staticBody = createBody();
		staticBody->isStatic = true;
		pStaticBox = attachBoxCollider(staticBody, { 0.5f,0.5f,0.5f }, GBTransform(), false);
		staticBody->ignoreSample = true;
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

	GBQuad* createQuad(GBTransform transform, GBVector3 position, float xSize, float ySize)
	{
		quads.push_back(std::make_unique<GBQuad>(transform, position, xSize, ySize));
		GBQuad* q = quads.back().get();
		q->id = getId();
		return q;
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
				}
			}
		}

		pTerrain->spacing = spacing;
		pTerrain->cellsX = pTerrain->triangles.size();
		pTerrain->cellsY = pTerrain->triangles[0].size() / 2;
		pTerrain->cellsZ = (int)(pTerrain->maxHeight - pTerrain->minHeight) / spacing + 1;

		pTerrain->pGrid = new GBGrid(origin, spacing, pTerrain->cellsX, pTerrain->cellsY, pTerrain->cellsZ, getId(), GBGridType::TERRAIN);

		for (const std::vector<GBTriangle*> tris : pTerrain->triangles)
		{
			for (const GBTriangle* pTri : tris)
			{
				GBAABB triAABB = pTri->toAABB();
				triAABB.grow(-0.01f * spacing);
				pTerrain->pGrid->insertStaticGeometry(triAABB, *(GBStaticGeometry*)pTri);
			}
		}

		gridMap.insertTerrainGrid(*pTerrain->pGrid);
		return pTerrain;
	}

	GBQuad* getQuad(int index)
	{
		return (index < quads.size()) ? quads[index].get() : nullptr;
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

		// remove from body's collider list
		if (pCollider->pBody) {
			auto& bodyColliders = pCollider->pBody->colliders;
			bodyColliders.erase(std::remove(bodyColliders.begin(), bodyColliders.end(), pCollider), bodyColliders.end());
		}

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

	bool deleteBody(GBBody* pBody)
	{
		if (!pBody) return false;

		gridMap.removeBody(*pBody);

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

	int solverIterations = 6 ;   // 6–12 typical
	static constexpr float maxDeltaTime = 1.0f / 60.0f;
	float timeScale = 1.0f;


	bool bodyIsBox(const GBBody& pBody)
	{
		if (pBody.colliders.size() == 1)
		{
			return pBody.colliders[0]->type == ColliderType::Box;
		}
		return false;
	}

	void solveDynamicManifold(const GBManifold& m, GBBody& A, GBBody& B, float dt, bool canTreatAsStatic = true)
	{
		//if (A.isKinematic || B.isKinematic)
		//	return;

		const float restitution = 0.1f;
		const float percent = 0.3f;
		const float forceCap = 100.0f;
		const float slopeRequirement = 0.9f;
		const float staticManifoldThreshold = 1.0f;
		const float impulseCompensation = 3.0f;

		int count = m.numContacts;

		GBVector3 dv = B.velocity - A.velocity;
		float vRelN = GBDot(dv, m.normal);
		//if (!m.isEdge && GBDot(GBVector3::up(), m.normal) > 0.90f && m.numContacts > 1 && GBAbs(vRelN) < 1.0f)
		if (canTreatAsStatic)
		{
			float upness = GBDot(GBVector3::up(), m.normal);
			if (m.pIncident && m.pIncident->colliders[0]->type == ColliderType::Sphere && GBAbs(vRelN) < staticManifoldThreshold)
			{
				//if(!m.pReference->isOnLayer(LAYER_STATIC_DYNAMIC_SPHERE) || upness>0.95)
				solveStaticSphereManifold(m, *m.pIncident, dt);
				//solveStaticCapsuleManifold(m, *m.pIncident);
				return;
			}
			else if (GBAbs(vRelN) < staticManifoldThreshold && m.numContacts > 1 && GBDot(GBVector3::up(), m.normal) > slopeRequirement)
			{
				//if(upness>0.95)
				//	solveStaticManifold(m, *m.pIncident, dt, true);
				//else if(m.pIncident->awakeTimer >0.5)
				//	solveStaticManifold(m, *m.pIncident, dt, true);
				if (upness > 0.95)
				{
					solveStaticManifold(m, *m.pIncident);
				}
				else if (m.pIncident->awakeTimer > 0.5)
				{
					solveStaticManifold(m, *m.pIncident);
				}
				return;
			}
		}

		bool ignoreA = A.isStatic || A.isKinematic;
		bool ignoreB = B.isStatic || B.isKinematic;
		float aModifier = 1.0f;
		float bModifier = 1.0f;



		for (int i = 0; i < count; i++)
		{
			const GBContact& c = m.contacts[i];
			GBVector3 n = c.normal;

			GBVector3 rA = c.position - A.transform.position;
			GBVector3 rB = c.position - B.transform.position;

			GBVector3 vA = A.velocity + GBCross(A.angularVelocity, rA);
			GBVector3 vB = B.velocity + GBCross(B.angularVelocity, rB);
			GBVector3 vRel = vB - vA;

			float vn = GBDot(vRel, n);

			//if (vn < 1.0f && vn>= 0.0f)
			//{
			//	A.setPointConstraint(m.contacts[i].position);
			//	B.setPointConstraint(m.contacts[i].position);
			if (vn < 0.0f)
				continue;

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

			GBVector3 impulse = n * j * impulseCompensation;

			if (!ignoreA)
				A.velocity -= impulse * A.invMass * aModifier;
			if (!ignoreB)
				B.velocity += impulse * B.invMass * bModifier;

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

					const float mu = 0.25f; // friction coefficient
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

		//// Linear damping
		//A.velocity *= 0.999f;
		//B.velocity *= 0.999f;

		//// Angular damping
		//A.angularVelocity *= 0.999f;
		//B.angularVelocity *= 0.999f;

		//if (GBDot(A.velocity, B.velocity) < 0)
		//{
		//	A.resetContactConstraints();
		//	B.resetContactConstraints();
		//}
	}

	void solveStaticSphereManifold(const GBManifold& manifold, GBBody& body, float dt)
	{
		if (body.isKinematic)
			return;

		const float restitution = 0.25f;
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
			if (body.isPlayerController)
			{
				float slope = GBDot(GBVector3::up(), manifold.normal);

				if (slope >= body.playerSlopeValue)
				{
					// Grounded on a walkable surface
					// Stop vertical velocity
					body.velocity.z = 0;
				}
			}
			return;
		}

		const float restitution = 0.1f;

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

	////bool ignoreFriction = false;
	//if (vn > -normalRestingThreshold && vtLen < tangentialRestingThreshold)
	//{
	//	//if (!manifold.pReference->pBody->isDynamic())
	//	{
	//		if (GBDot(manifold.normal, GBVector3::up()) < 0.80f)
	//		{
	//			body.resetConstraints();
	//		}
	//		//if (!otherIsDynamic && body.constraintType == GBBody::ConstraintType::NONE &&
	//		if (body.constraintType == GBBody::ConstraintType::NONE)
	//		{
	//			GBBody* pOther = manifold.pReference && manifold.pReference ? manifold.pReference : nullptr;
	//			if (manifold.numContacts > 0)
	//				body.setConstraintEdge(manifold.contacts[0].position, manifold.numContacts > 1 ? manifold.contacts[1].position :
	//					manifold.contacts[0].position, pOther);
	//			//	body.setConstraintPoint(manifold.contacts[0].position, pOther);
	//			//else

	//		}
	//	}
	//}
	//else
	//{
	//	if (manifold.pReference && manifold.pReference)
	//		manifold.pReference->wake();
	//	if (manifold.pIncident && manifold.pIncident)
	//		manifold.pIncident->wake();
	//}

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

	enum SolverType
	{
		SPHERE_SPHERE = 0,
		BOX_SPHERE = 1,
		BOX_BOX = 2,
		COMPOUND = 3,
		CAPSULE_CAPSULE = 4,
		CAPSULE_BOX = 5,
		CAPSULE_SPHERE = 6
	};

	bool generateBodyManifold(GBBody* a, GBBody* b, GBSATCollisionData& data, GBManifold& outManifold, SolverType& outType)
	{
		GBSphereCollider* pSphereA = nullptr;
		GBSphereCollider* pSphereB = nullptr;
		GBSphereCollider* pSphere = nullptr;
		GBBoxCollider* pBoxA = nullptr;
		GBBoxCollider* pBoxB = nullptr;
		GBBoxCollider* pBox = nullptr;
		GBCapsuleCollider* pCapA = nullptr;
		GBCapsuleCollider* pCapB = nullptr;
		GBCapsuleCollider* pCap = nullptr;
		if (a->colliders.size() == 1 && b->colliders.size() == 1)
		{
			if (a->colliders[0]->type == ColliderType::Sphere && b->colliders[0]->type == ColliderType::Sphere)
			{
				pSphereA = (GBSphereCollider*)a->colliders[0];
				pSphereB = (GBSphereCollider*)b->colliders[0];
				outType = SPHERE_SPHERE;
			}
			else if (a->colliders[0]->type == ColliderType::Box && b->colliders[0]->type == ColliderType::Box)
			{
				pBoxA = (GBBoxCollider*)a->colliders[0];
				pBoxB = (GBBoxCollider*)b->colliders[0];
				outType = BOX_BOX;
			}
			else if (a->colliders[0]->type == ColliderType::Capsule && b->colliders[0]->type == ColliderType::Capsule)
			{
				pCapA = (GBCapsuleCollider*)a->colliders[0];
				pCapB = (GBCapsuleCollider*)b->colliders[0];
				outType = CAPSULE_CAPSULE;
			}
			else
			{
				if (a->colliders[0]->type == ColliderType::Capsule || b->colliders[0]->type == ColliderType::Capsule)
				{
					if (a->colliders[0]->type == ColliderType::Capsule)
					{
						pCap = (GBCapsuleCollider*)a->colliders[0];
						if (b->colliders[0]->type == ColliderType::Box)
						{
							pBox = (GBBoxCollider*)b->colliders[0];
							outType = CAPSULE_BOX;
						}
						else
						{
							pSphere = (GBSphereCollider*)b->colliders[0];
							outType = CAPSULE_SPHERE;
						}
					}
					else if (b->colliders[0]->type == ColliderType::Capsule)
					{
						pCap = (GBCapsuleCollider*)b->colliders[0];
						if (a->colliders[0]->type == ColliderType::Box)
						{
							pBox = (GBBoxCollider*)a->colliders[0];
							outType = CAPSULE_BOX;
						}
						else
						{
							pSphere = (GBSphereCollider*)a->colliders[0];
							outType = CAPSULE_SPHERE;
						}
					}
				}
				else if (a->colliders[0]->type == ColliderType::Box)
				{
					pBox = (GBBoxCollider*)a->colliders[0];
					pSphere = (GBSphereCollider*)b->colliders[0];
					outType = BOX_SPHERE;
				}
				else
				{
					pBox = (GBBoxCollider*)b->colliders[0];
					pSphere = (GBSphereCollider*)a->colliders[0];
					outType = BOX_SPHERE;
				}
			}
		}
		else
		{
			outType = COMPOUND;
		}


		bool doesIntersect = false;
		outManifold.reset();
		switch (outType)
		{
		case SPHERE_SPHERE:
			if (pSphereA && pSphereB)
				doesIntersect = GBManifoldGeneration::GBManifoldSphereSphere(*pSphereA, *pSphereB, outManifold);
			break;
		case BOX_SPHERE:
			if (pSphere && pBox)
				doesIntersect = GBManifoldGeneration::GBManifoldSphereBox(*pSphere, *pBox, outManifold);
			break;
		case BOX_BOX:
			if (pBoxA && pBoxB)
			{
				if (GBManifoldGeneration::GBCollisionBoxBoxSAT(*pBoxA, *pBoxB, data))
				{
					doesIntersect = GBManifoldGeneration::GBManifoldBoxBox(*pBoxA, *pBoxB, data, outManifold);
				}
			}
			break;
		case CAPSULE_CAPSULE:
			if (pCapA && pCapB)
			{
				// We want kinematic/dynamic bodies to be incident....
				if (pCapB->pBody->isDynamic() || pCapB->pBody->isKinematic)
					doesIntersect = GBManifoldGeneration::GBManifoldCapsuleCapsule(*pCapA, *pCapB, outManifold);
				else
					doesIntersect = GBManifoldGeneration::GBManifoldCapsuleCapsule(*pCapB, *pCapA, outManifold);
			}
			break;
		case CAPSULE_BOX:
			if (pCap && pBox)
			{
				doesIntersect = GBManifoldGeneration::GBManifoldCapsuleBox(*pCap, *pBox, outManifold);
			}
			break;
		case CAPSULE_SPHERE:
			if (pCap && pSphere)
			{
				doesIntersect = GBManifoldGeneration::GBManifoldCapsuleSphere(*pCap, *pSphere, outManifold);
			}
			break;
		case COMPOUND:
			for (int i = 0; i < a->colliders.size(); i++)
			{
				for (int j = 0; j < b->colliders.size(); j++)
				{
					GBManifold manifold;
					data = GBSATCollisionData();
					GBCollider* colA = a->colliders[i];
					GBCollider* colB = b->colliders[j];
					if (colA->type == ColliderType::Sphere && colB->type == ColliderType::Sphere)
					{
						GBManifoldGeneration::GBManifoldSphereSphere(*(GBSphereCollider*)colA, *(GBSphereCollider*)colB, manifold);
					}
					else if (colA->type == ColliderType::Box && colB->type == ColliderType::Sphere)
					{
						GBManifoldGeneration::GBManifoldSphereBox(*(GBSphereCollider*)colB, *(GBBoxCollider*)colA, manifold);
					}
					else if (colA->type == ColliderType::Sphere && colB->type == ColliderType::Box)
					{
						GBManifoldGeneration::GBManifoldSphereBox(*(GBSphereCollider*)colA, *(GBBoxCollider*)colB, manifold);
					}
					else if (colA->type == ColliderType::Box && colB->type == ColliderType::Box)
					{
						if (GBManifoldGeneration::GBCollisionBoxBoxSAT(*(GBBoxCollider*)colA, *(GBBoxCollider*)colB, data))
						{
							GBManifoldGeneration::GBManifoldBoxBox(*(GBBoxCollider*)colA, *(GBBoxCollider*)colB, data, manifold);
						}
					}
					if (manifold.numContacts > 0)
					{
						if (manifold.separation > outManifold.separation)
						{
							outManifold.useNormal(manifold);
							outManifold.pReference = manifold.pReference;
							outManifold.pIncident = manifold.pIncident;
						}
						outManifold.combine(manifold);
					}
				}
			}
			break;
		}

		return outManifold.numContacts > 0;
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
		switch (broadPhaseType)
		{
		case BroadPhaseType::NONE:
			for (int i = bodyIndex + 1; i < bodies.size(); i++)
				outBodies.push_back(bodies[i]);
			break;
		case BroadPhaseType::UNIFORM_GRID:
			GBBody* pBody = bodies[bodyIndex];
			gridMap.sampleBodies(pBody->aabb, outBodies);
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
		std::vector<GBAABB> occupiedCells = getOccupiedCellAABBs(body);

		GBBoxCollider* pBox = nullptr;
		GBSphereCollider* pSphere = nullptr;
		GBCapsuleCollider* pCapsule = nullptr;

		for (int i = 0; i < occupiedCells.size(); i++)
		{
			// Right now just handle single colliders
			if (body.colliders.size() == 1)
			{
				GBManifold manifold;
				GBSATCollisionData data;
				GBCollider* pCollider = body.colliders[0];

				// Update the static box to the cell location
				pStaticBox->halfExtents = occupiedCells[i].halfExtents;
				pStaticBox->pBody->transform.position = occupiedCells[i].center;
				pStaticBox->pBody->updateColliders();

				switch (pCollider->type)
				{
				case ColliderType::Sphere:
					pSphere = (GBSphereCollider*)pCollider;
					if (pSphere)
					{
						GBContact contact;
						if (GBManifoldGeneration::GBContactSphereAABB(*pSphere, pStaticBox->aabb, contact))
						{
							manifold.addContact(contact, false);
							manifold.pIncident = pSphere->pBody;
							manifold.normal = contact.normal;
							manifold.separation = contact.penetrationDepth;
							manifold.clampSeparation(slop);
							pSphere->pBody->frameManifold.combine(manifold);
							solveDynamicPenetration(manifold, *manifold.pIncident, false);
							solveStaticSphereManifold(manifold, *pSphere->pBody, deltaTime);
						}
					}
					break;
				case ColliderType::Box:
					pBox = (GBBoxCollider*)pCollider;
					if (pBox)
					{
						if (GBManifoldGeneration::GBCollisionBoxBoxSAT(*pStaticBox, *pBox, data))
						{
							if (GBManifoldGeneration::GBManifoldBoxBox(*pStaticBox, *pBox, data, manifold))
							{
								frameStaticManifolds.push_back(manifold);
								pBox->pBody->frameManifold.combine(manifold);
								solveDynamicPenetration(manifold, *manifold.pIncident);
								solveStaticManifold(manifold, *manifold.pIncident);
							}
						}
					}
					break;

				case ColliderType::Capsule:
					pCapsule = (GBCapsuleCollider*)pCollider;
					if (pCapsule)
					{
						if (GBManifoldGeneration::GBManifoldCapsuleBox(*pCapsule, *pStaticBox, manifold))
						{
							solveDynamicPenetration(manifold, *manifold.pIncident, false);
							solveStaticManifold(manifold, *manifold.pIncident);
						}
					}
					break;
				}
			}
		}

		// Now handle the sampled static geometry
		std::vector<GBStaticGeometry*> sampledStaticGeometry;
		gridMap.sampleStaticGeometry(body.aabb, sampledStaticGeometry);
		gridMap.sampleTerrainGridStaticGeometry(body.aabb, sampledStaticGeometry);

		pBox = nullptr;
		pSphere = nullptr;
		pCapsule = nullptr;

		GBManifold combinedManifold;
		combinedManifold.separation = 0.0f;
		int numManifolds = 0;

		for (GBStaticGeometry* pGeometry : sampledStaticGeometry)
		{
			GBTriangle* pTriangle = (GBTriangle*)pGeometry;
			if (pTriangle)
			{
				// Right now just handle single colliders
				if (body.colliders.size() == 1)
				{
					GBSATCollisionData data;
					GBCollider* pCollider = body.colliders[0];
					switch (pCollider->type)
					{
					case ColliderType::Sphere:
						pSphere = (GBSphereCollider*)pCollider;
						if (pSphere)
						{
							GBContact contact;
							GBManifold manifold;
							if (GBManifoldGeneration::GBContactSphereTriangle(*pSphere, *pTriangle, contact))
							{
								manifold.addContact(contact, false);
								manifold.pIncident = pSphere->pBody;
								manifold.normal = contact.normal;
								manifold.separation = contact.penetrationDepth;
								manifold.clampSeparation(manifold.separation - slop);
								pSphere->pBody->frameManifold.combine(manifold);
								solveDynamicPenetration(manifold, *manifold.pIncident);
								solveStaticSphereManifold(manifold, *pSphere->pBody, deltaTime);
							}
						}
						break;
					case ColliderType::Box:
						pBox = (GBBoxCollider*)pCollider;
						if (pBox)
						{
							if (GBManifoldGeneration::GBCollisionBoxTriangleSAT(*pBox, *pTriangle, data))
							{
								GBManifold manifold;
								if (GBManifoldGeneration::GBManifoldBoxTriangle(*pTriangle, *pBox, data, manifold))
								{
									if (manifold.separation > combinedManifold.separation)
									{
										combinedManifold.useNormal(manifold);
										combinedManifold.combine(manifold);
									}

									body.addStaticGeometry(pGeometry);
								}
							}
						}
						break;
					case ColliderType::Capsule:
						pCapsule = (GBCapsuleCollider*)pCollider;
						if (pCapsule)
						{
							GBManifold manifold;
							if (GBManifoldGeneration::GBManifoldCapsuleTriangle(*pCapsule, *pTriangle, manifold))
							{
								if (manifold.numContacts > 0)
								{
									combinedManifold.normal += manifold.normal;
									numManifolds++;
									combinedManifold.combine(manifold);
									if (manifold.separation > combinedManifold.separation)
									{
										combinedManifold.separation = manifold.separation;
									}
								}
							}

						}
						break;
					}
				}
			}
		}

		if (pBox)
		{
			if (combinedManifold.numContacts > 0)
			{
				combinedManifold.clampSeparation(slop);
				pBox->pBody->frameManifold.combine(combinedManifold);
				solveDynamicPenetration(combinedManifold, *combinedManifold.pIncident);
				solveStaticManifold(combinedManifold, *combinedManifold.pIncident);
			}
		}
		else if (pCapsule)
		{
			combinedManifold.pIncident = pCapsule->pBody;
			combinedManifold.normal *= 1.0f / numManifolds;
			combinedManifold.normal = GBNormalize(combinedManifold.normal);
			if (combinedManifold.numContacts > 0)
			{
				solveDynamicPenetration(combinedManifold, *combinedManifold.pIncident, false);
				solveStaticManifold(combinedManifold, *combinedManifold.pIncident);
			}
		}
	}

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
		size_t operator()(const BodyPair& p) const
		{
			return std::hash<GBBody*>()(p.a) ^ (std::hash<GBBody*>()(p.b) << 1);
		}
	};

	bool containsPair(const std::unordered_map<BodyPair, GBManifold, BodyPairHash>& map, const GBBody* a, const GBBody* b)
	{
		return map.find(BodyPair((GBBody*)a, (GBBody*)b)) != map.end();
	}


	GBVector3 gravity = GBVector3(0, 0, -10.0f);
	uint64_t frame = 0;
	static constexpr float maxPenetration = 0.1f;
	const float slop = 0.05f;
	std::unordered_map<BodyPair, GBManifold, BodyPairHash> pairManifolds;
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
		}

		frameStaticManifolds.clear();
		std::unordered_map<BodyPair, GBManifold, BodyPairHash> curPairManifolds;

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
					body->clearContactedBodies();
					body->velocity = { 0,0,0 };
					body->angularVelocity = { 0,0,0 };
				}

				if (!body->isSleeping && (!body->isKinematic || body->isPlayerController))
					body->addForce(gravity * body->mass);
				body->update(interDeltaTime);
				body->clearContactedBodies();
			}


			std::vector<GBBody*> sortedBodies = sortBodiesByHeight(rigidBodies);
			std::vector<std::pair<GBBody*, GBBody*>> dynamicPairs;
			// Do all the boxes now
			for (int i = 0; i < sortedBodies.size(); i++)
			{
				GBBody* bodyA = sortedBodies[i];
				if (bodyA->ignoreSample || bodyA->isStatic || bodyA->isSleeping || bodyA->isTrigger)
					continue;
				bool didColide = false;
				std::vector<GBBody*> checkBodies;
				extractBroadPhaseBodies(i, sortedBodies, checkBodies);
				for (int j = 0; j < checkBodies.size(); j++)
				{
					GBBody* bodyB = checkBodies[j];
					if (bodyA == bodyB || !bodyA->sharesLayer(*bodyB))
						continue;
					std::pair<GBBody*, GBBody*> bodyPair;
					bodyPair.first = bodyA;
					bodyPair.second = bodyB;
					if (!containsPair(dynamicPairs, bodyA, bodyB))
						dynamicPairs.push_back(bodyPair);
					else
						continue;
					//if (bodyA->isSleeping || bodyB->isSleeping) continue;


					if (!bodyA->isStatic && !bodyB->isStatic)
					{
						// Skip triggers and handle in static solver
						if (bodyB->isTrigger)
							continue;

						GBSATCollisionData data;
						GBManifold manifold;
						SolverType type;
						if (generateBodyManifold(bodyA, bodyB, data, manifold, type))
						{
							if (manifold.numContacts == 0)
								continue;

							curPairManifolds[BodyPair(bodyA, bodyB)] = manifold;

							switch (type)
							{
							case SPHERE_SPHERE:
								manifold.pIncident->transform.position += manifold.normal * manifold.separation;
								solveDynamicManifold(manifold, *manifold.pReference, *manifold.pIncident, interDeltaTime, true);
								break;
							case BOX_SPHERE:
								solveDynamicPenetrationEqually(manifold);
								solveDynamicManifold(manifold,  *manifold.pIncident,*manifold.pReference, interDeltaTime, true);
								break;
							case BOX_BOX:
							{
								if (!manifold.pReference->isStatic)
									manifold.flipAndSwapIfContactOnTop(manifold.normal);
								solveDynamicPenetration(manifold, *manifold.pIncident);
								solveDynamicManifold(manifold, *manifold.pIncident, *manifold.pReference, interDeltaTime, true);
							}
							break;
							case CAPSULE_BOX:
								solveDynamicPenetrationEqually(manifold);
								solveDynamicManifold(manifold, *manifold.pReference, *manifold.pIncident, interDeltaTime, true);
								break;
							case CAPSULE_SPHERE:
								solveDynamicPenetrationEqually(manifold);
								solveDynamicManifold(manifold, *manifold.pReference, *manifold.pIncident, interDeltaTime, true);
								break;
							case CAPSULE_CAPSULE:
								solveDynamicPenetrationEqually(manifold);
								solveDynamicManifold(manifold, *manifold.pReference, *manifold.pIncident, interDeltaTime, true);
								break;
							case COMPOUND:
								solveDynamicPenetrationEqually(manifold);
								solveDynamicManifold(manifold, *bodyA, *bodyB, interDeltaTime, true);
								break;
							}
							bodyA->addDynamicContact(bodyB);
							bodyB->addDynamicContact(bodyA);

							if (manifold.pIncident && manifold.pReference)
							{
								manifold.pIncident->frameManifold.combine(manifold);
								GBManifold refManifold = GBManifold::asReference(manifold);
								manifold.pReference->frameManifold.combine(refManifold);
							}
						}

					}

				}

			}

			std::vector<std::pair<GBBody*, GBBody*>> staticPairs;
			for (int i = 0; i < sortedBodies.size(); i++)
			{
				GBBody* bodyA = sortedBodies[i];
				if (bodyA->ignoreSample || bodyA->isSleeping || bodyA->isTrigger)
					continue;
				if (bodyA->isDynamic() || bodyA->isKinematic)
					handleStaticGeometry(*bodyA, interDeltaTime);

				std::vector<GBBody*> checkBodies;
				extractBroadPhaseBodies(i, sortedBodies, checkBodies);
				for (int j = 0; j < checkBodies.size(); j++)
				{
					GBBody* bodyB = checkBodies[j];
					if (bodyA == bodyB || !bodyA->sharesLayer(*bodyB))
						continue;
					std::pair<GBBody*, GBBody*> bodyPair;
					bodyPair.first = bodyA;
					bodyPair.second = bodyB;
					if (!containsPair(staticPairs, bodyA, bodyB))
						staticPairs.push_back(bodyPair);
					else
						continue;
					if (bodyA->isStatic && bodyB->isStatic)
						continue;

					GBSATCollisionData data;
					GBManifold manifold;
					manifold.numContacts = 0;
					SolverType type;
					if (generateBodyManifold(bodyA, bodyB, data, manifold, type))
					{
						if (manifold.numContacts == 0)
							continue;

						GBBody* dynamicBody, * staticBody;
						//if (bodyA->isStatic || bodyB->isStatic)
						if (!bodyA->isDynamic() || !bodyB->isDynamic())
						{
							if (bodyA->isDynamic() || bodyA->isKinematic)
							{
								dynamicBody = bodyA;
								staticBody = bodyB;
							}
							else
							{
								dynamicBody = bodyB;
								staticBody = bodyA;
							}
							dynamicBody->addStaticContact(staticBody);

							bool wakeSleeping = false;
							static const float wakeThreshold = 0.055f;
							if (data.minOverlap > wakeThreshold)
								wakeSleeping = true;
							staticBody->addDynamicContact(dynamicBody, wakeSleeping, true);
							if (manifold.numContacts > 0)
							{

								curPairManifolds[BodyPair(bodyA, bodyB)] = manifold;

								// If B is a trigger, just add manifold for callback and don't resolve
								if (bodyB->isTrigger)
									continue;

								switch (type)
								{
								case SPHERE_SPHERE:
									// Don't bother, treat as dynamic
									break;
								case BOX_SPHERE:
									solveDynamicPenetration(manifold, *manifold.pIncident, false);
									solveStaticSphereManifold(manifold, *manifold.pIncident, interDeltaTime);

									break;
								case BOX_BOX:
									if (!manifold.pReference->isStatic)
										manifold.flipAndSwapIfContactOnTop(manifold.normal);
									if (!manifold.pIncident)
										continue;
									solveDynamicPenetration(manifold, *manifold.pIncident);
									solveStaticManifold(manifold, *manifold.pIncident);
									break;
								case CAPSULE_BOX:
									solveDynamicPenetration(manifold, *manifold.pIncident);
									// If the capsule is dynamic use the capsule solver, otherwise use the box solver
									if (manifold.pIncident->isDynamic() || manifold.pIncident->isPlayerController)
										solveStaticManifold(manifold, *manifold.pIncident);
									else
									{
										manifold.flipAndSwap();
										solveStaticManifold(manifold, *manifold.pIncident);
									}
									break;
								case CAPSULE_CAPSULE:
									solveDynamicPenetration(manifold, *manifold.pIncident);
									solveStaticManifold(manifold, *manifold.pIncident);
									break;
								case COMPOUND:
									if (!manifold.pIncident)
										continue;
									solveDynamicPenetration(manifold, *manifold.pIncident);
									solveStaticManifold(manifold, *manifold.pIncident);
									break;
								}
								if (manifold.pIncident && manifold.pReference)
								{
									manifold.pIncident->frameManifold.combine(manifold);
									GBManifold refManifold = GBManifold::asReference(manifold);
									manifold.pReference->frameManifold.combine(refManifold);
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


			for (auto& rb : rigidBodies)
			{
				GBBody* body = rb.get();

				if (body->frameManifold.hasGroundedManifold)
				{
					body->isGroundedCount++;
				}

				if (!body->isStatic)
					body->updateTransform(interDeltaTime);

				// combine linear and angular speed
				float speed = body->realVelocity(interDeltaTime).length();
				float angSpeed = body->realAngularVelocity(interDeltaTime).length();


				bool staticBelow = false;
				for (int i = 0; i < body->staticBodies.size(); i++)
				{
					GBBody* other = body->staticBodies[i];
					if (other)
					{
						if (other->transform.position.z < body->transform.position.z)
						{
							staticBelow = true;
							break;
						}
					}
				}
				for (int i = 0; i < body->staticGeometries.size(); i++)
				{
					GBStaticGeometry* pStatic = body->staticGeometries[i];
					if (pStatic)
					{
						if (pStatic->type == GBStaticGeometryType::TRIANGLE)
						{
							GBTriangle* pTri = (GBTriangle*)pStatic;
							GBVector3 upNormal = pTri->normal;
							if (GBDot(upNormal, GBVector3::up()) < 0)
								upNormal *= -1.0f;
							GBVector3 dp = body->transform.position - pTri->vertices[0];
							if (GBDot(dp, upNormal) > 0)
							{
								staticBelow = true;
								break;
							}
						}
					}
				}

				float sleepThresholdFactor = 1.1f;
				if (!body->isKinematic && speed < GBBody::sleepThreshold* sleepThresholdFactor && angSpeed < GBBody::sleepThreshold * sleepThresholdFactor)
				{
					body->sleepTimer += interDeltaTime;

					// Linear damping
					body->velocity *= 0.98f;

					// Angular damping
					body->angularVelocity *= 0.98f;

					if (staticBelow && body->sleepTimer >= GBBody::sleepTime)
					{
						body->isSleeping = true;
						body->velocity = GBVector3::zero();
						body->angularVelocity = GBVector3::zero();
					}
				}
				else
				{
					// moving, reset sleep
					body->isSleeping = false;
					body->sleepTimer = 0.0f;
					body->awakeTimer += interDeltaTime;
				}
			}
			frame++;
		}

		for (auto& [pair, manifold] : curPairManifolds)
		{
			auto it = pairManifolds.find(pair);

			if (it == pairManifolds.end())
			{
				dispatchEnterListeners(pair.a->id, manifold, pair.b);
				dispatchEnterListeners(pair.b->id, manifold, pair.a);
			}
			else
			{
				dispatchStayListeners(pair.a->id, manifold, pair.b);
				dispatchStayListeners(pair.b->id, manifold, pair.a);
			}
		}

		for (auto& [pair, manifold] : pairManifolds)
		{
			if (curPairManifolds.find(pair)==curPairManifolds.end())
			{
				dispatchExitListeners(pair.a->id, pair.b);
				dispatchExitListeners(pair.b->id, pair.a);
			}
		}

		pairManifolds = curPairManifolds;
		curPairManifolds.clear();

		for (auto& rb : rigidBodies)
		{
			GBBody* body = rb.get();
			// Only clear static geometries every step
			body->staticGeometries.clear();
			if (body->isGroundedCount > 0)
			{
				body->isGrounded = true;
			}
		}
	}

	std::vector<GBManifold> generateManifolds()
	{
		std::vector<GBManifold> generatedManifolds;
		for (int i = 0; i < rigidBodies.size() - 1; i++)
		{
			for (int j = i + 1; j < rigidBodies.size(); j++)
			{
				GBBody* bodyA = rigidBodies[i].get();
				GBBody* bodyB = rigidBodies[j].get();

				if (bodyA->isSleeping || bodyB->isSleeping) continue;

				if (bodyA->isDynamic() || bodyB->isDynamic())
				{
					for (int k = 0; k < bodyA->colliders.size(); k++)
					{
						GBCollider* colA = bodyA->colliders[k];
						for (int l = 0; l < bodyB->colliders.size(); l++)
						{
							GBCollider* colB = bodyB->colliders[l];
							GBSATCollisionData data;
							GBManifold manifold;
							if (overlapTest(colA, colB, data, manifold))
							{
								if (manifold.numContacts == 0)
									generateManifold(colA, colB, data, manifold);
								generatedManifolds.push_back(manifold);
							}
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


	const static int maxRecurseDepth = 5;
	void solveDynamicPenetrationRecurse(const GBManifold& manifold, GBBody& root, std::unordered_set<GBBody*>& visited, bool propogate = true)
	{
		if (visited.find(&root) != visited.end()) return; // already moved
		visited.insert(&root);

		if (root.isKinematic)
		{
			root.transform.position += manifold.normal * manifold.separation;
			return;
		}
		else if (root.isDynamic())
		{
			float sepCap = GBClamp(manifold.separation, manifold.separation, 0.05f);
			root.transform.position += manifold.normal * sepCap;
		}



		if (propogate && visited.size() < maxRecurseDepth)
		{
			for (GBBody* child : root.dynamicBodies)
			{
				if (bodyIsSphere(child))
				{
					if (GBDot(manifold.normal, GBVector3::up()) > 0.80f)
					{
						solveDynamicPenetrationRecurse(manifold, *child, visited);
					}
				}
				if (GBDot(manifold.normal, GBVector3::up()) > 0.80f && GBDot(child->velocity, GBVector3::up()) > 0.0f)
				{
					solveDynamicPenetrationRecurse(manifold, *child, visited);

				}
			}
		}
	}

	bool bodyIsSphere(const GBBody* pBody)
	{
		return pBody->colliders[0]->type == ColliderType::Sphere;
	}

	// Wrapper to call recursion
	void solveDynamicPenetration(GBManifold& manifold, GBBody& root, bool propogate = true)
	{
		std::unordered_set<GBBody*> visited;
		if (manifold.separation > maxPenetration)
			manifold.separation = maxPenetration;
		solveDynamicPenetrationRecurse(manifold, root, visited, propogate);
	}

	void solveDynamicPenetrationEqually(GBManifold& manifold)
	{
		bool iDynamic = manifold.pIncident &&!manifold.pIncident->isStatic ? true : false;
		bool rDynamic = manifold.pReference && !manifold.pReference->isStatic ? true : false;
		if (iDynamic && rDynamic)
		{
			manifold.pReference->transform.position -= manifold.separation * manifold.normal * 0.5f;
			manifold.pIncident->transform.position += manifold.separation * manifold.normal * 0.5f;
		}
		else if (iDynamic && !manifold.pIncident->isStatic)
		{
			manifold.pIncident->transform.position += manifold.separation * manifold.normal;
		}
		else if (rDynamic)
		{
			manifold.pReference->transform.position -= manifold.separation * manifold.normal;

		}
	}
};