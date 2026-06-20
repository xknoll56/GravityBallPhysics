#include "GBInclude.h"


#include <typeinfo>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <cassert>

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

struct GBSpringJoint
{
	GBBody* A = nullptr;
	GBBody* B = nullptr;

	GBVector3 localAnchorA;
	GBVector3 localAnchorB;

	float restLength = 0.0f;

	float stiffness = 300.0f;   // k
	float damping = 20.0f;      // c

	GBSpringJoint(GBBody* A, GBBody* B, GBVector3 worldLocalA) :
		A(A), B(B)
	{
		localAnchorA = A->transform.inverse().transformDirection(worldLocalA);
		GBVector3 relPos = B->transform.position - A->transform.position;
		localAnchorB = B->transform.inverse().transformDirection(worldLocalA - relPos);
	}

	GBVector3 accumulatedImpulse; // optional warm starting
};

struct GBBallJoint
{
	GBBody* A = nullptr;
	GBBody* B = nullptr;

	GBVector3 localAnchorA;
	GBVector3 localAnchorB;


	GBBallJoint(GBBody* A, GBBody* B, GBVector3 worldLocalA) :
		A(A), B(B)
	{
		localAnchorA = A->transform.inverse().transformDirection(worldLocalA);
		GBVector3 relPos = B->transform.position - A->transform.position;
		localAnchorB = B->transform.inverse().transformDirection(worldLocalA - relPos);
	}

	GBVector3 accumulatedImpulse; // optional warm starting
};

struct GBSimulation
{
	//GBGrid grid;
	GBGridMap gridMap;
	std::vector<GBCollider*> colliders;
	std::vector<std::unique_ptr<GBSphereCollider>> sphereColliders;
	std::vector<std::unique_ptr<GBBoxCollider>> boxColliders;
	std::vector<std::unique_ptr<GBCapsuleCollider>> capsuleColliders;
	std::vector<std::unique_ptr<GBSpringJoint>> springJoints;
	std::vector<std::unique_ptr<GBBallJoint>> ballJoints;
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
		pBody->setInertia(pBody->aabb.halfExtents);
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

	GBSpringJoint* createSpringJoint(GBBody* a, GBBody* b, GBVector3 localA)
	{
		springJoints.push_back(std::make_unique<GBSpringJoint>(a, b, localA));
		GBSpringJoint* pSpring = springJoints.back().get();
		BodyPair pair(a, b);
		pairSprings[pair] = pSpring;
		return pSpring;
	}

	GBBallJoint* createBallJoint(GBBody* a, GBBody* b, GBVector3 localA)
	{
		ballJoints.push_back(std::make_unique<GBBallJoint>(a, b, localA));
		GBBallJoint* pBall = ballJoints.back().get();
		BodyPair pair(a, b);
		pairBalls[pair] = pBall;
		return pBall;
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
		pBody->setInertia(pBody->aabb.halfExtents);
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
		pBody->setInertia(pBody->aabb.halfExtents);
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

	int solverIterations = 10;   // 6–12 typical
	static constexpr float maxDeltaTime = 1.0f / 30.0f;
	float timeScale = 1.0f;

	void init()
	{
		// Update all colliders
		for (int i = 0; i < rigidBodies.size(); i++)
		{
			rigidBodies[i].get()->updateColliders();
		}


		// Set surrounding bodies and wake if no attachments
		for (int i = 0; i < rigidBodies.size(); i++)
		{
			GBBody* pBody = rigidBodies[i].get();
			for (int j = 0; j < pBody->colliders.size(); j++)
			{
				GBCollider* pCol = pBody->colliders[j];
				std::vector<GBCollider*> surCols;
				gridMap.sampleColliders(pCol->aabb, surCols);

				if (surCols.size() == 0)
					pCol->pBody->isSleeping = false;

				for (int k = 0; k < surCols.size(); k++)
				{
					GBCollider* pOther = surCols[k];
					if (!pOther->pBody->isTrigger)
					{
						GBManifold manifold;
						if (generateColliderManifold(pCol, pOther, manifold))
						{
							if (manifold.pIncident)
							{
								manifold.pIncident->frameManifold.combineSupports(manifold, true);
							}
							if (manifold.pReference)
							{
								GBManifold refManifold = GBManifold::asReference(manifold);
								manifold.pReference->frameManifold.combineSupports(refManifold, true);
							}
						}
					}
				}

			}
		}
	}

	void solveDynamicManifold(const GBManifold& m, float dt, bool canTreatAsStatic = true)
	{
		assert(m.pIncident != nullptr);
		assert(m.pReference != nullptr);

		const float restitution = 0.05f;
		const float percent = 0.3f;
		const float forceCap = 150.0f;
		const float slopeRequirement = 0.9f;
		const float staticManifoldThreshold = 0.05f;

		int count = m.numContacts;


		for (int i = 0; i < count; i++)
		{
			const GBContact& c = m.contacts[i];
			GBVector3 n = c.normal;

			GBBody& A = *c.pIncident->pBody;
			GBBody& B = *c.pReference->pBody;


			bool ignoreA = A.isStatic || A.isKinematic;
			bool ignoreB = B.isStatic || B.isKinematic;

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
					{
						solveStaticManifold(m, dt);
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

			const float baumgarteBeta = 0.2f;      // 0.1 - 0.3 typical
			float penetration = c.distance; // positive penetration depth

			float bias = 0.0f;

			if (penetration > slop)
			{
				bias = baumgarteBeta *
					(penetration - slop) /
					dt;
			}

			float j =
				-(1.0f + restitution) * vn
				- bias;

			j /= invMassSum;
			j /= (float)count;

			j = GBClamp(j, -forceCap * (A.mass + B.mass) * dt, 0.0f);

			GBVector3 impulse = n * j;

			if (!ignoreA)
				A.velocity -= impulse * A.invMass;
			if (!ignoreB)
				B.velocity += impulse * B.invMass;

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
				A.angularVelocity -= GBCross(rA, impulse) * A.invInertia;
			if (!ignoreB)
				B.angularVelocity += GBCross(rB, impulse) * B.invInertia;

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
		const float restitutionThreshold = 0.5f; // CHANGE #2

		const GBContact& c = manifold.contacts[0];

		float flatness = GBDot(c.normal, GBVector3::up());
		const static float minFlat = 0.75f;
		bool notRestable = false;
		if (flatness < minFlat)
			notRestable = true;

		GBVector3 n = c.normal;
		GBVector3 r = c.position - body.transform.position;

		// use contact velocity
		GBVector3 vRel = body.velocity + GBCross(body.angularVelocity, r);

		float vn = GBDot(vRel, n);

		// --- Collision impulse ---
		if (vn < -restingThreshold || notRestable)
		{
			if (GBAbs(vn) > wakeThreshold && manifold.pReference && manifold.pReference->isMovable())
				manifold.pReference->wakeIsland();

			// restitution threshold instead of min bounce impulse
			float restitutionUsed =
				(GBAbs(vn) > restitutionThreshold)
				? restitution
				: 0.0f;

			float jn = -(1.0f + restitutionUsed) * vn;

			body.velocity += n * jn;
			body.angularVelocity += body.invInertia * GBCross(r, n * jn);

			return;
		}

		// --- Rolling without slip ---
		if (fabs(vn) < rollingThreshold)
		{
			GBVector3 vTangent = vRel - n * vn;
			float vTangentLen = vTangent.length();

			{
				GBVector3 omegaDesired =
					GBCross(r, -vTangent) / (r.lengthSquared());

				// blend instead of snap
				float blend = 10.0f * dt;
				blend = GBMin(blend, 1.0f);

				body.angularVelocity =
					body.angularVelocity * (1.0f - blend) +
					omegaDesired * blend;
			}

			float frictionCoeff = body.dynamicFriction;
			body.velocity -= vTangent * frictionCoeff * dt;

			body.frameManifold.hasGroundedManifold = true;
		}
	}


	void solveStaticManifold(
		const GBManifold& manifold,
		float dt)
	{
		assert(manifold.pIncident != nullptr);

		GBBody& body = *manifold.pIncident;
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

		if (bodyIsPureColliderType(*manifold.pIncident, ColliderType::Sphere))
		{
			solveStaticSphereManifold(manifold, *manifold.pIncident, dt);
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

			if (GBAbs(vn) > wakeThreshold && manifold.pReference && manifold.pReference->isMovable())
				manifold.pReference->wakeIsland();

			// --- Effective mass ---
			GBVector3 rn = GBCross(r, n);
			float invMassEff =
				body.invMass +
				GBDot(rn, body.invInertia * rn);

			if (invMassEff < 1e-8f)
				continue;

			const float baumgarteBeta = 0.2f;      // 0.1 - 0.3 typical
			float penetration = c.distance; // positive penetration depth

			float bias = 0.0f;

			if (penetration > slop)
			{
				bias = baumgarteBeta *
					(penetration - slop) /
					dt;
			}

			float jn =
				-(1.0f + restitution) * vn
				- bias;
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

	void solveSpringJoint(GBSpringJoint& j, float dt)
	{
		GBBody& A = *j.A;
		GBBody& B = *j.B;

		if (!A.isMovable() && !B.isMovable())
			return;

		GBVector3 pa = A.transform.localToWorldPoint(j.localAnchorA);
		GBVector3 pb = B.transform.localToWorldPoint(j.localAnchorB);

		GBVector3 rA = pa - A.transform.position;
		GBVector3 rB = pb - B.transform.position;

		GBVector3 vA = A.velocity + GBCross(A.angularVelocity, rA);
		if (!A.isMovable())
			vA = GBVector3();

		GBVector3 vB = B.velocity + GBCross(B.angularVelocity, rB);
		if (!B.isMovable())
			vB = GBVector3();

		GBVector3 relVel = vB - vA;

		GBVector3 delta = pb - pa;

		float dist = delta.length();
		if (dist < 1e-6f)
			return;

		GBVector3 n = delta / dist;

		// spring extension (x)
		float x = dist - j.restLength;

		// relative velocity along spring
		float vn = GBDot(relVel, n);

		// --- spring force (Hooke + damping) ---
		float force =
			(-j.stiffness * x) -
			(j.damping * vn);

		float invMass =
			A.invMass + B.invMass +
			GBDot(GBCross(rA, n), A.invInertia * GBCross(rA, n)) +
			GBDot(GBCross(rB, n), B.invInertia * GBCross(rB, n));

		// convert to impulse
		float jImpulse = force * dt;

		jImpulse /= invMass;

		GBVector3 impulse = n * jImpulse;

		// optional clamp for stability (VERY recommended in your engine)
		const float maxImpulse = 500.0f;
		float len = impulse.length();
		if (len > maxImpulse)
			impulse *= (maxImpulse / len);

		// apply linear
		if (A.isMovable())
		{
			A.velocity -= impulse * A.invMass;

			A.angularVelocity -=
				A.invInertia * GBCross(rA, impulse);
		}

		if (B.isMovable())
		{
			B.velocity += impulse * B.invMass;

			B.angularVelocity +=
				B.invInertia * GBCross(rB, impulse);
		}

	}
	void solveBallJoint(GBBallJoint& j, float dt)
	{
		GBBody& A = *j.A;
		GBBody& B = *j.B;

		if (!A.isMovable() && !B.isMovable())
			return;

		GBVector3 rA = A.transform.localToWorldPoint(j.localAnchorA) - A.transform.position;
		GBVector3 rB = B.transform.localToWorldPoint(j.localAnchorB) - B.transform.position;

		GBVector3 vA = A.velocity + GBCross(A.angularVelocity, rA);
		GBVector3 vB = B.velocity + GBCross(B.angularVelocity, rB);

		if (A.isKinematic)
			vA = A.velocity + GBCross(A.angularVelocity, rA);

		if (B.isKinematic)
			vB = B.velocity + GBCross(B.angularVelocity, rB);

		GBVector3 Cdot = vA - vB;

		// position error (same idea as your contact solver bias)
		GBVector3 error =
			(A.transform.position + rA) -
			(B.transform.position + rB);

		const float beta = 0.2f;
		GBVector3 bias = (error * beta) / dt;

		const GBVector3 axes[3] =
		{
			GBVector3(1,0,0),
			GBVector3(0,1,0),
			GBVector3(0,0,1)
		};

		GBVector3 impulse(0, 0, 0);

		float wA = A.invMass;
		float wB = B.invMass;

		for (int i = 0; i < 3; i++)
		{
			GBVector3 n = axes[i];

			float Cdot_i = GBDot(Cdot, n);
			float bias_i = GBDot(bias, n);

			GBVector3 rnA = GBCross(rA, n);
			GBVector3 rnB = GBCross(rB, n);

			float k =
				wA + wB +
				GBDot(rnA, A.invInertia * rnA) +
				GBDot(rnB, B.invInertia * rnB);

			if (k == 0.0f)
				continue;

			float lambda = -(Cdot_i + bias_i) / k;

			// IMPORTANT: same style as your contact solver (stability)
			lambda = GBClamp(lambda, -50.0f, 50.0f);

			impulse += n * lambda;
		}

		// APPLY (identical structure to your contact solver)
		if (A.isMovable())
			A.velocity += impulse * A.invMass;

		if (B.isMovable())
			B.velocity -= impulse * B.invMass;

		if (A.isMovable())
			A.angularVelocity += A.invInertia * GBCross(rA, impulse);

		if (B.isMovable())
			B.angularVelocity -= B.invInertia * GBCross(rB, impulse);
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
			outManifold.flipAndSwap();
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
			outManifold.flipAndSwap();
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
			outManifold.flipAndSwap();
		}
		break;
		}

		return outManifold.numContacts > 0;
	}

	bool generateColliderManifold(GBCollider* col, GBCollider* pOther, GBManifold& outManifold)
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
				generateBoxManifold((GBBoxCollider*)col, pOther, outManifold.data, outManifold);
				break;
			case ColliderType::Capsule:
				generateCapsuleManifold((GBCapsuleCollider*)col, pOther, outManifold);
				break;
			}
		}

		return outManifold.numContacts > 0;
	}

	bool generateBodySupportManifold(GBBody& body, GBManifold& outManifold, float tolerance = slop)
	{
		outManifold.clear();
		std::vector<GBCollider*> cols;
		GBAABB sampleaabb = body.aabb;
		sampleaabb.grow(slop);
		gridMap.sampleColliders(sampleaabb, cols);
		for (GBCollider* pCol : body.colliders)
		{
			for (GBCollider* pOtherCol : cols)
			{
				GBSATCollisionData data;
				GBManifold manifold;
				if (generateColliderManifold(pCol, pOtherCol, manifold))
				{
					if (manifold.pReference && pCol->pBody == manifold.pReference)
						manifold.flipAndSwap();
					outManifold.combineSupports(manifold);
					body.addDynamicContact(pOtherCol->pBody);
				}
			}
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
							solveDynamicPenetration(manifold);
							solveStaticManifold(manifold, deltaTime);
							pBox->pBody->frameManifold.combineSupports(manifold, true, slop);
						}
					}
					else if (pCapsule)
					{
						if (manifold.numContacts > 0)
						{
							solveDynamicPenetration(manifold);
							solveStaticManifold(manifold, deltaTime);
							pCapsule->pBody->frameManifold.combineSupports(manifold, true, slop);
						}
					}
					else if (pSphere)
					{
						if (manifold.numContacts > 0)
						{
							solveDynamicPenetration(manifold);
							solveStaticManifold(manifold,  deltaTime);
							pSphere->pBody->frameManifold.combineSupports(manifold, true, slop);
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

	void fixManifold(GBManifold& manifold)
	{
		assert(manifold.pIncident != nullptr);

		manifold.capContacts();

		manifold.flipAndSwapIfContactOnTop(manifold.normal);

		BodyPair pair(manifold.pIncident, manifold.pReference);

		auto it = pairSprings.find(pair);
		if (it != pairSprings.end())
		{
			GBSpringJoint* pSpring = it->second;

			if (pSpring->A == manifold.pReference)
			{
				manifold.flipAndSwap();
			}
		}
	}


	GBVector3 gravity = GBVector3(0, 0, -10.0f);
	uint64_t frame = 0;
	static constexpr float maxPenetration = 0.05f;
	static constexpr float wakeThreshold = 0.3f;
    static constexpr float sleepThreshold = 0.080f; // linear+angular speed below which sleep is considered
	static constexpr float sleepTime = 0.55f;       // time to accumulate before sleeping
	static constexpr float sleepDampingRatio = 0.5f;  // The awake time needed out of the sleep timer that heavier contact based damping occurs 
	static constexpr float slop = 0.005f;
	float ellapsedTime = 0.0f;
	int integerTime = 0;
	std::unordered_map<ColliderPair, GBManifold, ColliderPairHash> pairManifolds;
	std::unordered_map<BodyPair, GBSpringJoint*, BodyPairHash> pairSprings;
	std::unordered_map<BodyPair, GBBallJoint*, BodyPairHash> pairBalls;
	std::vector<GBManifold> manifoldStack;


	int floatingCheck = 0;

	// ------------------------------------------------------------
	// SIMULATION STEP
	// ------------------------------------------------------------
	void step(float deltaTime)
	{

		if (deltaTime > maxDeltaTime)
			deltaTime = maxDeltaTime;
		deltaTime = timeScale * deltaTime;
		float interDeltaTime = deltaTime / (float)solverIterations;

		ellapsedTime += deltaTime;
		integerTime = (int)ellapsedTime;

		//We only need to reset is grounded every frame
		for (auto& rb : rigidBodies)
		{
			GBBody* body = rb.get();

			if (!body->isSleeping)
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


			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			// SPRING SOLVER
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			for (auto& springIt : springJoints)
			{
				// Not much to do here but run the solver, has to go first so penetration solvers 
				// Correct any interpenetration

				GBSpringJoint* pSpring = springIt.get();
				solveSpringJoint(*pSpring, interDeltaTime);
			}

			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			// DYNAMIC SOLVER
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			std::vector<GBBody*> sortedBodies = sortBodiesByHeight(rigidBodies);
			std::unordered_set<ColliderPair, ColliderPairHash> dynamicPairs;
			for (int i = 0; i < sortedBodies.size(); i++)
			{
				GBBody* bodyA = sortedBodies[i];
				if (bodyA->ignoreSample || bodyA->isStatic || bodyA->isSleeping || bodyA->isTrigger)
					continue;

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
						if (!dynamicPairs.insert(pair).second)
							continue;


						if (!bodyA->isStatic && !bodyB->isStatic)
						{
							// Skip triggers and handle in static solver
							if (bodyB->isTrigger)
								continue;

							GBManifold manifold;
							if (generateColliderManifold(colA, colB, manifold))
							{
								fixManifold(manifold);

								curPairManifolds[pair] = manifold;

								solveDynamicPenetration(manifold);

								solveDynamicManifold(manifold, interDeltaTime, !manifold.pIncident->isKinematic);

								bool supportAdded = false;
								if (manifold.pIncident)
								{
									supportAdded = manifold.pIncident->frameManifold.combineSupports(manifold, true, slop);
								}
								if (manifold.pReference)
								{
									GBManifold refManifold = GBManifold::asReference(manifold);
									if (manifold.pReference->frameManifold.combineSupports(refManifold, true, slop))
										supportAdded = true;
								}
								if (supportAdded && manifold.pIncident && manifold.pReference)
								{
									manifold.pIncident->addDynamicContact(manifold.pReference);
									manifold.pReference->addDynamicContact(manifold.pIncident);
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

						GBManifold manifold;
						if (generateColliderManifold(colA, colB, manifold))
						{
							if (manifold.numContacts == 0)
								continue;

							if (!bodyA->isMovable() || !bodyB->isMovable())
							{

								fixManifold(manifold);

								curPairManifolds[ColliderPair(colA, colB)] = manifold;

								// If B is a trigger, just add manifold for callback and don't resolve
								if (bodyB->isTrigger)
									continue;

								solveDynamicPenetration(manifold);

								solveStaticManifold(manifold, interDeltaTime);

								bool supportAdded = false;
								if (manifold.pIncident)
								{
									supportAdded = manifold.pIncident->frameManifold.combineSupports(manifold, true, slop);
								}
								if (manifold.pReference)
								{
									GBManifold refManifold = GBManifold::asReference(manifold);
									if (manifold.pReference->frameManifold.combineSupports(refManifold, true, slop))
										supportAdded = true;
								}
								if (supportAdded && manifold.pIncident && manifold.pReference)
								{
									manifold.pIncident->addDynamicContact(manifold.pReference);
									manifold.pReference->addDynamicContact(manifold.pIncident);
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
			// BALL JOINT SOLVER
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			//*****************************************************************************************************
			for (auto& ballIt : ballJoints)
			{
				// Not much to do here but run the solver, has to go first so penetration solvers 
				// Correct any interpenetration

				GBBallJoint* pBall = ballIt.get();
				solveBallJoint(*pBall, interDeltaTime);
			}


			for (auto& rb : rigidBodies)
			{
				GBBody* body = rb.get();

				if (body->isAwake())
					body->updateTransform(interDeltaTime);


				float speed = body->velocity.length();
				float angSpeed = body->angularVelocity.length();

				body->awakeTimer += interDeltaTime;

				bool hasStatic = body->hasStaticAttachment();

				if (hasStatic)
				{
					const static float defaultDamping = 0.998f;
					body->velocity *= defaultDamping;
					body->angularVelocity *= defaultDamping;
				}


				if (!body->isKinematic && speed < sleepThreshold
					&& angSpeed < sleepThreshold)
				{
					body->sleepTimer += interDeltaTime;

					bool manifoldSleepCond = body->frameManifold.containsSupportOrigin(body->transform.position, GBLargeEpsilon);
					if (!body->isSleeping && body->sleepTimer >= sleepTime && hasStatic && manifoldSleepCond)
					{
						body->isSleeping = true;
						body->velocity = GBVector3::zero();
						body->angularVelocity = GBVector3::zero();
					}
				}
				else if (speed > wakeThreshold && angSpeed > wakeThreshold)
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
			if (curPairManifolds.find(pair) == curPairManifolds.end())
			{
				dispatchExitListeners(pair.a->pBody->id, pair.b->pBody);
				dispatchExitListeners(pair.b->pBody->id, pair.a->pBody);

				// Remove all contacts from the frame manifold held by each other
				pair.a->pBody->frameManifold.removeCollidersContact(pair.b);
				pair.b->pBody->frameManifold.removeCollidersContact(pair.a);

			}
		}

		pairManifolds = curPairManifolds;
		curPairManifolds.clear();


		for (auto& rb : rigidBodies)
		{
			GBBody* body = rb.get();

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
			if (body->isAwake())
			{
				if (body->frameManifold.numContacts < 1)
					body->wakeIsland();
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


	std::vector<GBManifold> generateBodiesManifolds(std::vector<GBBody*> bodies, BroadPhaseType sampleType = BroadPhaseType::NONE)
	{
		std::vector<GBManifold> generatedManifolds;
		for (GBBody* bodyA : bodies)
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
						GBManifold manifold;
						if (generateColliderManifold(colA, colB, manifold))
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
	void solveDynamicPenetration(GBManifold& manifold)
	{
		const float percent = 1.0f - (1.0f / solverIterations);
		float penetration = manifold.separation - slop;
		float adjustedSeparation =
			GBMin(GBMax(penetration, 0.0f), maxPenetration);

		GBVector3 upNormal = GBDot(GBVector3::up(), manifold.normal) * manifold.normal;
		GBVector3 horizontalNormal = manifold.normal - upNormal;

		if (manifold.pIncident->isMovable())
		{

			manifold.pIncident->transform.position +=
				upNormal * adjustedSeparation * percent;
		}

		if (manifold.pReference && manifold.pReference->isAwake())
		{

			manifold.pIncident->transform.position +=
				horizontalNormal * adjustedSeparation * percent * 0.5f;

			manifold.pReference->transform.position -=
				horizontalNormal * adjustedSeparation * percent * 0.5f;
		}
		else
		{
			manifold.pIncident->transform.position +=
				horizontalNormal * adjustedSeparation * percent;
		}
	}

	void solveDynamicPenetrationEqually(GBManifold& manifold)
	{
		bool iDynamic = manifold.pIncident && manifold.pIncident->isMovable() ? true : false;
		bool rDynamic = manifold.pReference && manifold.pReference->isMovable() ? true : false;
		float adjustedSeparation = GBMin(manifold.separation * (1.0f - slop), maxPenetration);
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