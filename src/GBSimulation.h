#include "GBInclude.h"


#include <typeinfo>
#include <algorithm>
#include <cstdint>

struct GBSimulation
{
	//GBGrid grid;
	GBGridMap gridMap;
	std::vector<GBCollider*> colliders;
	std::vector<std::unique_ptr<GBSphereCollider>> sphereColliders;
	std::vector<std::unique_ptr<GBBoxCollider>> boxColliders;
	std::vector<std::unique_ptr<GBBody>> rigidBodies;
	std::vector< std::unique_ptr<GBQuad>> quads;
	std::vector< std::unique_ptr<GBTriangle>> triangles;
	GBBoxCollider* pStaticBox;
	uint32_t idCount;

	//: gridMap(GBGridMap(GBVector3(-50,-50, -25), 1.0f, 100, 100, 50, 1, 1, 1))
	GBSimulation()
		: gridMap()
	{
		idCount = 0;
		setupStaticBox();
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

	GBTriangle* createTriangle(GBVector3 a, GBVector3 b, GBVector3 c)
	{
		triangles.push_back(std::make_unique<GBTriangle>(a, b, c));
		GBTriangle* pTriangle = triangles.back().get();
		pTriangle->id = getId();
		GBStaticGeometry* pStatic = (GBStaticGeometry*)pTriangle;
		gridMap.insertStaticGeometry(*pStatic);
		return pTriangle;
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

		pBody->resetContactConstraints();
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
		if(insertToGrid)
			gridMap.insertBody(*pBody);
		return col;
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
		if(insertToGrid)
			gridMap.insertBody(*pBody);
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

	int solverIterations = 4;   // 6–12 typical
	static constexpr float maxDeltaTime = 1.0f / 60.0f;
	float timeScale = 1.0f;

	void solveStaticManifold(const GBManifold& manifold, GBBody& body, float dt, bool otherIsDynamic = false)
	{

		const float restitution = 0.25f;
		const static float normalRestingThreshold = 4.0f;
		const static float tangentialRestingThreshold = 2.0f;
		const float frictionModifier = 0.5f;

		float sizeScale = 1.0f;
		//if (GBBoxCollider* box = (GBBoxCollider*)body.colliders[0])
		//	sizeScale = box->halfExtents.length();

		for (int i = 0; i < manifold.numContacts; i++)
		{
			//const GBContact& c = manifold.collapsed();
			const GBContact& c = manifold.contacts[i];
			GBVector3 n = c.normal;                  // contact normal
			GBVector3 r = c.position - body.transform.position; // contact relative position
			GBVector3 vRel = body.velocity + GBCross(body.angularVelocity, r); // contact velocity

			float vn = GBDot(vRel, n);

			// --- Normal impulse ---
			float invMassEff = body.invMass + GBDot(GBCross(r, n), body.invInertia * GBCross(r, n));
			float jn = -(1.0f + restitution) * vn / invMassEff;

			//if(otherIsDynamic)
			jn = GBClamp(jn, 0.0f, 20.0f * body.mass * dt);

			// --- Tangent (friction) impulse ---
			GBVector3 t = vRel - n * vn;
			float vtLen = t.length();
			if (vtLen > 1e-6f)
				t /= vtLen;
			else
				t = GBVector3::zero();

			float vt = GBDot(vRel, t);
			float invMassT = body.invMass + GBDot(GBCross(r, t), body.invInertia * GBCross(r, t));
			float jt = -vt / invMassT;

			//bool ignoreFriction = false;
			if (vn > -normalRestingThreshold && vtLen < tangentialRestingThreshold)
			{
				//if (!manifold.pReference->pBody->isDynamic())
				{
					if (GBDot(manifold.normal, GBVector3::up()) < 0.80f)
					{
						body.resetConstraints();
					}
					//if (!otherIsDynamic && body.constraintType == GBBody::ConstraintType::NONE &&
					if (body.constraintType == GBBody::ConstraintType::NONE)
					{
						GBBody* pOther = manifold.pReference && manifold.pReference->pBody ? manifold.pReference->pBody : nullptr;
						if (manifold.numContacts > 0)
							body.setConstraintEdge(manifold.contacts[0].position, manifold.numContacts > 1?manifold.contacts[1].position :
								manifold.contacts[0].position, pOther);
						//	body.setConstraintPoint(manifold.contacts[0].position, pOther);
						//else
						
					}
				}
			}
			else
			{
				if(manifold.pReference && manifold.pReference->pBody)
					manifold.pReference->pBody->wake();
				if(manifold.pIncident && manifold.pIncident->pBody)
					manifold.pIncident->pBody->wake();
			}


			// Apply normal impulse
			//if(!ignoreFriction)
			body.velocity += n * (jn * body.invMass);
			body.angularVelocity += body.invInertia * GBCross(r, n * jn);

			// Coulomb friction clamp
			float effectiveImpulse = std::max(jn, normalRestingThreshold);
			float maxFriction = body.dynamicFriction * effectiveImpulse * frictionModifier;
			jt = GBClamp(jt, -maxFriction, maxFriction);


			bool ignoreFriction = false;
			if (GBDot(manifold.normal, GBVector3::up()) < 0.80f || body.awakeTimer < 0.1f)
			{
				//ignoreFriction = true;
				jt *= 0.15f;

			}
			if (!ignoreFriction)
			{
				// Apply friction impulse
				body.velocity += t * (jt * body.invMass);
				body.angularVelocity -= body.invInertia * GBCross(r, t * jt);
			}
		}

		// Optional: cap angular velocity
		const float maxAngVel = 50.0f;
		body.angularVelocity = body.angularVelocity.clamped(maxAngVel);
	}



	void solveDynamicManifold(const GBManifold& m, GBBody& A, GBBody& B, float dt, bool canTreatAsStatic = true)
	{
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
			if (m.pIncident && m.pIncident->type == ColliderType::Sphere && GBAbs(vRelN) < staticManifoldThreshold)
			{
				if(!m.pReference->pBody->isOnLayer(LAYER_STATIC_DYNAMIC_SPHERE) || upness>0.95)
					solveStaticSphereManifold(m, *m.pIncident->pBody, dt);
				//return;
			}
			else if (GBAbs(vRelN) < staticManifoldThreshold &&m.numContacts > 1 && GBDot(GBVector3::up(), m.normal) > slopeRequirement )
			{
				if(upness>0.95)
					solveStaticManifold(m, *m.pIncident->pBody, dt, true);
				else if(m.pIncident->pBody->awakeTimer >0.5)
					solveStaticManifold(m, *m.pIncident->pBody, dt, true);
				//return;
			}
		}

		bool ignoreA = !A.isDynamic();
		bool ignoreB = !B.isDynamic();
		float aModifier = 1.0f;
		float bModifier = 1.0f;

		if (m.pIncident && m.pIncident->type == ColliderType::Sphere)
		{
			if (m.pReference)
			{
				m.pReference->pBody->wakeIsland();
				if (!m.pReference->pBody->isOnLayer(LAYER_STATIC_DYNAMIC_SPHERE))
				{
					bModifier = 0.1f;

					if (GBDot(A.velocity, B.velocity) < 0.0f)
						return;
				}
			}
		}


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
		if (GBDot(A.velocity, B.velocity) < 0)
		{
			A.resetContactConstraints();
			B.resetContactConstraints();
		}
	}

	void solveStaticSphereManifold(const GBManifold& manifold, GBBody& body, float dt)
	{
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
		if (vn < -restingThreshold  || notRestable) // only if approaching
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
			}
			break;

		case ColliderType::Sphere:
			switch (colB->type)
			{
			case ColliderType::Box:   return false;
			case ColliderType::Sphere:return false;
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
		COMPOUND = 3
	};

	bool generateBodyManifold(GBBody* a, GBBody* b, GBSATCollisionData& data, GBManifold& outManifold, SolverType& outType)
	{
		GBSphereCollider* pSphereA = nullptr;
		GBSphereCollider* pSphereB = nullptr;
		GBSphereCollider* pSphere = nullptr;
		GBBoxCollider* pBoxA = nullptr;
		GBBoxCollider* pBoxB = nullptr;
		GBBoxCollider* pBox = nullptr;
		if (a->colliders.size() == 1 && b->colliders.size() == 1)
		{
			if (a->colliders[0]->type == ColliderType::Sphere && b->colliders[0]->type == ColliderType::Sphere)
			{
				pSphereA = (GBSphereCollider*)a->colliders[0];
				pSphereB = (GBSphereCollider*)b->colliders[0];
				outType = SPHERE_SPHERE;
			}
			else if(a->colliders[0]->type == ColliderType::Box && b->colliders[0]->type == ColliderType::Box)
			{
				pBoxA = (GBBoxCollider*)a->colliders[0];
				pBoxB = (GBBoxCollider*)b->colliders[0];
				outType = BOX_BOX;
			}
			else
			{
				if (a->colliders[0]->type == ColliderType::Box)
				{
					pBox = (GBBoxCollider*)a->colliders[0];
					pSphere = (GBSphereCollider*)b->colliders[0];
				}
				else
				{
					pBox = (GBBoxCollider*)b->colliders[0];
					pSphere = (GBSphereCollider*)a->colliders[0];
				}
				outType = BOX_SPHERE;
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
			if(pSphereA && pSphereB)
				doesIntersect = GBManifoldGeneration::GBManifoldSphereSphere(*pSphereA, *pSphereB, outManifold);
			break;
		case BOX_SPHERE:
			if(pSphere && pBox)
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

	//BroadPhaseType broadPhaseType = BroadPhaseType::NONE;
	BroadPhaseType broadPhaseType = BroadPhaseType::UNIFORM_GRID;

	void extractBroadPhaseBodies(int bodyIndex, std::vector<GBBody*>& bodies, std::vector<GBBody*>& outBodies)
	{
		switch (broadPhaseType)
		{
		case BroadPhaseType::NONE:
			for (int i = bodyIndex+1; i < bodies.size(); i++)
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
							manifold.pIncident = (GBCollider*)pSphere;
							manifold.normal = contact.normal;
							manifold.separation = contact.penetrationDepth;
							manifold.clampSeparation(slop);
							pSphere->pBody->frameManifold.combine(manifold);
							solveDynamicPenetration(manifold, *manifold.pIncident->pBody, false);
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
								solveDynamicPenetration(manifold, *manifold.pIncident->pBody);
								solveStaticManifold(manifold, *manifold.pIncident->pBody, deltaTime);
								handleConstraintAction(manifold);
							}
						}
					}
					break;
				}
			}
		}

		// Now handle the sampled static geometry
		std::vector<GBStaticGeometry*> sampledStaticGeometry;
		gridMap.sampleStaticGeometry(body.aabb, sampledStaticGeometry);

		pBox = nullptr;
		pSphere = nullptr;

		GBManifold combinedManifold;
		combinedManifold.separation = 0.0f;

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
								manifold.pIncident = (GBCollider*)pSphere;
								manifold.normal = contact.normal;
								manifold.separation = contact.penetrationDepth;
								manifold.clampSeparation(manifold.separation - slop);
								pSphere->pBody->frameManifold.combine(manifold);
								solveDynamicPenetration(manifold, *manifold.pIncident->pBody, false);
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
				solveDynamicPenetration(combinedManifold, *combinedManifold.pIncident->pBody);
				solveStaticManifold(combinedManifold, *combinedManifold.pIncident->pBody, deltaTime);
				handleConstraintAction(combinedManifold);
			}
		}
	}


	GBVector3 gravity = GBVector3(0, 0, -10.0f);
	uint64_t frame = 0;
	static constexpr float maxPenetration = 0.1f;
	const float slop = 0.05f;
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

				body->addForce(gravity * body->mass); // gravity
				body->update(interDeltaTime);
				body->clearContactedBodies();
			}


			std::vector<GBBody*> sortedBodies = sortBodiesByHeight(rigidBodies);
			std::vector<std::pair<GBBody*, GBBody*>> handledPairs;
			// Do all the boxes now
			for (int i = 0; i < sortedBodies.size(); i++)
			{
				GBBody* bodyA = sortedBodies[i];
				if (bodyA->ignoreSample || bodyA->isStatic || bodyA->isSleeping)
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
					if (!containsPair(handledPairs, bodyA, bodyB))
						handledPairs.push_back(bodyPair);
					else
						continue;

					//if (bodyA->isSleeping || bodyB->isSleeping) continue;

					//if (!bodyA->isStatic || !bodyB->isStatic)
					if (bodyA->isDynamic() || bodyB->isDynamic())
					{
						GBSATCollisionData data;
						GBManifold manifold;
						SolverType type;
						if (generateBodyManifold(bodyA, bodyB, data, manifold, type))
						{
							if (manifold.numContacts == 0)
								continue;
							switch (type)
							{
							case SPHERE_SPHERE:
								solveDynamicPenetrationEqually(manifold);
								solveDynamicManifold(manifold, *manifold.pReference->pBody, *manifold.pIncident->pBody, interDeltaTime, true);
								break;
							case BOX_SPHERE:
								solveDynamicPenetrationEqually(manifold);
								solveDynamicManifold(manifold, *manifold.pReference->pBody, *manifold.pIncident->pBody, interDeltaTime, true);
								break;
							case BOX_BOX:
							{
								if (!manifold.pReference->pBody->isStatic)
									manifold.flipAndSwapIfContactOnTop(manifold.normal);
								float clampedSep = GBClamp(manifold.separation, manifold.separation, slop);
								manifold.pIncident->pBody->transform.position += manifold.normal * manifold.separation;
								solveDynamicManifold(manifold, *bodyA, *bodyB, interDeltaTime, true);
							}
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
								manifold.pIncident->pBody->frameManifold.combine(manifold);
								GBManifold refManifold = GBManifold::asReference(manifold);
								manifold.pReference->pBody->frameManifold.combine(refManifold);
							}
						}

					}

				}

			}

			handledPairs.clear();
			for (int i = 0; i < sortedBodies.size(); i++)
			{
				GBBody* bodyA = sortedBodies[i];
				if (bodyA->ignoreSample || bodyA->isSleeping)
					continue;
				if (bodyA->isDynamic())
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
					if (!containsPair(handledPairs, bodyA, bodyB))
						handledPairs.push_back(bodyPair);
					else
						continue;
					if (bodyA->isStatic && bodyB->isStatic)
						continue;

					GBSATCollisionData data;
					GBManifold manifold;
					SolverType type;
					if (generateBodyManifold(bodyA, bodyB, data, manifold, type))
					{
						if (manifold.numContacts == 0)
							continue;
						GBBody* dynamicBody, * staticBody;
						//if (bodyA->isStatic || bodyB->isStatic)
						if (!bodyA->isDynamic() || !bodyB->isDynamic())
						{
							if (bodyA->isDynamic())
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
							if (data.minOverlap > 0.005f)
								wakeSleeping = true;
							staticBody->addDynamicContact(dynamicBody, wakeSleeping);
							if (manifold.numContacts > 0)
							{
								switch (type)
								{
								case SPHERE_SPHERE:
									// Don't bother, treat as dynamic
									break;
								case BOX_SPHERE:
									solveDynamicPenetration(manifold, *manifold.pIncident->pBody);
									if (manifold.pReference->pBody->isStatic && manifold.pReference->pBody->isOnLayer(LAYER_STATIC_DYNAMIC_SPHERE))
										solveStaticSphereManifold(manifold, *manifold.pIncident->pBody, interDeltaTime);
									else
										solveDynamicManifold(manifold, *manifold.pReference->pBody, *manifold.pIncident->pBody, interDeltaTime, true);
									break;
								case BOX_BOX:
									if (!manifold.pReference->pBody->isStatic)
										manifold.flipAndSwapIfContactOnTop(manifold.normal);
									solveDynamicPenetration(manifold, *manifold.pIncident->pBody);
									solveStaticManifold(manifold, *manifold.pIncident->pBody, interDeltaTime);
									if (manifold.pIncident->pBody->isDynamic())
										handleConstraintAction(manifold);
									break;
								case COMPOUND:
									solveDynamicPenetration(manifold, *manifold.pIncident->pBody);
									solveStaticManifold(manifold, *manifold.pIncident->pBody, interDeltaTime);
									break;
								}
								if (manifold.pIncident && manifold.pReference)
								{
									manifold.pIncident->pBody->frameManifold.combine(manifold);
									GBManifold refManifold = GBManifold::asReference(manifold);
									manifold.pReference->pBody->frameManifold.combine(refManifold);
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

				// Wake if no static attachments
				//if (body->staticGeometries.size() == 0 && !body->hasStaticAttachment())
				//{
					//body->wakeIsland();
					//body->resetContactConstraints();
					//body->isGrounded = false;
				//}
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

				if (speed < GBBody::sleepThreshold && angSpeed < GBBody::sleepThreshold)
				{
					body->sleepTimer += interDeltaTime;

					if (staticBelow && body->sleepTimer >= GBBody::sleepTime && body->frameManifold.canSleep(body->transform.position))
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
			for(int j = 0; j<triangles.size();j++)
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

	void handleConstraintAction(const GBManifold& manifold)
	{
		if (manifold.pIncident->pBody->constraintType == GBBody::ConstraintType::NONE)
		{
			return;
		}

		GBVector3 r = manifold.contacts[0].position;
		GBVector3 dir = GBCross(-manifold.normal, r.normalized()).normalized();
		GBBody* pOther = manifold.pReference && manifold.pReference->pBody ? manifold.pReference->pBody : nullptr;

		if (manifold.numContacts == 1)
		{
			manifold.pIncident->pBody->setConstraintPoint(manifold.contacts[0].position, pOther);
		}
		else if (manifold.numContacts == 2)
		{
			manifold.pIncident->pBody->setConstraintEdge(manifold.contacts[0].position, manifold.contacts[1].position, pOther);
		}
		else
		{
			manifold.pIncident->pBody->isSleeping = true;
		}
	}

	const static int maxRecurseDepth = 5;
	void solveDynamicPenetrationRecurse(const GBManifold& manifold, GBBody& root, std::unordered_set<GBBody*>& visited, bool propogate = true)
	{
		if (visited.find(&root) != visited.end()) return; // already moved
		visited.insert(&root);

		if (root.isDynamic())
		{
			float sepCap = GBClamp(manifold.separation, manifold.separation, 0.05f);
				root.transform.position += manifold.normal * sepCap;
		}

		if (propogate && visited.size()<maxRecurseDepth)
		{
			for (GBBody* child : root.dynamicBodies)
			{
				if (GBDot(manifold.normal, GBVector3::up()) > 0.80f && GBDot(child->velocity, GBVector3::up()) > 0.0f)
				{
					solveDynamicPenetrationRecurse(manifold, *child, visited);

				}
			}
		}
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
		bool iDynamic = manifold.pIncident && manifold.pIncident->pBody->isDynamic() ? true : false;
		bool rDynamic = manifold.pReference && manifold.pReference->pBody->isDynamic() ? true : false;
		if (iDynamic && rDynamic)
		{
			manifold.pReference->pBody->transform.position -= manifold.separation * manifold.normal * 0.5f;
			manifold.pIncident->pBody->transform.position += manifold.separation * manifold.normal * 0.5f;
		}
		else if (iDynamic && !manifold.pIncident->pBody->isStatic)
		{
			manifold.pIncident->pBody->transform.position += manifold.separation * manifold.normal;
		}
		else if(rDynamic)
		{
			manifold.pReference->pBody->transform.position -= manifold.separation * manifold.normal;

		}
	}
};


// DYNAMIC SOLVER
// 
//for (int i = 0; i < sortedBodies.size(); i++)
//{
//	GBBody* bodyA = sortedBodies[i];
//	if (bodyA->ignoreSample || bodyA->isStatic ||bodyA->isSleeping)
//		continue;
//	bool didColide = false;
//	std::vector<GBBody*> checkBodies;
//	extractBroadPhaseBodies(i, sortedBodies, checkBodies);
//	for (int j = 0; j<checkBodies.size(); j++)
//	{
//		GBBody* bodyB = checkBodies[j];
//		if (bodyA == bodyB || !bodyA->sharesLayer(*bodyB))
//			continue;
//		std::pair<GBBody*, GBBody*> bodyPair;
//		bodyPair.first = bodyA;
//		bodyPair.second = bodyB;
//		if (!containsPair(handledPairs, bodyA, bodyB))
//			handledPairs.push_back(bodyPair);
//		else
//			continue;

//		//if (bodyA->isSleeping || bodyB->isSleeping) continue;

//		//if (!bodyA->isStatic || !bodyB->isStatic)
//		if (bodyA->isDynamic() || bodyB->isDynamic())
//		{
//			for (int k = 0; k < bodyA->colliders.size(); k++)
//			{
//				GBCollider* colA = bodyA->colliders[k];
//				for (int l = 0; l < bodyB->colliders.size(); l++)
//				{
//					GBCollider* colB = bodyB->colliders[l];
//					GBSATCollisionData data;
//					GBManifold manifold;
//					if (overlapTest(colA, colB, data, manifold))
//					{
//						if (manifold.numContacts == 0)
//							generateManifold(colA, colB, data, manifold);
//						if (manifold.numContacts == 0)
//							continue;
//							// Swap if incident is below
//						if (manifold.pIncident->type != ColliderType::Sphere && !manifold.pReference->pBody->isStatic)
//							manifold.flipAndSwapIfContactOnTop(manifold.normal);
//							//manifold.flipAndSwapIfOnTop();
//						bool isBallPhysics = manifold.pIncident->type == ColliderType::Sphere;
//						if (!isBallPhysics)
//						{
//							float clampedSep = GBClamp(manifold.separation, manifold.separation, slop);
//							manifold.pIncident->pBody->transform.position += manifold.normal * manifold.separation;
//						}
//						if (isBallPhysics)
//						{
//							if (manifold.pReference->type == ColliderType::Box)
//							{
//								bool boxDynamic = manifold.pReference->pBody->isDynamic();
//								bool sphereDynamic = manifold.pIncident->pBody->isDynamic();
//								if (boxDynamic && sphereDynamic)
//								{
//									manifold.pIncident->pBody->transform.position += manifold.normal * manifold.separation*0.5f;
//									manifold.pReference->pBody->transform.position -= manifold.normal * manifold.separation*0.5f;
//								}
//								else if (boxDynamic)
//								{
//									manifold.pReference->pBody->transform.position -= manifold.normal * manifold.separation;
//								}
//								else
//								{
//									manifold.pIncident->pBody->transform.position += manifold.normal * manifold.separation;
//								}

//								//if(!manifold.pReference->pBody->isOnLayer(LAYER_STATIC_DYNAMIC_SPHERE))
//									solveDynamicManifold(manifold, *manifold.pReference->pBody, *manifold.pIncident->pBody, interDeltaTime, !isBallPhysics);
//							}
//						}
//						else
//							solveDynamicManifold(manifold, *bodyA, *bodyB, interDeltaTime, !isBallPhysics);
//						bodyA->addDynamicContact(bodyB);
//						bodyB->addDynamicContact(bodyA);
//						manifold.pIncident->pBody->frameManifold.combine(manifold);
//						GBManifold refManifold = GBManifold::asReference(manifold);
//						manifold.pReference->pBody->frameManifold.combine(refManifold);
//					}
//				}
//			}
//		}
//	}

//}


// STATIC SOLVER
//for (int i = 0; i < sortedBodies.size(); i++)
//{
//	GBBody* bodyA = sortedBodies[i];
//	if (bodyA->ignoreSample || bodyA->isSleeping)
//		continue;
//	if (bodyA->isDynamic())
//		handleStaticGeometry(*bodyA, interDeltaTime);

//	std::vector<GBBody*> checkBodies;
//	extractBroadPhaseBodies(i, sortedBodies, checkBodies);
//	for (int j = 0; j < checkBodies.size(); j++)
//	{
//		GBBody* bodyB = checkBodies[j];
//		if (bodyA == bodyB || !bodyA->sharesLayer(*bodyB))
//			continue;
//		std::pair<GBBody*, GBBody*> bodyPair;
//		bodyPair.first = bodyA;
//		bodyPair.second = bodyB;
//		if (!containsPair(handledPairs, bodyA, bodyB))
//			handledPairs.push_back(bodyPair);
//		else
//			continue;
//		if (bodyA->isStatic && bodyB->isStatic)
//			continue;

//		for (int k = 0; k < bodyA->colliders.size(); k++)
//		{
//			GBCollider* colA = bodyA->colliders[k];
//			for (int l = 0; l < bodyB->colliders.size(); l++)
//			{
//				GBCollider* colB = bodyB->colliders[l];
//				GBSATCollisionData data;
//				GBManifold manifold;
//				if (overlapTest(colA, colB, data, manifold))
//				{
//					GBBody* dynamicBody, * staticBody;
//					//if (bodyA->isStatic || bodyB->isStatic)
//					if (!bodyA->isDynamic() || !bodyB->isDynamic())
//					{
//						if (bodyA->isDynamic())
//						{
//							dynamicBody = bodyA;
//							staticBody = bodyB;
//						}
//						else
//						{
//							dynamicBody = bodyB;
//							staticBody = bodyA;
//						}
//						dynamicBody->addStaticContact(staticBody);

//						bool wakeSleeping = false;
//						if (data.minOverlap > 0.005f)
//							wakeSleeping = true;
//						staticBody->addDynamicContact(dynamicBody, wakeSleeping);

//						if (manifold.numContacts == 0)
//							generateManifold(colA, colB, data, manifold);
//						if (manifold.numContacts > 0)
//						{
//							// Swap if incident is below
//							if (manifold.pIncident->type != ColliderType::Sphere && !manifold.pReference->pBody->isStatic)
//								manifold.flipAndSwapIfContactOnTop(manifold.normal);
//							//manifold.flipAndSwapIfOnTop();
//						//manifold.pIncident->pBody->transform.position += manifold.normal * manifold.separation;
//							if (manifold.pIncident->pBody->isSleeping)
//								continue;
//							solveDynamicPenetration(manifold, *manifold.pIncident->pBody);
//							if (manifold.pIncident->type == ColliderType::Sphere)
//							{
//								if (manifold.pReference->pBody->isStatic && manifold.pReference->pBody->isOnLayer(LAYER_STATIC_DYNAMIC_SPHERE))
//									solveStaticSphereManifold(manifold, *manifold.pIncident->pBody, interDeltaTime);
//								else
//									solveDynamicManifold(manifold, *manifold.pReference->pBody, *manifold.pIncident->pBody, interDeltaTime, true);

//							}
//							else
//								solveStaticManifold(manifold, *manifold.pIncident->pBody, interDeltaTime);
//							if (manifold.pIncident->pBody->isDynamic() && manifold.pIncident->type != ColliderType::Sphere)
//								handleConstraintAction(manifold);
//							manifold.pIncident->pBody->frameManifold.combine(manifold);
//							GBManifold refManifold = GBManifold::asReference(manifold);
//							manifold.pReference->pBody->frameManifold.combine(refManifold);
//						}
//					}
//					else
//					{
//						bodyA->addStaticContact(bodyB);
//						bodyB->addStaticContact(bodyA);
//					}
//				}
//			}
//		}
//	}

//}