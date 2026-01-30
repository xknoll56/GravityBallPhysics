#pragma
#include <memory>
#include <vector>
#include <unordered_set>
#include "GBInclude.h"
#include "GBAABB.h"

enum class ColliderType
{
	Sphere,
	Box,
	AABB
};

struct GBBody;
struct GBCell;


struct GBCollider
{

	GBAABB aabb;
	ColliderType type;
	GBTransform transform;
	GBTransform localTransform;
	GBBody* pBody = nullptr;

	GBCollider()
		: aabb(), type(ColliderType::Sphere)
	{
	}
	virtual void updateAABB() = 0;
	virtual ~GBCollider() {}

	void setPosition(const GBVector3& position, bool doUpdateAABB = true)
	{
		transform.setPosition(position);
		if(doUpdateAABB)
			updateAABB();
	}

	void setRotation(const GBQuaternion& rotation, bool doUpdateAABB = true)
	{
		transform.setRotation(rotation);
		if (doUpdateAABB)
			updateAABB();
	}
	void translate(const GBVector3& translation, bool doUpdateAABB = true)
	{
		transform.translate(translation);
		if (doUpdateAABB)
			updateAABB();
	}
	void rotate(const GBQuaternion& rotation, bool doUpdateAABB = true)
	{
		transform.rotate(rotation);
		if (doUpdateAABB)
			updateAABB();
	}

	void setTransform(const GBTransform& newTransform, bool doUpdateAABB = true)
	{
		setPosition(newTransform.position, false);
		setRotation(newTransform.rotation, false);

		if (doUpdateAABB)
			updateAABB();
	}

	virtual float volume() const = 0;

	void* pData = (void*)nullptr;
	size_t id = (size_t)nullptr;
	std::vector<GBCell*> occupiedCells;
	bool isContacted = false;
};

struct GBContact {

	GBVector3 position;
	GBVector3 normal; // From collider A to B
	float penetrationDepth;
	GBCollider* pReference;
	GBCollider* pIncident;
	GBContact()
		: position(0, 0, 0), normal(0, 0, 0), penetrationDepth(0), pReference(nullptr), pIncident(nullptr)
	{
	}
	GBContact(const GBVector3& position, const GBVector3& normal, float penetrationDepth, GBCollider* pReference = nullptr, GBCollider* pIncident = nullptr)
		: position(position), normal(normal), penetrationDepth(penetrationDepth), pReference(pReference), pIncident(pIncident)
	{
	}

	void applyTransformation(const GBTransform transform)
	{
		position = transform.transformPoint(position);
		normal = transform.transformDirection(normal);
	}
};

struct GBManifold
{
	GBContact contacts[4];
	int numContacts = 0;
	float separation = 0.0f;
	GBVector3 normal;
	GBCollider* pIncident = nullptr;
	GBCollider* pReference = nullptr;
	bool isEdge = false;
	bool isDynamicManifold = false;

	GBManifold()
	{

	}

	GBManifold(const GBContact& contact, GBCollider* pIncident = nullptr, GBCollider* pReference = nullptr) :
		pIncident(pIncident), pReference(pReference)
	{
		addContact(contact);
	}
	// --------------------------------------------------
	// Utility helpers
	// --------------------------------------------------

	void clampSeparation(const float max)
	{
		separation = GBClamp(separation, 0.0f, max);
	}

	int findDuplicate(const GBContact& c, float epsilon = 1e-5f) const
	{
		for (int i = 0; i < numContacts; i++)
		{
			if (contacts[i].position.epsilonEqual(c.position, epsilon))
				return i;
		}
		return -1;
	}

	int findShallowest() const
	{
		// shallowest = smallest penetrationDepth
		int index = 0;
		float minDepth = contacts[0].penetrationDepth;

		for (int i = 1; i < numContacts; i++)
		{
			if (contacts[i].penetrationDepth < minDepth)
			{
				minDepth = contacts[i].penetrationDepth;
				index = i;
			}
		}
		return index;
	}

	void sortByDepth()
	{
		// Insertion sort (N <= 4)
		// Order: deepest → shallowest
		for (int i = 1; i < numContacts; i++)
		{
			GBContact key = contacts[i];
			int j = i - 1;

			while (j >= 0 &&
				contacts[j].penetrationDepth < key.penetrationDepth)
			{
				contacts[j + 1] = contacts[j];
				j--;
			}

			contacts[j + 1] = key;
		}
	}

	// --------------------------------------------------
	// Public API
	// --------------------------------------------------

	void clear()
	{
		numContacts = 0;
	}

	void reset()
	{
		numContacts = 0;
		normal = GBVector3();
		separation = 0.0f;
		pIncident = nullptr;
		pReference = nullptr;
		isEdge = false;
		isDynamicManifold = false;
		hasGroundedManifold = false;
	}

	void addContact(const GBContact& contact, bool sorted = true)
	{
		if (!sorted)
		{
			if (numContacts < 4)
				contacts[numContacts++] = contact;
		}
		else
		{
			addContactSortedUnique(contact);
		}
	}

	void addContactSortedUnique(const GBContact& contact)
	{
		// Reject duplicates
		if (findDuplicate(contact) != -1)
			return;

		if (numContacts < 4)
		{
			contacts[numContacts++] = contact;
		}
		else
		{
			// Replace shallowest if new contact is deeper
			int shallowest = findShallowest();

			if (contact.penetrationDepth >
				contacts[shallowest].penetrationDepth)
			{
				contacts[shallowest] = contact;
			}
			else
			{
				return; // worse than all existing contacts
			}
		}

		sortByDepth();
	}

	void applyTransformation(const GBTransform& transform)
	{
		for (int i = 0; i < numContacts; i++)
			contacts[i].applyTransformation(transform);
	}

	void flip()
	{
		normal = -normal;
		for (int i = 0; i < numContacts; i++)
			contacts[i].normal = -contacts[i].normal;
	}

	void flipAndSwap()
	{
		GBCollider* temp = pIncident;
		pIncident = pReference;
		pReference = temp;
		flip();
	}

	void flipAndSwapIfOnTop();
	void flipAndSwapIfContactOnTop(GBVector3 manifoldNormal);

	float size() const
	{
		if (numContacts < 2)
			return 0.0f;

		if (numContacts == 2)
			return (contacts[1].position - contacts[0].position).length();

		// 3 or more contacts → area of first triangle
		GBVector3 A = contacts[0].position;
		GBVector3 B = contacts[1].position;
		GBVector3 C = contacts[2].position;

		// Area = 0.5 * |(B - A) x (C - A)|
		GBVector3 AB = B - A;
		GBVector3 AC = C - A;
		return 0.5f * AB.cross(AC).length();
	}

	bool treatedAsStatic = false;

	GBContact collapsed() const
	{
		GBVector3 avg = GBVector3::zero();
		for (int i = 0; i < numContacts; i++)
		{
			avg += contacts[i].position;
		}
		avg *= (1.0f / (float)numContacts);
		float sep = separation == 0.0f ? contacts[0].penetrationDepth : separation;
		return GBContact(avg, contacts[0].normal, sep, pReference, pIncident);
	}

	void combine(const GBManifold& other)
	{
		for (int i = 0; i < other.numContacts; i++)
		{
			addContact(other.contacts[i]);
		}
	}

	void useNormal(const GBManifold& other)
	{
		normal = other.normal;
		isEdge = other.isEdge;
		separation = other.separation;
		pReference = other.pReference;
		pIncident = other.pIncident;
	}

	bool equalPair(const GBManifold& other) const
	{
		return ((pIncident == other.pIncident) && (pReference == other.pReference)) || ((pIncident == other.pReference) && (pReference == other.pIncident));
	}

	static GBManifold asReference(const GBManifold& other)
	{
		GBManifold manifold;
		manifold.useNormal(other);
		manifold.combine(other);
		manifold.flipAndSwap();
		return manifold;
	}

	bool canSleep(const GBVector3& com) const;

	bool hasGroundedManifold = false;
};

struct GBStaticGeometry;

struct GBBody
{
	// WORLD transform of the body
	GBTransform transform;

	// Colliders
	//GBCollider* pCollider;
	std::vector<GBCollider*> colliders;

	// Contacted bodies
	std::vector<GBBody*> dynamicBodies;
	std::vector<GBBody*> staticBodies;
	std::vector<GBStaticGeometry*> staticGeometries;

	// Physics properties
	float mass = 1.0f;
	float invMass = 1.0f;

	GBVector3 velocity;            // linear velocity
	GBVector3 angularVelocity;     // rotational velocity in local space
	GBVector3 forceAccum;          // accumulated force
	GBVector3 torqueAccum;         // accumulated torque


	// Material properties
	float restitution = 0.3f;    // bounciness, 0 = no bounce, 1 = full bounce
	float staticFriction = 0.5f; // prevents sliding
	float dynamicFriction = 0.3f; // sliding friction

	GBVector3 inertia;        // principal moments of inertia (local space)
	GBVector3 invInertia;     // inverse of inertia

	// Inside BrickBody
	bool isSleeping = false;
	float sleepTimer = 0.0f;

	static constexpr float sleepThreshold = 0.25f; // linear+angular speed below which sleep is considered
	static constexpr float sleepTime = 0.1f;       // time to accumulate before sleeping
	
	bool onAwake = false;
	bool isStatic = false;
	float awakeTimer = 0.0f;
	
	GBVector3 prevPosition;
	GBQuaternion prevRotation;

	GBAABB aabb;

	std::vector<GBCell*> occupiedCells;

	bool isDirty = true;

	bool doesTriggerGridRebuild = true;

	bool ignoreSample = false;
	int id;
	
	bool firstWake = true;
	void* pData = (void*)nullptr;

	GBManifold frameManifold;

	bool isGrounded = false;
	int isGroundedCount = 0;

	struct PointConstraint
	{
		GBBody* pBody = nullptr;
		GBBody* pOtherBody = nullptr;
		GBVector3 worldConstraintPoint;  // fixed world-space point
		GBVector3 localConstraintPoint;  // fixed in body space
		GBVector3 otherLocalConstraintPoint;
		float breakingThreshold = 0.25f;

		void setConstraintPoint(const GBVector3& worldPoint, GBBody* other)
		{
			worldConstraintPoint = worldPoint;
			localConstraintPoint = pBody->transform.inverse().transformPoint(worldPoint);

			if (other && !other->isStatic)
			{
				this->pOtherBody = other;
				otherLocalConstraintPoint = other->transform.inverse().transformPoint(worldPoint);
			}
		}

		void updateConstraintPoint(float dt)
		{
			// Current world position of the constraint point
			GBVector3 currentWorld = pBody->transform.transformPoint(localConstraintPoint);

			if (pOtherBody && !pOtherBody->isStatic)
			{
				GBVector3 newWorldPoint = pOtherBody->transform.transformPoint(otherLocalConstraintPoint);
				GBVector3 deltaWorldPoint = newWorldPoint - worldConstraintPoint;
				if (deltaWorldPoint.lengthSquared() > breakingThreshold)
				{
					pOtherBody = nullptr;
				}
				else
				{
					worldConstraintPoint = newWorldPoint;
				}
			}
			// 1️⃣ Position correction
			GBVector3 correction = worldConstraintPoint - currentWorld;
			pBody->transform.position += correction;

			// 2️⃣ Rotation correction with thresholds to prevent jitter
			GBVector3 r = currentWorld - pBody->transform.position;
			GBVector3 target = worldConstraintPoint - pBody->transform.position;

			if (r.length() > 1e-6f && target.length() > 1e-6f)
			{
				GBVector3 axis = GBCross(r, target);
				float axisLen = axis.length();
				if (axisLen > 1e-6f) // only rotate if meaningful
				{
					axis /= axisLen;
					float angle = acosf(GBClamp(GBDot(r.normalized(), target.normalized()), -1.f, 1.f));

					// optionally interpolate rotation to reduce jitter
					float factor = 0.5f;
					GBQuaternion rotation = GBQuaternion::fromAxisAngle(axis, angle * factor);
					GBQuaternion oldRotation = pBody->transform.rotation;
					pBody->transform.rotate(rotation);

					// 2️⃣ Rotation correction using local-alignment check
					// Transform the world constraint point into the body's local space
					GBVector3 newLocal = pBody->transform.inverse().transformPoint(worldConstraintPoint);

					// Compute alignment between new local vector and original local vector
					float alignment = GBAbs(GBDot(newLocal.normalized(), localConstraintPoint.normalized()));

					const float tolerance = 1e-8f; // small tolerance for alignment
					if (alignment > 1.0f - tolerance) // only rotate if significantly misaligned
					{
						// Revert the change
						pBody->transform.rotation = oldRotation;
					}
				}
			}

		}
	};

	struct EdgeConstraint : PointConstraint
	{
		GBEdge worldEdge;   // fixed edge in world space
		GBEdge otherLocalEdge;

		void setConstraintEdge(const GBEdge& edge, GBBody* other)
		{
			worldEdge = edge;

			worldConstraintPoint = worldEdge.closestPointOnLine(pBody->transform.position);
			setConstraintPoint(worldConstraintPoint, other);

			if(other && !other->isStatic)
			otherLocalEdge = GBEdge(other->transform.inverse().transformPoint(worldEdge.a),
				other->transform.inverse().transformPoint(worldEdge.b));
		}

		void updateConstraint(float dt)
		{
			if (pOtherBody && !pOtherBody->isStatic)
			{
				worldEdge = GBEdge(pOtherBody->transform.transformPoint(otherLocalEdge.a),
					pOtherBody->transform.transformPoint(otherLocalEdge.b));
			}
			GBVector3 edgeDir = worldEdge.getBOutDirection();
			pBody->angularVelocity = edgeDir * GBDot(edgeDir, pBody->angularVelocity);
			updateConstraintPoint(dt);
		}
	};

	enum ConstraintType
	{
		NONE = 0,
		POINT = 1,
		EDGE = 2,
		PLANE = 3
	};

	PointConstraint pointConstraint;
	EdgeConstraint edgeConstraint;
	ConstraintType constraintType = ConstraintType::NONE;

	void setConstraintPoint(GBVector3 point, GBBody* other)
	{
		pointConstraint.pBody = this;
		pointConstraint.setConstraintPoint(point, other);
		constraintType = ConstraintType::POINT;
	}

	void setConstraintEdge(GBVector3 point1, GBVector3 point2, GBBody* other, float samePointError = 0.1f)
	{
		GBVector3 local1 = transform.inverse().transformPoint(point1);
		GBVector3 local2 = transform.inverse().transformPoint(point2);
		GBVector3 sign1 = GBSign(local1);
		GBVector3 sign2 = GBSign(local2);
		GBVector3 localSign = GBSign(pointConstraint.localConstraintPoint);

		if (!sign1.epsilonEqual(sign2))
		{
			edgeConstraint.pBody = this;
			constraintType = ConstraintType::EDGE;
			edgeConstraint.setConstraintEdge(GBEdge(point1, point2), other);
		}

		bool s1Equal = false;
		bool s2Equal = false;
		if (sign1.epsilonEqual(localSign))
		{
			s1Equal = true;
		}
		if (sign2.epsilonEqual(localSign))
		{
			s2Equal = true;
		}
		if (!s1Equal || !s2Equal)
		{
			edgeConstraint.pBody = this;
			constraintType = ConstraintType::EDGE;
			edgeConstraint.setConstraintEdge(GBEdge(point1, point2), other);
		}
		else if (s1Equal)
		{
			setConstraintPoint(point2, other);
		}
		else
		{
			setConstraintPoint(point1, other);
		}

	}

	void resetConstraints()
	{
		GBBody* pOtherBody = nullptr;
		bool isPoint = false;
		if (constraintType == ConstraintType::POINT)
		{
			pOtherBody = pointConstraint.pOtherBody;
			isPoint = true;
		}
		else if (constraintType == ConstraintType::EDGE)
			pOtherBody = edgeConstraint.pOtherBody;
		if (pOtherBody && pOtherBody != this)
		{
			if (pOtherBody->constraintType == ConstraintType::POINT)
				pOtherBody->pointConstraint.pOtherBody = nullptr;
			else if (pOtherBody->constraintType == ConstraintType::EDGE)
				pOtherBody->edgeConstraint.pOtherBody = nullptr;
		}
		pointConstraint = PointConstraint();
		edgeConstraint = EdgeConstraint();
		constraintType = ConstraintType::NONE;
	}

	void resetContactConstraints()
	{
		std::unordered_set<GBBody*> visited;
		resetConstraintsRecursive(visited);
	}

	void resetConstraintsRecursive(std::unordered_set<GBBody*>& visited)
	{
		if (visited.count(this)) return;
		visited.insert(this);

		resetConstraints();

		for (GBBody* b : dynamicBodies)
			if (b)
			{
				b->resetConstraintsRecursive(visited);
			}

		for (GBBody* b : staticBodies)
			if (b)
			{
				b->resetConstraintsRecursive(visited);
			}
	}


	GBBody(float mass = 1.0f, const GBVector3& halfExtents = { 0.5f, 0.5f, 0.5f })
		: mass(mass)
	{
		invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
		velocity = { 0,0,0 };
		angularVelocity = { 0,0,0 };
		forceAccum = { 0,0,0 };
		torqueAccum = { 0,0,0 };

		// Default material
		restitution = 0.3f;
		staticFriction = 0.5f;
		dynamicFriction = 0.3f;

		setInertia(halfExtents);
	}

	void setInertia(GBVector3 halfExtents)
	{
		// Box inertia formula: I = 1/12 * m * (h^2 + d^2) for each axis
		inertia.x = (1.0f / 12.0f) * mass * (halfExtents.y * 2 * halfExtents.y * 2 + halfExtents.z * 2 * halfExtents.z * 2);
		inertia.y = (1.0f / 12.0f) * mass * (halfExtents.x * 2 * halfExtents.x * 2 + halfExtents.z * 2 * halfExtents.z * 2);
		inertia.z = (1.0f / 12.0f) * mass * (halfExtents.x * 2 * halfExtents.x * 2 + halfExtents.y * 2 * halfExtents.y * 2);

		invInertia = {
			(inertia.x > 0.0f) ? 1.0f / inertia.x : 0.0f,
			(inertia.y > 0.0f) ? 1.0f / inertia.y : 0.0f,
			(inertia.z > 0.0f) ? 1.0f / inertia.z : 0.0f
		};
	}


	void setMass(float m)
	{
		mass = m;
		invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
	}


	// Add a force at the center of mass
	void addForce(const GBVector3& force)
	{
		forceAccum += force;
	}

	// Add a force at a point in world space (produces torque)
	void addForceAtPoint(const GBVector3& force, const GBVector3& point)
	{
		forceAccum += force;
		GBVector3 r = point - transform.position; // lever arm
		torqueAccum += GBCross(r, force);
	}

	// Add torque directly
	void addTorque(const GBVector3& torque)
	{
		torqueAccum += torque;
	}

	// Apply an instantaneous linear impulse at the center of mass
	void applyImpulse(const GBVector3& impulse)
	{
		velocity += impulse * invMass;
	}

	// Apply an instantaneous impulse at a world-space point (produces torque)
	void applyImpulseAtPoint(const GBVector3& impulse, const GBVector3& point)
	{
		velocity += impulse * invMass;

		GBVector3 r = point - transform.position; // lever arm
		GBVector3 deltaAngular = GBCross(r, impulse); // torque
		angularVelocity.x += deltaAngular.x * invInertia.x;
		angularVelocity.y += deltaAngular.y * invInertia.y;
		angularVelocity.z += deltaAngular.z * invInertia.z;
	}

	// Clear accumulated forces and torque (usually at the end of the frame)
	void clearForces()
	{
		forceAccum = { 0, 0, 0 };
		torqueAccum = { 0, 0, 0 };
	}

	void reset()
	{
		clearForces();
		velocity = GBVector3::zero();
		angularVelocity = GBVector3::zero();
		resetContactConstraints();
		wakeIsland();
	}

	// Gyroscopic stabilization
	void applyGyroStabilization(float gyroFactor = 0.1f)
	{
		if (angularVelocity.lengthSquared() < 1e-6f)
			return;

		GBVector3 L = {
			angularVelocity.x * inertia.x,
			angularVelocity.y * inertia.y,
			angularVelocity.z * inertia.z
		};

		GBVector3 gyroTorque = GBCross(angularVelocity, L) * gyroFactor;

		angularVelocity.x += gyroTorque.x * invInertia.x;
		angularVelocity.y += gyroTorque.y * invInertia.y;
		angularVelocity.z += gyroTorque.z * invInertia.z;
	}

	bool isDynamic()
	{
		if (isStatic)
			return false;
		if (isSleeping)
			return false;
		return true;
	}

	void wakeRecursive(std::unordered_set<GBBody*>& visited)
	{
		if (visited.count(this)) return;
		visited.insert(this);

		isSleeping = false;
		sleepTimer = 0.0f;
		staticGeometries.clear();
		isGrounded = false;

		for (GBBody* b : dynamicBodies)
			if (b && !b->isStatic)
				b->wakeRecursive(visited);

		for (GBBody* b : staticBodies)
			if (b && !b->isStatic)
				b->wakeRecursive(visited);
	}

	// wrapper
	void wakeIsland()
	{
		std::unordered_set<GBBody*> visited;
		wakeRecursive(visited);
	}

	bool hasStaticAttachmentRecursive(std::unordered_set<const GBBody*>& visited) const
	{
		if (visited.count(this))
			return false;

		visited.insert(this);

		if (isStatic)
			return true;

		for (GBBody* b : dynamicBodies)
		{
			if (b && b->hasStaticAttachmentRecursive(visited))
				return true;
		}

		for (GBBody* b : staticBodies)
		{
			if (b && b->hasStaticAttachmentRecursive(visited))
				return true;
		}

		return false;
	}

	bool hasStaticAttachment() const
	{
		std::unordered_set<const GBBody*> visited;
		return hasStaticAttachmentRecursive(visited);
	}

	void wake()
	{
		isSleeping = false;
		sleepTimer = 0.0f;
		awakeTimer = 0.0f;
		resetContactConstraints();
		staticGeometries.clear();
	}


	void addDynamicContact(GBBody* other, bool awaken = false)
	{
		if (std::find(dynamicBodies.begin(),
			dynamicBodies.end(),
			other) == dynamicBodies.end())
		{
			dynamicBodies.push_back(other);
		}

		if (awaken && isSleeping)
			wake();

		if (awaken && other->isSleeping)
			other->wake();
	}

	std::vector<GBVector3> staticNormals;

	void addStaticContact(GBBody* other, GBVector3 normal = GBVector3::zero())
	{
		if (std::find(staticBodies.begin(),
			staticBodies.end(),
			other) == staticBodies.end())
		{
			staticBodies.push_back(other);
			staticNormals.push_back(normal);
		}
	}

	void addStaticGeometry(GBStaticGeometry* pStaticGeo)
	{
		if (std::find(staticGeometries.begin(),
			staticGeometries.end(),
			pStaticGeo) == staticGeometries.end())
		{
			staticGeometries.push_back(pStaticGeo);
		}
	}


	// Physics update (same as before, now clears forces automatically)
	void update(float dt)
	{
		if (invMass <= 0.0f) return; // static body

		// Skip physics if sleeping
		if (isSleeping || isStatic)
		{
			updateColliders();
			clearForces(); // make sure forces aren't accumulating
			return;
		}

		// Linear
		GBVector3 acceleration = forceAccum * invMass;
		velocity += acceleration * dt;
		//GBVector3 nextPosition = transform.position + velocity * dt;
		//prevPosition = transform.position;
		//transform.position = nextPosition;

		// Angular
		GBVector3 angularAcc = {
			torqueAccum.x * invInertia.x,
			torqueAccum.y * invInertia.y,
			torqueAccum.z * invInertia.z
		};
		angularVelocity += angularAcc * dt;

		//if (angularVelocity.lengthSquared() > 0.0f)
		//{
		//	float angle = angularVelocity.length() * dt;
		//	GBVector3 axis = angularVelocity.normalized();
		//	GBQuaternion deltaRot = GBQuaternion::fromAxisAngle(axis, angle);
		//	transform.rotation = deltaRot * transform.rotation;
		//	transform.rotation.normalize();
		//}

		// Clear accumulators
		clearForces();

		// Sync colliders
		updateColliders();

		frameManifold.reset();

	}


	void updateTransform(float dt)
	{
		GBVector3 nextPosition = transform.position + velocity * dt;
		prevPosition = transform.position;
		transform.position = nextPosition;

		if (angularVelocity.lengthSquared() > 0.0f)
		{
			float angle = angularVelocity.length() * dt;
			GBVector3 axis = angularVelocity.normalized();
			GBQuaternion deltaRot = GBQuaternion::fromAxisAngle(axis, angle);
			prevRotation = transform.rotation;
			transform.rotation = deltaRot * transform.rotation;
			transform.rotation.normalize();
		}

		//handleConstraints();
		switch (constraintType)
		{
		case ConstraintType::POINT:
			pointConstraint.updateConstraintPoint(dt);
			break;
		case ConstraintType::EDGE:
			edgeConstraint.updateConstraint(dt);
			break;
		case ConstraintType::PLANE:
			break;
		}
	}

	void reevaluateTrueVelocities(float dt)
	{
		velocity = (transform.position - prevPosition) / dt;
		angularVelocity = GBQuaternion::toAngularVelocity(prevRotation, transform.rotation, dt);
	}

	void updateColliders()
	{
		int i = 0;
		for (auto* c : colliders)
		{
			GBTransform world =
				c->localTransform * transform;   // compose (pos + rot + scale)

			c->setTransform(world);
			c->isContacted = false;
			if (i == 0)
			{
				aabb = c->aabb;
			}
			else
			{
				aabb.includeAABB(c->aabb);
			}
			i++;
		}
	}

	void clearContactedBodies()
	{
		dynamicBodies.clear();
		staticBodies.clear();
		staticNormals.clear();
	}

	GBVector3 realVelocity(float dt)
	{
		return (transform.position - prevPosition) / dt;
	}

	GBVector3 realAngularVelocity(float dt)
	{
		return GBQuaternion::toAngularVelocity(prevRotation, transform.rotation, dt);
	}
};

inline void GBManifold::flipAndSwapIfOnTop()
{
	if (pIncident->pBody->transform.position.z < pReference->pBody->transform.position.z)
	{
		flipAndSwap();
	}
}

inline void GBManifold::flipAndSwapIfContactOnTop(GBVector3 manifoldNormal)
{
	if (GBDot(manifoldNormal, GBVector3::up()) < 0.00f)
		manifoldNormal *= -1.0f;
	float projRef = GBDot(manifoldNormal, pReference->pBody->transform.position);
	float projInc = GBDot(manifoldNormal, pIncident->pBody->transform.position);
	if (projInc < projRef)
	{
		flipAndSwap();
	}
}


struct GBAABBCollider : public GBCollider {
	GBAABBCollider()
	{
		type = ColliderType::AABB;
		aabb = GBAABB(GBVector3(), GBVector3{0.5f,0.5f,0.5f});
		updateAABB();
	}
	GBAABBCollider(const GBVector3& halfExtents)
	{
		type = ColliderType::AABB;
		aabb = GBAABB(GBVector3(), halfExtents);
		updateAABB();
	}

	void updateAABB() override
	{
		aabb = GBAABB(transform.position, aabb.halfExtents);
	}

	float volume() const  override
	{
		return aabb.halfExtents.x * aabb.halfExtents.y * aabb.halfExtents.z;
	}
};

struct GBSphereCollider : public GBCollider {
	float radius;
	GBSphereCollider()
		: GBCollider(), radius(0.5f)
	{
		type = ColliderType::Sphere;
		updateAABB();
	}

	GBSphereCollider(float radius)
		: GBCollider(), radius(radius)
	{
		type = ColliderType::Sphere;
		updateAABB();
	}

	void updateAABB() override
	{
		aabb = GBAABB(transform.position, GBVector3(radius, radius, radius));
	}	

	float volume() const  override
	{
		return GB_PI * radius * radius * radius;
	}
};


enum GBStaticGeometryType
{
	PLANE = 0,
	TRIANGLE = 1,
	QUAD = 2,
	NONE = 3
};

struct GBStaticGeometry
{
	GBStaticGeometryType type;
	std::vector<GBCell*> occupiedCells;
	GBStaticGeometry(GBStaticGeometryType type) :
		type(type)
	{

	}
	GBStaticGeometry() :
		type(GBStaticGeometryType::NONE)
	{

	}
};

struct GBPlane : GBStaticGeometry
{
	GBVector3 normal; // Should be normalized
	GBVector3 point;
	GBPlane()
		: normal(GBVector3::up()), point(GBVector3(0, 0, 0)), GBStaticGeometry(GBStaticGeometryType::PLANE)
	{
	}
	GBPlane(const GBVector3& normal,GBVector3 point)
		: normal(normal), point(point), GBStaticGeometry(GBStaticGeometryType::PLANE)
	{
	}
};

struct GBQuad : GBStaticGeometry
{
	GBFrame frame;
	GBVector3 position;
	float xSize;
	float ySize;

	enum EdgeDirections
	{
		BACK = 0,
		RIGHT,
		FORWARD,
		LEFT
	};

	GBQuad() : GBStaticGeometry(GBStaticGeometryType::QUAD)
	{
		frame = GBFrame();
		position = { 0,0,0 };
		xSize = 1.0f;
		ySize = 1.0f;
	}

	GBQuad(GBTransform transform, GBVector3 position, float xSize, float ySize) :
		frame(GBFrame(transform)), position(position), xSize(xSize), ySize(ySize), GBStaticGeometry(GBStaticGeometryType::QUAD)
	{

	}

	GBQuad( GBVector3 forward, GBVector3 right, GBVector3 up, GBVector3 position, float xSize, float ySize) :
		frame(GBFrame(forward, right, up)), position(position), xSize(xSize), ySize(ySize), GBStaticGeometry(GBStaticGeometryType::QUAD)
	{

	}

	GBPlane toPlane() const
	{
		return GBPlane(frame.up, position);
	}

	void extractVerts(GBVector3 verts[4]) const
	{
		verts[0] = position - frame.forward * xSize - frame.right * ySize;
		verts[1] = position - frame.forward * xSize + frame.right * ySize;
		verts[2] = position + frame.forward * xSize + frame.right * ySize;
		verts[3] = position + frame.forward * xSize - frame.right * ySize;
	}

	void applyTransform(const GBTransform transform)
	{
		frame.applyTransform(transform);
		position = transform.transformPoint(position);
	}

	void extractEdges(GBEdge edges[4]) const
	{
		GBVector3 verts[4];
		extractVerts(verts);
		edges[0] = GBEdge(verts[0], verts[1]);
		edges[1] = GBEdge(verts[1], verts[2]);
		edges[2] = GBEdge(verts[2], verts[3]);
		edges[3] = GBEdge(verts[3], verts[0]);
	}

	bool isPointContained(GBVector3 point) const
	{
		GBVector3 dp = point - position;
		float rightDist = GBDot(frame.right, dp);
		float forwardDist = GBDot(frame.forward, dp);
		if (GBAbs(rightDist) <= ySize)
		{
			if (GBAbs(forwardDist) <= xSize)
			{
				return true;
			}
		}
		return false;
	}

	GBEdge closestEdgeToPoint(GBVector3 point) const
	{
		GBVector3 dp = point - position;

		float rightDist = GBDot(frame.right, dp) / ySize;
		float forwardDist = GBDot(frame.forward, dp) / xSize;

		GBEdge edges[4];
		extractEdges(edges);

		if (GBAbs(rightDist) >= GBAbs(forwardDist))
		{
			return (rightDist >= 0.0f)
				? edges[RIGHT]
				: edges[LEFT];
		}
		else
		{
			return (forwardDist >= 0.0f)
				? edges[FORWARD]
				: edges[BACK];
		}
	}
};

struct GBTriangle : GBStaticGeometry
{
	GBVector3 vertices[3];
	GBVector3 normal;

	GBTriangle(const GBVector3& v0, const GBVector3& v1, const GBVector3& v2) :
		GBStaticGeometry(GBStaticGeometryType::TRIANGLE)
	{
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		normal = GBCross(v1 - v0, v2 - v0).normalized();
	}

	GBPlane toPlane() const
	{
		return GBPlane(normal, vertices[0]);
	}

	void extractEdges(GBEdge outEdges[3]) const
	{
		outEdges[0] = GBEdge(vertices[0], vertices[1]);
		outEdges[1] = GBEdge(vertices[1], vertices[2]);
		outEdges[2] = GBEdge(vertices[2], vertices[0]);
	}

	GBEdge closestEdgeToPoint(const GBVector3& p) const
	{
		GBEdge edges[3];
		extractEdges(edges);
		GBEdge closestEdge = edges[0];
		float minDistSq = FLT_MAX;
		for (int i = 0; i < 3; i++)
		{
			GBVector3 closestPoint;
			edges[i].closestPointBetween(p, closestPoint);
			float distSq = (closestPoint - p).lengthSquared();
			if (distSq < minDistSq)
			{
				minDistSq = distSq;
				closestEdge = edges[i];
			}
		}
		return closestEdge;
	}

	bool PointInTriangle(
		const GBVector3& p,
		float epsilon = 1e-6f
	) const
	{
		const GBVector3& A = vertices[0];
		const GBVector3& B = vertices[1];
		const GBVector3& C = vertices[2];
		const GBVector3& N = normal;

		GBVector3 c0 = GBCross(B - A, p - A);
		GBVector3 c1 = GBCross(C - B, p - B);
		GBVector3 c2 = GBCross(A - C, p - C);

		if (GBDot(c0, N) < -epsilon) return false;
		if (GBDot(c1, N) < -epsilon) return false;
		if (GBDot(c2, N) < -epsilon) return false;

		return true;
	}

	GBAABB toAABB() const
	{
		GBVector3 minV = vertices[0];
		GBVector3 maxV = vertices[0];
		for (int i = 1; i < 3; i++)
		{
			minV = GBMin(minV, vertices[i]);
			maxV = GBMax(maxV, vertices[i]);
		}
		GBVector3 center = (minV + maxV) * 0.5f;
		GBVector3 halfExtents = (maxV - minV) * 0.5f;
		return GBAABB(center, halfExtents);
	}

	void applyTranslation(GBVector3 trans)
	{
		for (int i = 0; i < 3; i++)
			vertices[i] += trans;
	}

	void applyTransformation(const GBTransform& transform)
	{
		for (int i = 0; i < 3; i++)
		{
			vertices[i] = transform.transformPoint(vertices[i]);
		}
		normal = transform.transformDirection(normal);
	}

	GBVector3 center() const
	{
		GBVector3 avg = vertices[0];
		avg += vertices[1];
		avg += vertices[2];
		return avg /= 3.0f;
	}
};

inline bool GBManifold::canSleep(const GBVector3& com) const
{
	if (numContacts < 2)
		return false; // not enough support

	// Compute average normal of all contacts
	GBVector3 avgNormal = GBVector3::zero();
	for (int i = 0; i < numContacts; i++)
		avgNormal += contacts[i].normal;
	avgNormal = avgNormal.normalized();

	// Reject if support is too tilted
	const float minUpDot = 0.90f; // ~36 degrees tilt tolerance
	if (GBDot(avgNormal, GBVector3::up()) < minUpDot)
		return false;

	// --- 2-contact edge case ---
	if (numContacts == 2)
	{
		GBEdge edge(contacts[0].position, contacts[1].position);
		GBVector3 closest;
		edge.closestPointBetween(com, closest);

		GBVector3 edgeDir = (edge.b - edge.a).normalized();
		GBVector3 toCOM = com - closest;

		// Project to plane perpendicular to edge
		GBVector3 perp = toCOM - edgeDir * GBDot(toCOM, edgeDir);

		float distSq = perp.lengthSquared();
		float maxDistSq = 0.02f * 0.02f; // tune for scene scale

		return distSq <= maxDistSq;
	}

	// --- 3+ contacts triangle case ---
	// Take 3 deepest contacts
	GBTriangle tri(contacts[0].position, contacts[1].position, contacts[2].position);

	// Project COM onto triangle plane
	GBVector3 N = tri.normal;
	float dist = GBDot(com - tri.vertices[0], N);
	GBVector3 proj = com - N * dist;

	// Check if projected point is inside triangle
	return tri.PointInTriangle(proj);
}

struct GBBoxCollider : public GBCollider {
	GBVector3 halfExtents;
	GBVector3 vertices[8];

	void setVerts()
	{
		GBVector3 up = transform.up();
		GBVector3 right = transform.right();
		GBVector3 forward = transform.forward();
		vertices[0] = transform.position - (right * halfExtents.x) - (up * halfExtents.y) - (forward * halfExtents.z);
		vertices[1] = transform.position + (right * halfExtents.x) - (up * halfExtents.y) - (forward * halfExtents.z);
		vertices[2] = transform.position + (right * halfExtents.x) + (up * halfExtents.y) - (forward * halfExtents.z);
		vertices[3] = transform.position - (right * halfExtents.x) + (up * halfExtents.y) - (forward * halfExtents.z);
		vertices[4] = transform.position - (right * halfExtents.x) - (up * halfExtents.y) + (forward * halfExtents.z);
		vertices[5] = transform.position + (right * halfExtents.x) - (up * halfExtents.y) + (forward * halfExtents.z);
		vertices[6] = transform.position + (right * halfExtents.x) + (up * halfExtents.y) + (forward * halfExtents.z);
		vertices[7] = transform.position - (right * halfExtents.x) + (up * halfExtents.y) + (forward * halfExtents.z);
	}

	GBBoxCollider()
		: halfExtents(0.5f, 0.5f, 0.5f)
	{
		type = ColliderType::Box;
		updateAABB();
		setVerts();
	}
	GBBoxCollider(const GBVector3& halfExtents)
		: halfExtents(halfExtents)
	{
		type = ColliderType::Box;
		updateAABB();
		setVerts();
	}
	void updateAABB() override
	{
		GBVector3 forward = transform.forward();  // X
		GBVector3 right = transform.right();    // Y
		GBVector3 up = transform.up();       // Z

		GBVector3 absForward = GBAbs(forward);
		GBVector3 absRight = GBAbs(right);
		GBVector3 absUp = GBAbs(up);

		// Map halfExtents correctly to the axes
		GBVector3 worldExtents =
			absForward * halfExtents.x +  // X → forward
			absRight * halfExtents.y +  // Y → right
			absUp * halfExtents.z;   // Z → up

		aabb = GBAABB::fromLowAndHigh(
			transform.position - worldExtents,
			transform.position + worldExtents
		);
	}


	void setPosition(const GBVector3& position, bool doUpdateAABB = true)
	{
		transform.setPosition(position);
		if (doUpdateAABB)
		{
			updateAABB();
			setVerts();
		}
	}

	void setRotation(const GBQuaternion& rotation, bool doUpdateAABB = true)
	{
		transform.setRotation(rotation);
		if (doUpdateAABB)
		{
			updateAABB();
			setVerts();
		}
	}

	void translate(const GBVector3& translation, bool doUpdateAABB = true)
	{
		transform.translate(translation);
		if (doUpdateAABB)
		{
			updateAABB();
			setVerts();
		}
	}

	void rotate(const GBQuaternion& rotation, bool doUpdateAABB = true)
	{
		transform.rotate(rotation);
		if (doUpdateAABB)
		{
			updateAABB();
			setVerts();
		}
	}

	GBQuad cardinalToQuad(GBCardinal dir)
	{

	}

	float volume() const  override
	{
		return halfExtents.x * halfExtents.y * halfExtents.z;
	}
};


enum GBSATCollisionType
{
	FaceA = 0,
	FaceB,
	EdgeEdge
};

struct GBSATCollisionData
{
	GBSATCollisionType bestType;
	GBCardinal bestCardinal;
	GBVector3 bestAxis;
	float minOverlap;

	GBSATCollisionData()
	{
		bestType = GBSATCollisionType::FaceA;
		bestCardinal = GBCardinal::PosX;
		bestAxis = GBVector3::forward();
		minOverlap = FLT_MAX;
	}
};

struct GBManifoldGeneration
{
	static bool GBRaycastPlane(
		const GBPlane& plane,
		const GBVector3& rayOrigin,
		GBVector3 rayDir,
		GBContact& outContact
	)
	{
		rayDir = rayDir.normalized();
		float denom = GBDot(plane.normal, rayDir);

		// Ray parallel to plane
		if (GBAbs(denom) < 1e-6f)
			return false;

		float t = GBDot(plane.normal, plane.point - rayOrigin) / denom;

		// Intersection behind ray origin
		if (t < 0.0f)
			return false;

		outContact.position = rayOrigin + rayDir * t;

		// Ensure normal faces against ray
		outContact.normal = (denom < 0.0f) ? plane.normal : -plane.normal;

		outContact.penetrationDepth = t;
		return true;
	}

	static bool GBRaycastTriangle(
		const GBTriangle& triangle,
		const GBVector3& rayOrigin,
		const GBVector3& rayDir,
		GBContact& outContact
	)
	{
		GBPlane plane = triangle.toPlane();
		GBContact planeContact;
		if (!GBRaycastPlane(plane, rayOrigin, rayDir, planeContact))
			return false;
		if (triangle.PointInTriangle(planeContact.position))
		{
			outContact = planeContact;
			return true;
		}
		return false;
	}

	static bool GBRaycastAABB(
		const GBAABB& box,
		const GBVector3& rayOrigin,
		const GBVector3& rayDir,
		GBContact& outContact,
		float maxDistance = 1e30f)
	{
		GBVector3 invDir = {
			1.0f / (rayDir.x != 0.0f ? rayDir.x : 1e-8f),
			1.0f / (rayDir.y != 0.0f ? rayDir.y : 1e-8f),
			1.0f / (rayDir.z != 0.0f ? rayDir.z : 1e-8f)
		};

		GBVector3 t1 = (box.low() - rayOrigin) * invDir;
		GBVector3 t2 = (box.high() - rayOrigin) * invDir;

		GBVector3 tMinVec = GBMin(t1, t2);
		GBVector3 tMaxVec = GBMax(t1, t2);

		float tMin = GBMax(GBMax(tMinVec.x, tMinVec.y), tMinVec.z);
		float tMax = GBMin(GBMin(tMaxVec.x, tMaxVec.y), tMaxVec.z);

		// No intersection or beyond max distance
		if (tMax < 0 || tMin > tMax || tMin > maxDistance)
			return false;

		bool inside = tMin < 0.0f;  // ray origin inside box
		float tHit = inside ? 0.0f : tMin;

		// Contact point
		outContact.position = inside ? rayOrigin : rayOrigin + rayDir * tHit;

		// Determine normal
		GBVector3 normal(0, 0, 0);
		if (inside)
		{
			// Pick the closest face to rayOrigin
			GBVector3 delta = rayOrigin - box.center;
			GBVector3 absDelta = GBAbs(delta);
			if (absDelta.x > absDelta.y && absDelta.x > absDelta.z)
				normal = (delta.x > 0) ? GBVector3(1, 0, 0) : GBVector3(-1, 0, 0);
			else if (absDelta.y > absDelta.z)
				normal = (delta.y > 0) ? GBVector3(0, 1, 0) : GBVector3(0, -1, 0);
			else
				normal = (delta.z > 0) ? GBVector3(0, 0, 1) : GBVector3(0, 0, -1);
		}
		else
		{
			// Outside: normal along axis of tMin
			if (tMinVec.x > tMinVec.y && tMinVec.x > tMinVec.z)
				normal.x = (invDir.x < 0) ? 1.0f : -1.0f;
			else if (tMinVec.y > tMinVec.z)
				normal.y = (invDir.y < 0) ? 1.0f : -1.0f;
			else
				normal.z = (invDir.z < 0) ? 1.0f : -1.0f;
		}

		outContact.normal = normal;
		outContact.penetrationDepth = inside ? 0.0f : tHit;

		return true;
	}

	static bool GBRaycastSphere(
		const GBSphereCollider& sphere,
		const GBVector3& rayOrigin,
		const GBVector3& rayDir,
		GBContact& outContact,
		float maxDistance = 1e30f)
	{
		GBVector3 m = rayOrigin - sphere.transform.position;
		float b = m.dot(rayDir);           // projection of m onto ray
		float c = m.dot(m) - sphere.radius * sphere.radius;

		// Ray origin outside sphere and pointing away
		if (c > 0.0f && b > 0.0f)
			return false;

		float discriminant = b * b - c;
		if (discriminant < 0.0f)
			return false; // no intersection

		float tHit = -b - sqrtf(discriminant);

		// If inside sphere, clamp tHit to 0
		bool inside = tHit < 0.0f;
		tHit = inside ? 0.0f : tHit;

		if (tHit > maxDistance)
			return false;

		// Fill contact
		outContact.position = rayOrigin + rayDir * tHit;
		outContact.normal = (inside ? rayOrigin - sphere.transform.position : outContact.position - sphere.transform.position).normalized();
		outContact.penetrationDepth = tHit;

		return true;
	}

	static bool GBRaycastOBB(
		const GBTransform& boxTransform,
		const GBVector3& boxHalfExtents,
		const GBVector3& rayOrigin,
		const GBVector3& rayDir,
		GBContact& outContact,
		float maxDistance = 1e30f)
	{
		GBTransform inverseTransform = boxTransform.inverse();
		GBVector3 inverseOrigin = inverseTransform.transformPoint(rayOrigin);
		GBVector3 inverseDir = inverseTransform.transformDirection(rayDir);
		GBAABB aabb(GBVector3::zero(), boxHalfExtents);
		if (GBRaycastAABB(aabb, inverseOrigin, inverseDir, outContact, maxDistance))
		{
			outContact.applyTransformation(boxTransform);
			return true;
		}
		return false;
	}

	static bool GBRaycastOBB(
		const GBBoxCollider& box,
		const GBVector3& rayOrigin,
		const GBVector3& rayDir,
		GBContact& outContact,
		float maxDistance = 1e30f)
	{
		return GBRaycastOBB(box.transform, box.halfExtents, rayOrigin, rayDir, outContact, maxDistance);
	}


	static bool GBContactSpherePoint(
		const GBVector3& spherePos,
		const float radius,
		const GBVector3& point,
		GBContact& outContact)
	{
		GBVector3 dir = spherePos - point;
		float dist = dir.length();
		if (dist <= radius)
		{
			outContact.position = point;
			outContact.normal = dir.normalized();
			outContact.penetrationDepth = radius - dist;
			return true;
		}
		return false;
	}

	static bool GBContactSpherePoint(
		const GBSphereCollider& sphere,
		const GBVector3& point,
		GBContact& outContact)
	{
		return GBManifoldGeneration::GBContactSpherePoint(sphere.transform.position, sphere.radius, point, outContact);
	}

	static bool GBContactSphereEdge(
		const GBVector3& spherePos,
		const float radius,
		const GBEdge& edge,
		GBContact& outContact)
	{

		if (edge.closestPointBetween(spherePos, outContact.position))
		{
			float dist = (outContact.position - spherePos).length();
			if (dist <= radius)
			{
				outContact.normal = (spherePos - outContact.position).normalized();
				outContact.penetrationDepth = radius - dist;
				return true;
			}
		}
		if (GBContactSpherePoint(spherePos, radius, edge.a, outContact))
		{
			return true;
		}
		else if (GBContactSpherePoint(spherePos, radius, edge.b, outContact))
		{
			return true;
		}
		return false;
	}

	static bool GBContactSphereEdge(
		const GBSphereCollider& sphere,
		const GBEdge& edge,
		GBContact& outContact)
	{
		return GBContactSphereEdge(sphere.transform.position, sphere.radius, edge, outContact);
	}

	static bool GBContactSphereTriangle(
		const GBSphereCollider& sphere,
		const GBTriangle& triangle,
		GBContact& outContact)
	{
		GBPlane plane = triangle.toPlane();
		GBVector3 normal = GBFixNormal(plane.normal, plane.point, sphere.transform.position);
		GBContact contact;
		if (GBManifoldGeneration::GBRaycastPlane(plane, sphere.transform.position, -normal, contact))
		{
			// Check if the contact point is inside the triangle
			if (triangle.PointInTriangle(contact.position))
			{
				GBVector3 dir = sphere.transform.position - contact.position;
				float distSq = GBDot(dir, dir);
				float radiusSq = sphere.radius * sphere.radius;
				if (distSq < radiusSq)
				{
					float dist = std::sqrt(distSq);
					outContact.position = contact.position;
					outContact.normal = normal; // Avoid division by zero
					outContact.penetrationDepth = sphere.radius - dist;
					return true;
				}
			}
			GBEdge closestEdge = triangle.closestEdgeToPoint(sphere.transform.position);
			GBContact edgeContact;
			if (GBManifoldGeneration::GBContactSphereEdge(sphere, closestEdge, edgeContact))
			{
				outContact = edgeContact;
				return true;
			}
		}
		return false;
	}

	static bool GBContactSphereQuad(
		const GBSphereCollider& sphere,
		const GBQuad& quad,
		GBContact& outContact)
	{
		GBPlane plane = quad.toPlane();
		GBVector3 normal = GBFixNormal(plane.normal, plane.point, sphere.transform.position);
		GBContact contact;
		if (GBManifoldGeneration::GBRaycastPlane(plane, sphere.transform.position, -normal, contact))
		{
			// Check if the contact point is inside the triangle
			if (quad.isPointContained(contact.position))
			{
				GBVector3 dir = sphere.transform.position - contact.position;
				float distSq = GBDot(dir, dir);
				float radiusSq = sphere.radius * sphere.radius;
				if (distSq < radiusSq)
				{
					float dist = std::sqrt(distSq);
					outContact.position = contact.position;
					outContact.normal = normal; // Avoid division by zero
					outContact.penetrationDepth = sphere.radius - dist;
					outContact.pIncident = (GBCollider*)&sphere;
					return true;
				}
			}
			GBEdge closestEdge = quad.closestEdgeToPoint(sphere.transform.position);
			GBContact edgeContact;
			if (GBManifoldGeneration::GBContactSphereEdge(sphere, closestEdge, edgeContact))
			{
				outContact = edgeContact;
				outContact.pIncident = (GBCollider*)&sphere;
				return true;
			}
		}
		return false;
	}


	static bool GBContactSphereSphere(
		const GBSphereCollider& a,
		const GBSphereCollider& b,
		GBContact& outContact)
	{
		GBVector3 dp = b.transform.position - a.transform.position;
		float distSq = dp.lengthSquared();
		float r = a.radius + b.radius;

		if (distSq > r * r)
			return false;

		float dist = std::sqrt(distSq);

		// normal from A → B
		GBVector3 normal =
			(dist > 0.0f) ? (dp / dist) : GBVector3{ 1,0,0 };

		outContact.normal = normal;
		outContact.position =
			a.transform.position + normal * a.radius;
		outContact.penetrationDepth = r - dist;

		return true;
	}

	static bool GBManifoldSphereSphere(
		const GBSphereCollider& a,
		const GBSphereCollider& b,
		GBManifold& outManifold)
	{
		GBContact c;
		if (GBContactSphereSphere(a, b, c))
		{
			outManifold.addContact(c);
			outManifold.pReference = (GBCollider*)&a;
			outManifold.pIncident = (GBCollider*)&b;
			outManifold.separation = c.penetrationDepth;
			outManifold.normal = c.normal;
			return true;
		}
		return false;
	}


	static bool GBContactSphereAABB(
		const GBSphereCollider& sphere,
		const GBAABB& box,
		GBContact& outContact)
	{
		return GBContactSphereAABB(sphere.transform.position, sphere.radius, box, outContact);
	}
	static bool GBContactSphereAABB(
		const GBVector3& spherePosition,
		const float radius,
		const GBAABB& box,
		GBContact& outContact)
	{
		GBVector3 delta = GBAbs(spherePosition - box.center);

		float penetrationX = box.halfExtents.x - delta.x + radius;
		float penetrationY = box.halfExtents.y - delta.y + radius;
		float penetrationZ = box.halfExtents.z - delta.z + radius;

		if (penetrationX > 0 && penetrationY > 0 && penetrationZ > 0)
		{
			GBCardinal dir;
			// Normal + penetration depth
			if (penetrationX < penetrationY && penetrationX < penetrationZ)
			{
				if (spherePosition.x < box.center.x)
				{
					outContact.normal = GBVector3(-1, 0, 0);
					dir = GBCardinal::NegX;
				}
				else
				{
					outContact.normal = GBVector3(1, 0, 0);
					dir = GBCardinal::PosX;
				}
				outContact.penetrationDepth = penetrationX;
			}
			else if (penetrationY < penetrationZ)
			{
				if (spherePosition.y < box.center.y)
				{
					outContact.normal = GBVector3(0, -1, 0);
					dir = GBCardinal::NegY;
				}
				else
				{
					outContact.normal = GBVector3(0, 1, 0);
					dir = GBCardinal::PosY;
				}
				outContact.penetrationDepth = penetrationY;
			}
			else
			{
				if (spherePosition.z < box.center.z)
				{
					outContact.normal = GBVector3(0, 0, -1);
					dir = GBCardinal::NegZ;
				}
				else
				{
					outContact.normal = GBVector3(0, 0, 1);
					dir = GBCardinal::PosZ;
				}
				outContact.penetrationDepth = penetrationZ;
			}
			switch (dir)
			{
			case GBCardinal::PosX:
				outContact.position = GBVector3(box.center.x + box.halfExtents.x,
					spherePosition.y,
					spherePosition.z);
				break;
			case GBCardinal::NegX:
				outContact.position = GBVector3(box.center.x - box.halfExtents.x,
					spherePosition.y,
					spherePosition.z);
				break;
			case GBCardinal::PosY:
				outContact.position = GBVector3(spherePosition.x,
					box.center.y + box.halfExtents.y,
					spherePosition.z);
				break;
			case GBCardinal::NegY:
				outContact.position = GBVector3(spherePosition.x,
					box.center.y - box.halfExtents.y,
					spherePosition.z);
				break;
			case GBCardinal::PosZ:
				outContact.position = GBVector3(spherePosition.x,
					spherePosition.y,
					box.center.z + box.halfExtents.z);
				break;
			case GBCardinal::NegZ:
				outContact.position = GBVector3(spherePosition.x,
					spherePosition.y,
					box.center.z - box.halfExtents.z);
				break;
			}
			if (!box.isPointOnPlane(outContact.position, dir))
			{
				// Edge case
				GBEdge edge = box.pointToClosestEdge(spherePosition);
				if (!GBManifoldGeneration::GBContactSphereEdge(spherePosition, radius, edge, outContact))
					return false;
			}

			// ✅ Closest point on AABB


			return true;
		}

		return false;
	}

	static bool GBContactSphereBox(
		const GBSphereCollider& sphere,
		const GBBoxCollider& box,
		GBContact& outContact)
	{
		GBTransform inverse = box.transform.inverse();
		GBAABB aabb({ 0,0,0 }, box.halfExtents);
		GBVector3 transformedPosition = sphere.transform.position;
		transformedPosition = inverse.transformPoint(transformedPosition);
		if (GBManifoldGeneration::GBContactSphereAABB(transformedPosition, sphere.radius, aabb, outContact))
		{
			outContact.applyTransformation(box.transform);
			return true;
		}
		return false;
	}

	static bool GBManifoldSphereBox(
		const GBSphereCollider& sphere,
		const GBBoxCollider& box,
		GBManifold& outManifold)
	{
		GBContact c;
		if (GBContactSphereBox(sphere, box, c))
		{
			outManifold.addContact(c);
			outManifold.pReference = (GBCollider*)&box;
			outManifold.pIncident = (GBCollider*)&sphere;
			outManifold.separation = c.penetrationDepth;
			outManifold.normal = c.normal;
			return true;
		}
		return false;
	}

	static bool GBContactClipVertexToTriangleFace(
		const GBVector3& vertex,
		const GBTriangle& triangle,
		const GBVector3 triangleNormal,
		GBContact& contact)
	{
		GBPlane plane = triangle.toPlane();
		if (GBRaycastPlane(plane, vertex, triangleNormal, contact))
		{
			if (contact.penetrationDepth > 0.0f && triangle.PointInTriangle(contact.position))
			{
				contact.normal = triangleNormal;
				return true;
			}
		}
		return false;
	}
	static bool GBContactClipVertexToFace(
		const GBVector3& vertex,
		const GBAABB& aabb,
		GBCardinal face,
		GBContact& contact)
	{
		GBVector3 normal;
		GBVector3 pointOnPlane;

		switch (face)
		{
		case GBCardinal::PosX:
			normal = GBVector3(1, 0, 0);
			pointOnPlane = { aabb.high().x, aabb.center.y, aabb.center.z };
			break;
		case GBCardinal::NegX:
			normal = GBVector3(-1, 0, 0);
			pointOnPlane = { aabb.low().x, aabb.center.y, aabb.center.z };
			break;
		case GBCardinal::PosY:
			normal = GBVector3(0, 1, 0);
			pointOnPlane = { aabb.center.x, aabb.high().y, aabb.center.z };
			break;
		case GBCardinal::NegY:
			normal = GBVector3(0, -1, 0);
			pointOnPlane = { aabb.center.x, aabb.low().y, aabb.center.z };
			break;
		case GBCardinal::PosZ:
			normal = GBVector3(0, 0, 1);
			pointOnPlane = { aabb.center.x, aabb.center.y, aabb.high().z };
			break;
		case GBCardinal::NegZ:
			normal = GBVector3(0, 0, -1);
			pointOnPlane = { aabb.center.x, aabb.center.y, aabb.low().z };
			break;
		}

		// Project vertex onto the plane
		float distance = GBDot(normal, vertex - pointOnPlane);
		GBVector3 projected = vertex - normal * distance;

		// Check if projected point lies within face bounds
		if (!aabb.isPointOnPlane(projected, face))
			return false;

		contact.position = projected;
		contact.normal = normal;
		contact.penetrationDepth = -distance; // optional, depends on usage

		return true;
	}


	static GBVector3 GBCardinalToVector3(GBCardinal dir)
	{
		switch (dir)
		{
		case GBCardinal::PosX:
			return GBVector3::forward();
		case GBCardinal::NegX:
			return GBVector3::back();
		case GBCardinal::PosY:
			return GBVector3::right();
		case GBCardinal::NegY:
			return GBVector3::left();
		case GBCardinal::PosZ:
			return GBVector3::up();
		case GBCardinal::NegZ:
			return GBVector3::down();
		}
		return GBVector3::zero();
	}


	static bool GBManifoldClipQuadVerts(
		const GBQuad& quad,
		const GBAABB& aabb,
		GBCardinal face,
		GBManifold& outManifold)
	{

		// Extract quad vertices
		GBVector3 verts[4];
		quad.extractVerts(verts);

		// Determine face normal
		GBVector3 faceNormal = GBCardinalToVector3(face);

		// Face plane point (center of the face)
		GBVector3 planePoint = aabb.center;
		switch (face)
		{
		case GBCardinal::PosX: planePoint.x += aabb.halfExtents.x; break;
		case GBCardinal::NegX: planePoint.x -= aabb.halfExtents.x; break;
		case GBCardinal::PosY: planePoint.y += aabb.halfExtents.y; break;
		case GBCardinal::NegY: planePoint.y -= aabb.halfExtents.y; break;
		case GBCardinal::PosZ: planePoint.z += aabb.halfExtents.z; break;
		case GBCardinal::NegZ: planePoint.z -= aabb.halfExtents.z; break;
		}

		// Build plane once
		GBPlane facePlane(faceNormal, planePoint);

		for (int i = 0; i < 4; i++)
		{
			GBContact contact;

			// Clip quad vertex to AABB face
			if (!GBContactClipVertexToFace(verts[i], aabb, face, contact))
				continue;

			// Compute penetration depth
			float separation = GBDot(
				planePoint - verts[i],
				faceNormal
			);

			// Reject if vertex is not penetrating
			if (separation < 0.0f)
				continue;

			contact.normal = faceNormal;
			contact.penetrationDepth = separation;

			outManifold.addContact(contact);
		}

		return outManifold.numContacts > 0;
	}


	static bool GBManifoldClipTriangleOnAABBVerts(
		const GBTriangle& triangle,
		const GBAABB& aabb,
		GBCardinal face,
		GBManifold& outManifold)
	{
		// Determine face normal
		GBVector3 faceNormal = GBCardinalToVector3(face);

		// Face plane point (center of the face)
		GBVector3 planePoint = aabb.center;
		switch (face)
		{
		case GBCardinal::PosX: planePoint.x += aabb.halfExtents.x; break;
		case GBCardinal::NegX: planePoint.x -= aabb.halfExtents.x; break;
		case GBCardinal::PosY: planePoint.y += aabb.halfExtents.y; break;
		case GBCardinal::NegY: planePoint.y -= aabb.halfExtents.y; break;
		case GBCardinal::PosZ: planePoint.z += aabb.halfExtents.z; break;
		case GBCardinal::NegZ: planePoint.z -= aabb.halfExtents.z; break;
		}

		// Build plane once
		GBPlane facePlane(faceNormal, planePoint);

		for (int i = 0; i < 3; i++)
		{
			GBContact contact;

			// Clip quad vertex to AABB face
			if (!GBContactClipVertexToFace(triangle.vertices[i], aabb, face, contact))
				continue;

			// Compute penetration depth
			float separation = GBDot(
				planePoint - triangle.vertices[i],
				faceNormal
			);

			// Reject if vertex is not penetrating
			if (separation < 0.0f)
				continue;

			contact.normal = faceNormal;
			contact.penetrationDepth = separation;
			contact.position -= contact.normal * contact.penetrationDepth;

			outManifold.addContact(contact);
		}

		return outManifold.numContacts > 0;
	}

	static bool GBManifoldClipQuadOnTriangleVerts(
		const GBTriangle& triangle,
		const GBQuad& quad,
		GBVector3 normal,
		GBManifold& outManifold)
	{
		// Determine face normal, the box incident face normal should be the opposite of the triangle normal
		GBVector3 faceNormal = normal;
		GBVector3 triangleNormal = triangle.normal;
		if (GBDot(triangleNormal, faceNormal) > 0.0f)
			triangleNormal *= -1.0f;

		GBVector3 verts[4];
		quad.extractVerts(verts);

		GBEdge triangleEdges[3];

		for (int i = 0; i < 4; i++)
		{
			GBContact contact;

			// Clip quad vertex to AABB face
			if (!GBContactClipVertexToTriangleFace(verts[i], triangle, triangleNormal, contact))
				continue;

			// Compute penetration depth
			float separation = GBDot(
				contact.position - verts[i],
				triangleNormal
			);

			// Reject if vertex is not penetrating
			if (separation < 0.0f)
				continue;

			contact.normal = triangleNormal;
			contact.penetrationDepth = separation;


			outManifold.addContact(contact);
		}

		return outManifold.numContacts > 0;
	}


	//static bool GBManifoldClipAABBOnTriangleVerts(
	//	const GBTriangle& triangle,
	//	const GBAABB& aabb,
	//	GBCardinal face,
	//	GBManifold& outManifold)
	//{
	//	GBVector3 faceNormal = triangle.normal;
	//	if (GBDot(aabb.center - triangle.vertices[0], faceNormal) < 0.0f)
	//	{
	//		faceNormal *= -1.0f;
	//	}

	//	// Face plane point (center of the face)
	//	GBVector3 planePoint = aabb.center;
	//	switch (face)
	//	{
	//	case GBCardinal::PosX: planePoint.x += aabb.halfExtents.x; break;
	//	case GBCardinal::NegX: planePoint.x -= aabb.halfExtents.x; break;
	//	case GBCardinal::PosY: planePoint.y += aabb.halfExtents.y; break;
	//	case GBCardinal::NegY: planePoint.y -= aabb.halfExtents.y; break;
	//	case GBCardinal::PosZ: planePoint.z += aabb.halfExtents.z; break;
	//	case GBCardinal::NegZ: planePoint.z -= aabb.halfExtents.z; break;
	//	}

	//	// Build plane once
	//	GBPlane facePlane(faceNormal, triangle.vertices[0]);

	//	GBVector3 boxVerts[4];
	//	aabb.extractVerts(boxVerts);

	//	for (int i = 0; i < 3; i++)
	//	{
	//		GBContact contact;

	//		// Clip quad vertex to AABB face
	//		if (!GBContactClipVertexToFace(triangle.vertices[i], aabb, face, contact))
	//			continue;

	//		// Compute penetration depth
	//		float separation = GBDot(
	//			planePoint - triangle.vertices[i],
	//			faceNormal
	//		);

	//		// Reject if vertex is not penetrating
	//		if (separation < 0.0f)
	//			continue;

	//		contact.normal = faceNormal;
	//		contact.penetrationDepth = separation;

	//		outManifold.addContact(contact);
	//	}

	//	return outManifold.numContacts > 0;
	//}

	static bool GBContactClipEdge(
		const GBEdge& edge,
		const GBAABB& aabb,
		const GBCardinal edgeDir,
		const GBPlane& edgePlane,
		const GBCardinal referencePlaneDir,
		GBContact& outContact
		)
	{
		GBVector3 dir = edge.b - edge.a;
		float len = dir.length();
		dir *= (1.0f / len);
		if (GBRaycastPlane(edgePlane, edge.a, dir, outContact))
		{
			if (aabb.isPointOnPlane(outContact.position, edgeDir) && outContact.penetrationDepth <= len)
			{
				// We have a hit
				GBVector3 adjustedPosition = aabb.forcePointOntoFacePlane(outContact.position, referencePlaneDir);
				outContact.penetrationDepth = (outContact.position - adjustedPosition).length();
				outContact.normal = GBCardinalToVector3(referencePlaneDir);
				outContact.position = adjustedPosition;
				return true;
			}
		}
		return false;
	}



	static bool GBManifoldClipQuadEdges(
		const GBQuad& quad,
		const GBAABB& aabb,
		GBCardinal face,
		GBManifold& outManifold)
	{
		// Extract quad vertices
		GBVector3 verts[4];
		quad.extractVerts(verts);

		// Build quad edges
		GBEdge edges[4] = {
			GBEdge(verts[0], verts[1]),
			GBEdge(verts[1], verts[2]),
			GBEdge(verts[2], verts[3]),
			GBEdge(verts[3], verts[0])
		};

		
		GBPlane facePlane = GBCardinalDirToPlane(aabb, face);
		bool foundContact = false;
		GBCardinal tangentDirs[4];
		GBCardinalExtractTangents(face, tangentDirs);
		tangentDirs[2] = GBCardinalGetReverse(tangentDirs[0]);
		tangentDirs[3] = GBCardinalGetReverse(tangentDirs[1]);

		GBPlane tangentPlanes[4];
		for (int i = 0; i < 4; i++)
			tangentPlanes[i] = GBCardinalDirToPlane(aabb, tangentDirs[i]);

		for (int i = 0; i < 4; i++)
		{
			const GBEdge& edge = edges[i];
			for (int j = 0; j < 4; j++)
			{
				GBContact contact;
				if (GBContactClipEdge(edge, aabb, tangentDirs[j], tangentPlanes[j], face, contact))
				{
					outManifold.addContact(contact);
					foundContact = true;
				}
			}			
		}
		return foundContact;
	}

	static bool GBManifoldClipTriangleEdges(
		const GBTriangle& triangle,
		const GBAABB& aabb,
		GBCardinal face,
		GBManifold& outManifold)
	{
		// Build quad edges
		GBEdge edges[3];
		triangle.extractEdges(edges);


		GBPlane facePlane = GBCardinalDirToPlane(aabb, face);
		bool foundContact = false;
		GBCardinal tangentDirs[4];
		GBCardinalExtractTangents(face, tangentDirs);
		tangentDirs[2] = GBCardinalGetReverse(tangentDirs[0]);
		tangentDirs[3] = GBCardinalGetReverse(tangentDirs[1]);

		GBPlane tangentPlanes[4];
		for (int i = 0; i < 4; i++)
			tangentPlanes[i] = GBCardinalDirToPlane(aabb, tangentDirs[i]);

		for (int i = 0; i < 3; i++)
		{
			const GBEdge& edge = edges[i];
			for (int j = 0; j < 4; j++)
			{
				GBContact contact;
				if (GBContactClipEdge(edge, aabb, tangentDirs[j], tangentPlanes[j], face, contact))
				{
					//contact.position += outManifold.normal * contact.penetrationDepth;
					outManifold.addContact(contact);
					foundContact = true;
				}
			}
		}
		return foundContact;
	}

	static bool GBBuildQuadAABBManifold(
		const GBQuad& quad,
		const GBAABB& aabb,
		GBCardinal face,
		GBManifold& outManifold, bool checkEdges = true)
	{
		bool foundContact = false;

		// 1️⃣ Vertex → face clipping
		if (GBManifoldClipQuadVerts(quad, aabb, face, outManifold))
		{
			foundContact =  true;
			if (outManifold.numContacts == 4)
				return true;
		}

		if (checkEdges)
		{
			// 2️⃣ Edge → face clipping (fallback)
			if (GBManifoldClipQuadEdges(quad, aabb, face, outManifold))
			{
				foundContact = true;
			}
		}

		return foundContact;
	}


	static bool GBBuildTriangleAABBManifold(
		const GBTriangle& triangle,
		const GBAABB& aabb,
		GBCardinal face,
		GBManifold& outManifold, bool checkEdges = true)
	{
		bool foundContact = false;

		// 1️⃣ Vertex → face clipping
		if (GBManifoldClipTriangleOnAABBVerts(triangle, aabb, face, outManifold))
		{
			foundContact = true;
			for (int i = 0; i < outManifold.numContacts; i++)
				outManifold.contacts[i].normal = outManifold.normal;
			if (outManifold.numContacts == 4)
				return true;
		}

		GBQuad incidentFace = GBCardinalDirToQuad(aabb, face);
		if (GBManifoldClipQuadOnTriangleVerts(triangle, incidentFace, incidentFace.frame.up, outManifold))
		{
			foundContact = true;
			if (outManifold.numContacts == 4)
				return true;
		}

		if (checkEdges)
		{
			// 2️⃣ Edge → face clipping (fallback)
			if (GBManifoldClipTriangleEdges(triangle, aabb, face, outManifold))
			{
				foundContact = true;
			}
		}

		return foundContact;
	}


	static bool GBCollisionBoxBoxSAT(
		const GBBoxCollider& A,
		const GBBoxCollider& B,
		GBSATCollisionData& colData,
		float epsilon = 1e-6f)
	{
		const GBVector3 cA = A.transform.position;
		const GBVector3 cB = B.transform.position;
		const GBVector3 t = cB - cA;

		// Unreal convention: X = forward, Y = right, Z = up
		const GBVector3 Aaxes[3] = { A.transform.forward(), A.transform.right(), A.transform.up() };
		const GBVector3 Baxes[3] = { B.transform.forward(), B.transform.right(), B.transform.up() };

		const GBVector3 a = A.halfExtents;
		const GBVector3 b = B.halfExtents;

		colData.minOverlap = std::numeric_limits<float>::max();
		colData.bestCardinal = PosX; // default

		// Temporary reference tracking
		const GBBoxCollider* referenceBox = nullptr;

		auto testAxis = [&](const GBVector3& axis, GBSATCollisionType type, int axisIndex) -> bool
			{
				float lenSq = GBDot(axis, axis);
				if (lenSq < epsilon) return true;

				float centerDist = GBAbs(GBDot(t, axis));

				float rA =
					GBAbs(GBDot(axis, Aaxes[0])) * a.x +
					GBAbs(GBDot(axis, Aaxes[1])) * a.y +
					GBAbs(GBDot(axis, Aaxes[2])) * a.z;

				float rB =
					GBAbs(GBDot(axis, Baxes[0])) * b.x +
					GBAbs(GBDot(axis, Baxes[1])) * b.y +
					GBAbs(GBDot(axis, Baxes[2])) * b.z;

				float overlap = (rA + rB) - centerDist;
				if (overlap < 0.0f) return false;

				float invLen = 1.0f / std::sqrt(lenSq);
				overlap *= invLen;

				if (overlap < colData.minOverlap)
				{
					colData.minOverlap = overlap;
					GBVector3 n = axis * invLen;

					// Determine reference box and flip normal if necessary
					if (type == GBSATCollisionType::FaceA)
					{
						referenceBox = &A;
						if (GBDot(n, t) < 0.0f) n = -n; // reference → incident
						// Map axis to cardinal in reference box space
						static const GBCardinal posMap[3] = { PosX, PosY, PosZ };
						static const GBCardinal negMap[3] = { NegX, NegY, NegZ };
						colData.bestCardinal = (GBDot(n, Aaxes[axisIndex]) > 0) ? posMap[axisIndex] : negMap[axisIndex];
					}
					else if (type == GBSATCollisionType::FaceB)
					{
						referenceBox = &B;
						if (GBDot(n, t) > 0.0f) n = -n; // reference → incident
						static const GBCardinal posMap[3] = { PosX, PosY, PosZ };
						static const GBCardinal negMap[3] = { NegX, NegY, NegZ };
						colData.bestCardinal = (GBDot(n, Baxes[axisIndex]) > 0) ? posMap[axisIndex] : negMap[axisIndex];
					}
					// Edge-edge axes: n left as-is, cardinal not assigned

					colData.bestAxis = n;
					colData.bestType = type;
				}
				return true;
			};

		// Face axes of A
		for (int i = 0; i < 3; ++i)
			if (!testAxis(Aaxes[i], GBSATCollisionType::FaceA, i)) return false;

		// Face axes of B
		for (int i = 0; i < 3; ++i)
			if (!testAxis(Baxes[i], GBSATCollisionType::FaceB, i)) return false;

		// Edge × Edge
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				if (!testAxis(GBCross(Aaxes[i], Baxes[j]), GBSATCollisionType::EdgeEdge, -1)) return false;

		if (colData.bestType == GBSATCollisionType::EdgeEdge)
		{
			if (GBDot(colData.bestAxis, t) < 0.0f)
				colData.bestAxis = -colData.bestAxis;
		}
		return true;
	}



	static void GBCardinalExtractTangents(GBCardinal dir, GBCardinal tangents[2])
	{
		switch (dir)
		{
		case GBCardinal::PosX:
		case GBCardinal::NegX:
			tangents[0] = GBCardinal::PosY;
			tangents[1] = GBCardinal::PosZ;
			return;
		case GBCardinal::PosY:
		case GBCardinal::NegY:
			tangents[0] = GBCardinal::PosX;
			tangents[1] = GBCardinal::PosZ;
			return;
		case GBCardinal::PosZ:
		case GBCardinal::NegZ:
			tangents[0] = GBCardinal::PosX;
			tangents[1] = GBCardinal::PosY;
			return;
		}
	}

	static GBCardinal GBCardinalGetReverse(GBCardinal dir)
	{
		switch (dir)
		{
		case GBCardinal::PosX:
			return GBCardinal::NegX;
		case GBCardinal::NegX:
			return GBCardinal::PosX;
		case GBCardinal::PosY:
			return GBCardinal::NegY;
		case GBCardinal::NegY:
			return GBCardinal::PosY;
		case GBCardinal::PosZ:
			return GBCardinal::NegZ;
		case GBCardinal::NegZ:
			return GBCardinal::PosZ;
		}
		return GBCardinal::None;
	}

	static GBPlane GBCardinalDirToPlane(const GBAABB& aabb, GBCardinal dir)
	{
		return GBPlane(GBCardinalToVector3(dir), aabb.getPointOnPlaneFromCardinal(dir));
	}

	static GBQuad GBCardinalDirToQuad(const GBAABB& aabb, GBCardinal dir)
	{
		GBCardinal tangents[2];
		GBCardinalExtractTangents(dir, tangents);
		GBVector3 tangentVector1 = GBCardinalToVector3(tangents[0]);
		GBVector3 tangentVector2 = GBCardinalToVector3(tangents[1]);
		float tangentSize1 = aabb.getSizeByCardinal(tangents[0]);
		float tangentSize2 = aabb.getSizeByCardinal(tangents[1]);
		float upSize = aabb.getSizeByCardinal(dir);
		GBVector3 up = GBCardinalToVector3(dir);
		return GBQuad(tangentVector1, tangentVector2, up, aabb.center + up*upSize, tangentSize1, tangentSize2);
	}
	
	static GBQuad GBAAABBDirectionToQuad(const GBAABB& aabb, GBVector3 dir)
	{
		return GBCardinalDirToQuad(aabb, aabb.directionToFace(dir));
	}

	static GBQuad GBBoxDirectionToQuad(const GBBoxCollider& box, GBVector3 dir)
	{
		GBVector3 local = box.transform.inverse().transformDirection(dir);
		GBAABB aabb(GBVector3::zero(), box.halfExtents);
		GBQuad quad = GBAAABBDirectionToQuad(aabb, local);
		quad.applyTransform(box.transform);
		return quad;
	}

	static GBVector3 GBBoxDirectionToClosestUp(const GBBoxCollider& box, GBVector3 dir)
	{
		GBVector3 local = box.transform.inverse().transformDirection(dir);
		GBAABB aabb(GBVector3::zero(), box.halfExtents);
		GBQuad quad = GBAAABBDirectionToQuad(aabb, local);
		quad.applyTransform(box.transform);
		return quad.frame.up;
	}

	static GBEdge GBBoxDirectionToEdge(const GBBoxCollider& box, GBVector3 dir)
	{
		GBVector3 local = box.transform.inverse().transformDirection(dir);
		GBAABB aabb(GBVector3::zero(), box.halfExtents);
		GBEdge edge = aabb.dirToClosestEdge(dir);
		edge.applyTransform(box.transform);
		return edge;
	}

	static GBQuad GBBoxCardinalToQuad(const GBBoxCollider& box, GBCardinal cardinal)
	{
		GBAABB aabb(GBVector3::zero(), box.halfExtents);
		GBQuad quad = GBCardinalDirToQuad(aabb, cardinal);
		quad.applyTransform(box.transform);
		return quad;
	}

	static bool GBManifoldEdgeEdge(const GBQuad& quadA,
		const GBQuad& quadB,
		const GBVector3& dir,
		GBManifold& outManifold)
	{
		outManifold.clear();
		GBEdge edgesA[4];
		GBEdge edgesB[4];
		quadA.extractEdges(edgesA);
		quadB.extractEdges(edgesB);

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				GBEdge connection;
				if (GBEdge::closestEdgeBetween(edgesA[i], edgesB[j], connection, true))
				{
					GBContact contact;
					contact.position = connection.a;
					contact.normal = connection.a - connection.b;
					contact.penetrationDepth = contact.normal.length();
					contact.normal *= (1.0f / contact.penetrationDepth);
					if (GBDot(contact.normal, dir) > 0.99f)
						outManifold.addContact(contact);
				}
			}
		}
		return outManifold.numContacts > 0;
	}

	static bool GBManifoldTriangleQuadEdgeEdge(const GBTriangle& triangle,
		const GBQuad& quad,
		const GBVector3& dir,
		GBManifold& outManifold)
	{
		outManifold.clear();
		GBEdge edgesTri[3];
		GBEdge edgesQuad[4];
		triangle.extractEdges(edgesTri);
		quad.extractEdges(edgesQuad);

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				GBEdge connection;
				if (GBEdge::closestEdgeBetween(edgesTri[i], edgesQuad[j], connection, true))
				{
					GBContact contact;
					contact.position = connection.a;
					contact.normal = connection.a - connection.b;
					contact.penetrationDepth = contact.normal.length();
					contact.normal *= (1.0f / contact.penetrationDepth);
					if (GBDot(contact.normal, dir) > 0.99f)
						outManifold.addContact(contact);
				}
			}
		}
		return outManifold.numContacts > 0;
	}

	static bool GBManifoldBoxBox(GBBoxCollider& boxA,
		GBBoxCollider& boxB,
		const GBSATCollisionData& colData,
		GBManifold& outManifold)
	{
		GBVector3 initPos = GBVector3::zero();
		if (boxA.pBody && boxB.pBody)
		{
			if (!boxA.pBody->isStatic || !boxB.pBody->isStatic)
			{
				initPos = boxA.pBody->isStatic ? boxB.transform.position : boxA.transform.position;
				boxA.transform.position -= initPos;
				boxB.transform.position -= initPos;
			}
		}
		else
		{
			// Just use the first box as origin
			initPos = boxA.transform.position;
			boxA.transform.position -= initPos;
			boxB.transform.position -= initPos;
		}

		outManifold.separation = colData.minOverlap;
		outManifold.normal = colData.bestAxis;

		const GBBoxCollider* pReference;
		const GBBoxCollider* pIncident;
		if (colData.bestType == GBSATCollisionType::FaceA)
		{
			// A is the reference B is the incident
			pReference = &boxA;
			pIncident = &boxB;
		}
		else if (colData.bestType == GBSATCollisionType::FaceB)
		{
			// B is the reference A is the incident
			pReference = &boxB;
			pIncident = &boxA;
		}
		else
		{
			GBQuad qa = GBManifoldGeneration::GBBoxDirectionToQuad(boxA, colData.bestAxis);
			GBQuad qb = GBManifoldGeneration::GBBoxDirectionToQuad(boxB, -colData.bestAxis);
			pReference = &boxA;
			pIncident = &boxB;
			outManifold.pReference = (GBCollider*)&boxA;
			outManifold.pIncident = (GBCollider*)&boxB;
			outManifold.isEdge = true;
			if (GBManifoldGeneration::GBManifoldEdgeEdge(qa, qb, colData.bestAxis, outManifold))
			{
				boxA.transform.position += initPos;
				boxB.transform.position += initPos;
				outManifold.applyTransformation(GBTransform(initPos, GBQuaternion()));
				return true;
			}
		}

		outManifold.clear();
		GBTransform referenceInverse = pReference->transform.inverse();
		GBTransform incidentInverse = pIncident->transform.inverse();


		GBQuad incidentFace = GBAAABBDirectionToQuad(GBAABB(GBVector3::zero(), pIncident->halfExtents),
			incidentInverse.transformDirection(-colData.bestAxis));
		incidentFace.applyTransform(pIncident->transform);
		incidentFace.applyTransform(referenceInverse);

		GBAABB referenceAABB(GBVector3::zero(), pReference->halfExtents);

		if (GBBuildQuadAABBManifold(incidentFace, referenceAABB, colData.bestCardinal, outManifold))
		{
			outManifold.applyTransformation(pReference->transform);
			outManifold.pReference = (GBCollider*)pReference;
			outManifold.pIncident = (GBCollider*)pIncident;
			if (GBDot(outManifold.normal, colData.bestAxis) < 0)
			{
				outManifold.normal *= -1.0f;
			}
		}

		if (outManifold.numContacts == 0)
		{
			GBQuad referenceFace = GBAAABBDirectionToQuad(GBAABB(GBVector3::zero(), pReference->halfExtents),
				referenceInverse.transformDirection(colData.bestAxis));
			referenceFace.applyTransform(pReference->transform);
			referenceFace.applyTransform(incidentInverse);
			GBAABB incidentAABB(GBVector3::zero(), pIncident->halfExtents);

			GBManifold m;
			if (GBBuildQuadAABBManifold(referenceFace, incidentAABB, GBCardinalGetReverse(colData.bestCardinal), m, false))
			{
				m.applyTransformation(pIncident->transform);
				for (int i = 0; i < m.numContacts; i++)
				{
					m.contacts[i].normal *= -1.0f;
				}
			}
			outManifold.combine(m);
		}

		outManifold.pIncident = (GBCollider*)pIncident;
		outManifold.pReference = (GBCollider*)pReference;

		if (outManifold.pIncident->pBody && outManifold.pReference->pBody)
		{
			if (outManifold.pIncident->pBody->isStatic)
			{
				GBCollider* temp = outManifold.pIncident;
				outManifold.pIncident = outManifold.pReference;
				outManifold.pReference = temp;
				outManifold.flip();
			}
		}

		boxA.transform.position += initPos;
		boxB.transform.position += initPos;
		outManifold.applyTransformation(GBTransform(initPos, GBQuaternion()));

		return outManifold.numContacts > 0;
	}

	static bool GBManifoldBodyQuad(const GBBody& body,
		const GBQuad& quad,
		GBManifold& outManifold)
	{
		outManifold.clear();
		for (int i = 0; i < body.colliders.size(); i++)
		{
			GBCollider* pCollider = body.colliders[i];
			switch (pCollider->type)
			{
			case ColliderType::Sphere:
			{
				GBSphereCollider* pSphere = (GBSphereCollider*)pCollider;
				GBContact c;
				if (GBContactSphereQuad(*pSphere, quad, c))
				{
					pSphere->isContacted = true;
					outManifold.addContact(c);
				}
			}
				break;
			case ColliderType::Box:
			{

			}
				break;
			}
		}

		return outManifold.numContacts > 0;
	}

	static inline bool AxisParallelToTriangle(const GBVector3& axis,
		const GBVector3& triNormal,
		float eps = 1e-4f)
	{
		return GBAbs(GBAbs(GBDot(axis.normalized(), triNormal)) - 1.0f) < eps;
	}

	static inline void projectTriangle(
		const GBTriangle& T,
		const GBVector3& axis,
		float& outMin,
		float& outMax)
	{
		outMin = outMax = GBDot(T.vertices[0], axis);
		for (int i = 1; i < 3; ++i)
		{
			float d = GBDot(T.vertices[i], axis);
			outMin = GBMin(outMin, d);
			outMax = GBMax(outMax, d);
		}
	}

	static bool GBCollisionBoxTriangleSAT(
		const GBBoxCollider& A,
		const GBTriangle& T,
		GBSATCollisionData& colData,
		float epsilon = 1e-6f)
	{
		const GBVector3 cA = A.transform.position;

		const GBVector3 Aaxes[3] = {
			A.transform.forward(),
			A.transform.right(),
			A.transform.up()
		};

		const GBVector3 a = A.halfExtents;

		colData.minOverlap = FLT_MAX;

		const GBVector3 triCenter =
			(T.vertices[0] + T.vertices[1] + T.vertices[2]) * (1.0f / 3.0f);

		auto testAxis = [&](const GBVector3& axis,
			GBSATCollisionType type,
			bool allowAsMin) -> bool
			{
				float lenSq = GBDot(axis, axis);
				if (lenSq < epsilon)
					return true;

				GBVector3 n = axis / std::sqrt(lenSq);

				// ---- Box projection ----
				float rA =
					GBAbs(GBDot(n, Aaxes[0])) * a.x +
					GBAbs(GBDot(n, Aaxes[1])) * a.y +
					GBAbs(GBDot(n, Aaxes[2])) * a.z;

				float c = GBDot(cA, n);
				float minA = c - rA;
				float maxA = c + rA;

				// ---- Triangle projection ----
				float minT, maxT;
				projectTriangle(T, n, minT, maxT);

				// ---- Compute penetration along axis ----
				// Signed distance from triangle to box along the axis
				float d1 = maxT - minA; // box min penetrates triangle
				float d2 = maxA - minT; // triangle max penetrates box
				float overlap = GBMin(d1, d2);

				if (overlap < 0.0f)
					return false;

				if (allowAsMin && overlap < colData.minOverlap && overlap > 0.0f)
				{
					colData.minOverlap = overlap;

					// Normal ALWAYS triangle → box
					if (GBDot(n, cA - triCenter) < 0.0f)
						n = -n;

					colData.bestAxis = n;
					colData.bestType =
						(type == GBSATCollisionType::EdgeEdge)
						? GBSATCollisionType::EdgeEdge
						: GBSATCollisionType::FaceA;
				}

				return true;
			};

		// ---- Box face axes (valid penetration axes) ----
		for (int i = 0; i < 3; ++i)
		{
			if (!testAxis(Aaxes[i],
				GBSATCollisionType::FaceB,
				true))
				return false;
		}

		// ---- Triangle face normal (separation test ONLY) ----
		if (!testAxis(T.normal, GBSATCollisionType::FaceA, false))
			return false;

		// ---- Edge × Edge (valid penetration axes) ----
		GBEdge edges[3];
		T.extractEdges(edges);

		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
			{
				GBVector3 axis = GBCross(Aaxes[i], edges[j].getBOutDirection());
				float lenSq = GBDot(axis, axis);
				if (lenSq < epsilon) continue;

				GBVector3 n = axis / std::sqrt(lenSq);

				if (!testAxis(axis, GBSATCollisionType::EdgeEdge, true))
					return false;
			}

		return true;
	}

	static bool GBManifoldBoxTriangle(GBTriangle& triangle,
		GBBoxCollider& box,
		GBSATCollisionData& colData,
		GBManifold& outManifold)
	{
		GBVector3 initPos = box.transform.position;
		box.transform.position -= initPos;
		triangle.applyTranslation(-initPos);

		outManifold.separation = colData.minOverlap;
		outManifold.normal = colData.bestAxis;
		GBVector3 normal = outManifold.normal;

		//if (colData.bestType == GBSATCollisionType::FaceA || colData.bestType == GBSATCollisionType::FaceB)
		{
			//B is always incident
			outManifold.pIncident = &box;
			normal = outManifold.normal;
		}
		//else

		GBManifold edgeManifold;
		{
			GBQuad qa = GBManifoldGeneration::GBBoxDirectionToQuad(box, -colData.bestAxis);
			edgeManifold.pIncident = (GBCollider*)&box;
			edgeManifold.isEdge = true;
			if (GBManifoldGeneration::GBManifoldTriangleQuadEdgeEdge(triangle, qa, colData.bestAxis, edgeManifold))
			{
				//box.transform.position += initPos;
				//triangle.applyTranslation(initPos);
				edgeManifold.applyTransformation(GBTransform(initPos, GBQuaternion()));
				//outManifold.normal = outManifold.contacts[0].normal;
				//return true;
			}
		}

		outManifold.clear();
		GBTransform boxInverse = box.transform.inverse();
		// Correct the best cardinal
		GBVector3 axisLocal = boxInverse.transformDirection(-colData.bestAxis);
		GBAABB referenceAABB(GBVector3::zero(), box.halfExtents);
		GBCardinal correctedCardinal = referenceAABB.directionToFace(axisLocal);
		triangle.applyTransformation(boxInverse);
		if (GBBuildTriangleAABBManifold(triangle, referenceAABB, correctedCardinal, outManifold))
		{
			outManifold.applyTransformation(box.transform);
			outManifold.normal = normal;
		}
		triangle.applyTransformation(box.transform);

		box.transform.position += initPos;
		triangle.applyTranslation(initPos);
		outManifold.applyTransformation(GBTransform(initPos, GBQuaternion()));
		outManifold.separation = outManifold.contacts[0].penetrationDepth;

		outManifold.normal = triangle.normal;
		if (GBDot(outManifold.normal, colData.bestAxis) < 0.0f)
			outManifold.normal *= -1.0f;
		for (int i = 0; i < outManifold.numContacts; i++)
		{
			if (GBDot(outManifold.normal, outManifold.contacts[i].normal) < 0.0f)
			{
				outManifold.contacts[i].normal *= -1.0f;
			}
		}

		if (colData.bestType == GBSATCollisionType::EdgeEdge)
		{
			outManifold.isEdge = true;
			if(edgeManifold.separation > outManifold.separation)
				outManifold.normal = colData.bestAxis;
			outManifold.combine(edgeManifold);
			outManifold.separation = GBMin(outManifold.contacts[0].penetrationDepth, outManifold.separation);
		}

		

		return outManifold.numContacts > 0;
	}

	static bool GBRaycastColliderVector(
		const GBVector3& rayOrigin,
		GBVector3 rayDir,
		const std::vector<GBCollider*>& colliders,
		GBContact& outContact
	)
	{
		float minDist = FLT_MAX;
		GBContact closestContact;
		GBContact temp;
		for (GBCollider* pCollider : colliders)
		{
			switch (pCollider->type)
			{
			case ColliderType::Box:
				if (GBRaycastOBB(*(GBBoxCollider*)pCollider, rayOrigin, rayDir, temp))
				{
					if (temp.penetrationDepth < minDist)
					{
						closestContact = temp;
						minDist = temp.penetrationDepth;
					}
				}
				break;
			case ColliderType::Sphere:
				if (GBRaycastSphere(*(GBSphereCollider*)pCollider, rayOrigin, rayDir, temp))
				{
					if (temp.penetrationDepth < minDist)
					{
						closestContact = temp;
						minDist = temp.penetrationDepth;
					}
				}
				break;
			}
		}

		if (minDist != FLT_MAX)
		{
			outContact = closestContact;
			return true;
		}

		return false;
	}

	static bool GBRaycastBodiesVector(
		const GBVector3& rayOrigin,
		GBVector3 rayDir,
		const std::vector<GBBody*>& bodies,
		GBContact& outContact
	)
	{
		float minDist = FLT_MAX;
		GBContact closestContact;
		GBContact temp;
		for (GBBody* pBody : bodies)
		{
			if (GBRaycastColliderVector(rayOrigin, rayDir, pBody->colliders, temp))
			{
				if (temp.penetrationDepth < minDist)
				{
					closestContact = temp;
					minDist = temp.penetrationDepth;
				}
			}
		}

		if (minDist != FLT_MAX)
		{
			outContact = closestContact;
			return true;
		}

		return false;
	}
};