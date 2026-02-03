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

struct GBPlane;

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

	void pruneBehindPlane(const GBPlane& plane);
};

struct GBStaticGeometry;

enum Layers : uint32_t {
	LAYER_ALL = 0xFFFFFFFF,
	LAYER_STATIC = 1 << 0,
	LAYER_DYNAMIC = 1 << 1,
	LAYER_DYNAMIC_SPHERE = 1 << 2,
	LAYER_DYNAMIC_BOX = 1 << 3,
	LAYER_STATIC_DYNAMIC_SPHERE = LAYER_STATIC | LAYER_DYNAMIC_SPHERE,
};

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

	uint32_t layer = 0xFFFFFFFF; // all 32 bits set → belongs to all layers
	uint32_t mask = 0xFFFFFFFF; // collides with all layers

	bool skipSolverForBody = false;

	bool sharesLayer(const GBBody& other)
	{
		return ((mask & other.layer) && (layer & other.mask));
	}

	bool isOnLayer(uint32_t testLayer) const
	{
		return (layer & testLayer) != 0;
	}

	struct PointConstraint
	{
		GBBody* pBody = nullptr;
		GBBody* pOtherBody = nullptr;
		GBVector3 worldConstraintPoint;  // fixed world-space point
		GBVector3 localConstraintPoint;  // fixed in body space
		GBVector3 otherLocalConstraintPoint;
		float breakingThreshold = 0.05f;

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
				float len = deltaWorldPoint.length();
				if (len > breakingThreshold)
				{
					GBVector3 deltaN = deltaWorldPoint.normalized();
					worldConstraintPoint = worldConstraintPoint + deltaN * breakingThreshold;
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

	static const int maxRecursiveDepth = 5;

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

		if (visited.size() < maxRecursiveDepth)
		{
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
		resetConstraints();

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

		if (visited.size() < maxRecursiveDepth)
		{
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
				transform * c->localTransform;   // compose (pos + rot + scale)

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
	uint32_t id;

	GBStaticGeometry(GBStaticGeometryType type) :
		type(type)
	{

	}
	GBStaticGeometry() :
		type(GBStaticGeometryType::NONE)
	{

	}

	uint32_t layer = 0xFFFFFFFF; // all 32 bits set → belongs to all layers
	uint32_t mask = 0xFFFFFFFF; // collides with all layers

	bool sharesLayer(const GBBody& other)
	{
		return ((mask & other.layer) && (layer & other.mask));
	}

	bool isOnLayer(uint32_t testLayer) const
	{
		return (layer & testLayer) != 0;
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

inline void GBManifold::pruneBehindPlane(const GBPlane& plane)
{
	GBManifold pruned;
	pruned.useNormal(*this);
	for (int i = 0; i < numContacts; i++)
	{
		if (GBDot(contacts[i].position - plane.point, plane.normal) < 0)
			pruned.addContact(contacts[i]);
	}
	clear();
	combine(pruned);
}

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
