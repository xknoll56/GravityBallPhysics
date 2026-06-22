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
	AABB,
	Capsule,
};

struct GBBody;
struct GBCell;

enum GBGeometryType
{
	COLLIDER = 0,
	STATIC = 1
};

struct GBGeometry
{
	GBGeometryType geometryType;
	virtual ~GBGeometry() = default;
};


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

	void setPosition(const GBVector3& position, bool doUpdateAABB = true);
	void setRotation(const GBQuaternion& rotation, bool doUpdateAABB = true);
	void translate(const GBVector3& translation, bool doUpdateAABB = true);
	void rotate(const GBQuaternion& rotationDelta, bool doUpdateAABB = true);
	void setTransform(const GBTransform& newTransform, bool doUpdateAABB = true);

	virtual float volume() const = 0;

	void* pData = (void*)nullptr;
	size_t id = (size_t)nullptr;
	std::vector<GBCell*> occupiedCells;
	bool isContacted = false;
};

struct GBHit {

	GBVector3 position;
	GBVector3 normal; // From collider A to B
	float distance;
	GBCollider* pReference = nullptr;
	GBCollider* pIncident = nullptr;
	GBHit()
		: position(0, 0, 0), normal(0, 0, 0), distance(0), pReference(nullptr), pIncident(nullptr)
	{
	}
	GBHit(const GBVector3& position, const GBVector3& normal, float penetrationDepth, GBCollider* pReference = nullptr, GBCollider* pIncident = nullptr)
		: position(position), normal(normal), distance(penetrationDepth), pReference(pReference), pIncident(pIncident)
	{
	}

	void applyTransformation(const GBTransform transform)
	{
		position = transform.transformPoint(position);
		normal = transform.transformDirection(normal);
	}


	GBVector3 getShapecastEndpoint(GBVector3 startPos, GBVector3 dir)
	{
		return startPos + dir * distance;
	}
};

typedef GBHit GBContact;
typedef GBHit GBRay;   // temporary alias

struct GBPlane;

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

const static int MAX_CONTACTS = 10;
struct GBManifold
{
	GBContact contacts[MAX_CONTACTS];
	int numContacts = 0;
	float separation = 0.0f;
	GBVector3 normal;
	GBBody* pIncident = nullptr;
	GBBody* pReference = nullptr;
	bool isEdge = false;
	bool isDynamicManifold = false;
	bool isJoint = false;
	const static int manifoldContactClamp = 4;
	GBSATCollisionData data;
	GBCardinal incidentFace = GBCardinal::None;
	GBCardinal referemceFace = GBCardinal::None;
	std::unordered_set<GBCollider*> referenceColliders;

	GBManifold()
	{

	}

	GBManifold(const GBContact& contact, GBBody* pIncident = nullptr, GBBody* pReference = nullptr) :
		pIncident(pIncident), pReference(pReference)
	{
		addContact(contact);
	}

	GBManifold operator=(const GBManifold other)
	{
		numContacts = other.numContacts;
		separation = other.separation;
		normal = other.normal;
		pIncident = other.pIncident;
		pReference = other.pReference;
		isEdge = other.isEdge;
		isDynamicManifold = other.isDynamicManifold;
		isJoint = other.isJoint;
		data = other.data;
		incidentFace = other.incidentFace;
		referemceFace = other.referemceFace;
		for (int i = 0; i < numContacts; i++)
			contacts[i] = other.contacts[i];

		return *this;
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
		float minDepth = contacts[0].distance;

		for (int i = 1; i < numContacts; i++)
		{
			if (contacts[i].distance < minDepth)
			{
				minDepth = contacts[i].distance;
				index = i;
			}
		}
		return index;
	}

	void sortByDepth()
	{
		// Insertion sort (N <= MAX_CONTACTS)
		// Order: deepest → shallowest
		for (int i = 1; i < numContacts; i++)
		{
			GBContact key = contacts[i];
			int j = i - 1;

			while (j >= 0 &&
				contacts[j].distance < key.distance)
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

	void addContact(const GBContact& contact, bool sorted = true, float epsilon = GBEpsilon)
	{
		if (!sorted)
		{
			if (numContacts < MAX_CONTACTS)
				contacts[numContacts++] = contact;
		}
		else
		{
			addContactSortedUnique(contact, GBEpsilon);
		}

		referenceColliders.insert(contact.pIncident);
		referenceColliders.insert(contact.pReference);
	}

	int countColliders(const GBBody& self) const
	{
		if (numContacts > 0)
		{
			std::unordered_set<GBCollider*> cols;
			for (int i = 0; i < numContacts; i++)
			{
				if (contacts[i].pIncident && contacts[i].pIncident->pBody  == &self)
					cols.insert(contacts[i].pIncident);
				if (contacts[i].pReference && contacts[i].pReference->pBody == &self)
					cols.insert(contacts[i].pReference);
			}
			return (int)GBMax(1, cols.size() - 1);
		}
		return 1;
	}

	void addContactSortedUnique(const GBContact& contact, float epsilon = GBEpsilon)
	{
		// Reject duplicates
		if (findDuplicate(contact, epsilon) != -1)
			return;

		if (numContacts < MAX_CONTACTS)
		{
			contacts[numContacts++] = contact;
		}
		else
		{
			// Replace shallowest if new contact is deeper
			int shallowest = findShallowest();

			if (contact.distance >
				contacts[shallowest].distance)
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

	void removeContact(int index)
	{
		if (index < 0 || index >= numContacts)
			return;

		for (int i = index; i < numContacts - 1; ++i)
		{
			contacts[i] = contacts[i + 1];
		}

		contacts[numContacts - 1] = GBContact{};
		--numContacts;
	}

	bool removeBodiesContact(const GBBody* body)
	{
		bool contactRemoved = false;
		for (int i = 0; i < numContacts; i++)
		{
			if ((contacts[i].pReference &&
				contacts[i].pReference->pBody == body) ||
				(contacts[i].pIncident &&
					contacts[i].pIncident->pBody == body))
			{
				removeContact(i);
				i--;
				contactRemoved = true;
			}
		}
		return contactRemoved;
	}

	bool removeCollidersContact(const GBCollider* pCol)
	{
		bool contactRemoved = false;
		for (int i = 0; i < numContacts; i++)
		{
			if ((contacts[i].pReference &&
				contacts[i].pReference == pCol) ||
				(contacts[i].pIncident &&
					contacts[i].pIncident == pCol))
			{
				removeContact(i);
				i--;
				contactRemoved = true;
			}
		}
		return contactRemoved;
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
		{
			contacts[i].normal = -contacts[i].normal;
			if (contacts[i].pIncident)
			{
				GBCollider* pTemp = contacts[i].pIncident;
				contacts[i].pIncident = contacts[i].pReference;
				contacts[i].pReference = pTemp;
			}
		}
	}

	void flipAndSwap()
	{
		GBBody* temp = pIncident;
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

	float supportSize() const
	{
		if (numContacts < 2)
			return 0.0f;

		const static float fakeEdgeWidth = 0.05f;
		if (numContacts == 2)
			return (contacts[1].position - contacts[0].position).length() * fakeEdgeWidth;

		// 3 or more contacts → area of first triangle
		GBVector3 A = contacts[0].position;
		GBVector3 B = contacts[1].position;
		GBVector3 C = contacts[2].position;
		A = A.xyComponent();
		B = B.xyComponent();
		C = C.xyComponent();

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
		float sep = separation == 0.0f ? contacts[0].distance : separation;
		return GBContact(avg, contacts[0].normal, sep);
	}

	void combine(const GBManifold& other, bool updateNormal = false)
	{
		if (updateNormal)
		{
			if (other.separation > separation)
				useNormal(other);
		}
		for (int i = 0; i < other.numContacts; i++)
		{
			addContact(other.contacts[i]);
		}
	}

	bool combineSupports(const GBManifold& other, bool updateNormal = true, float epsilon = GBEpsilon, float upwardThreshold = 0.05f)
	{
		bool supportAdded = false;
		if (updateNormal)
		{
			if (other.separation > separation)
				useNormal(other);
		}
		for (int i = 0; i < other.numContacts; i++)
		{
			if (other.contacts[i].normal.z > upwardThreshold)
			{
				addContact(other.contacts[i], true, upwardThreshold);
				supportAdded = true;
			}
		}
		return supportAdded;
	}

	void useNormal(const GBManifold& other)
	{
		normal = other.normal;
		isEdge = other.isEdge;
		separation = other.separation;
		pReference = other.pReference;
		pIncident = other.pIncident;
	}

	void useNormal(const GBContact& contact)
	{
		normal = contact.normal;
		separation = GBAbs(contact.distance);
		alignContactsWithNormal();
	}

	void alignContactsWithNormal()
	{
		for (int i = 0; i < numContacts; i++)
		{
			contacts[i].normal = GBAlign(contacts[i].normal, normal);
			contacts[i].distance = GBAbs(contacts[i].distance);
		}
	}

	void alignNormalWithIncident();

	bool equalPair(const GBManifold& other) const
	{
		return ((pIncident == other.pIncident) && (pReference == other.pReference)) || ((pIncident == other.pReference) && (pReference == other.pIncident));
	}

	GBBody* getOtherBody(GBBody* knownBody) const
	{
		if (pIncident == knownBody)
			return pReference;
		else
			return pIncident;
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

	inline void pruneBehindPlane(const GBPlane& plane);

	void pruneOutsideAABB(GBAABB aabb, float epsilon = 1e-5);

	inline void pruneWithNormal(float epsilon = 1e-5);

	void correctPenetrations()
	{
		for (int i = 0; i < numContacts; i++)
		{
			contacts[i].distance = GBAbs(contacts[i].distance);
		}
	}


	GBVector3 getFeatureBasedNormal(const int i) const
	{
		if (numContacts < 3)
			return contacts[i].normal;
		else
			return normal;
	}

	float maxUpContact(GBContact& outContact) const
	{
		float max = -FLT_MAX;
		for (int i = 0; i < numContacts; i++)
		{
			float upness = GBDot(contacts[i].normal, GBVector3::up());
			if (upness > max)
			{
				outContact = contacts[i];
				max = upness;
			}
		}

		return max;

	}

	GBManifold asSupportManifold()
	{
		GBManifold support;
		for (int i = 0; i < numContacts; i++)
		{
			if (contacts[i].normal.z > 0.0f)
			{
				support.addContact(contacts[i], true, 0.005f);
			}
		}
		support.useNormal(support.contacts[0]);
		return support;
	}

	bool containsSupportOrigin(GBVector3 pos, float tolerance = 0.005f, float minRestingSlope = 0.7071f);

	void capContacts(int capacity = manifoldContactClamp)
	{
		if (numContacts > capacity)
			numContacts = capacity;
	}


	void setContactColliders(GBCollider* _pIncident, GBCollider* _pReference)
	{
		for (int i = 0; i < numContacts; i++)
		{
			contacts[i].pIncident = _pIncident;
			contacts[i].pReference = _pReference;
		}
	}
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
	std::vector<GBStaticGeometry*> staticGeometries;

	// Physics properties
	float mass = 1.0f;
	float invMass = 1.0f;

	GBVector3 velocity;            // linear velocity
	GBVector3 angularVelocity;     // rotational velocity in local space
	GBVector3 forceAccum;          // accumulated force
	GBVector3 torqueAccum;         // accumulated torque

	GBVector3 prevFrameVelocity;
	GBVector3 prevFrameAngularVelocity;

	// Material properties
	float restitution = 0.1f;    // bounciness, 0 = no bounce, 1 = full bounce
	float staticFriction = 0.5f; // prevents sliding
	float dynamicFriction = 0.3f; // sliding friction

	GBVector3 inertia;        // principal moments of inertia (local space)
	GBVector3 invInertia;     // inverse of inertia

	// Inside BrickBody
	bool isSleeping = false;
	float sleepTimer = 0.0f;

	
	bool onAwake = false;
	bool isStatic = false;
	float awakeTimer = 0.0f;
	
	GBVector3 prevPosition;
	GBQuaternion prevRotation;

	GBAABB aabb;


	bool isDirty = true;

	bool doesTriggerGridRebuild = true;

	bool ignoreSample = false;
	int id;
	
	bool firstWake = true;
	void* pData = (void*)nullptr;

	GBManifold frameManifold;
	bool doTestSupports = false;

	bool isGrounded = false;

	bool isPlayerController = false;
	float playerSlopeValue = 0.7f;

	uint32_t layer = 0xFFFFFFFF; // all 32 bits set → belongs to all layers
	uint32_t mask = 0xFFFFFFFF; // collides with all layers

	bool skipSolverForBody = false;

	bool isKinematic = false;
	bool ignoreKinematicVelocityClamp = false;
	bool useGravity = true;


	bool sharesLayer(const GBBody& other)
	{
		return ((mask & other.layer) && (layer & other.mask));
	}

	bool isOnLayer(uint32_t testLayer) const
	{
		return (layer & testLayer) != 0;
	}

	static const int maxRecursiveDepth = 5;


	GBBody(float mass = 1.0f, const GBVector3& halfExtents = { 0.5f, 0.5f, 0.5f })
		: mass(mass)
	{

		GBVector3 zero = GBVector3::zero();
		invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
		velocity = zero;
		angularVelocity = zero;
		forceAccum = zero;
		torqueAccum = zero;

		prevFrameVelocity = zero;
		prevFrameAngularVelocity = zero;


		// Default material
		restitution = 0.1f;
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

		setInertia(aabb.halfExtents);
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
		if (isKinematic)
			return false;
		if (isTrigger)
			return false;
		return true;
	}

	bool isMovable()
	{
		return !isStatic && !isTrigger;
	}

	bool isAwake()
	{
		return isMovable() && !isSleeping;
	}

	bool isTrigger = false;

	void wakeRecursive(std::unordered_set<GBBody*>& visited, int& depth, const int maxDepth)
	{
		if (visited.count(this)) return;
		visited.insert(this);

		wake();

		depth += 1;
		if (depth > maxDepth)
			return;

		for (GBBody* b : dynamicBodies)
			if (b && !b->isStatic && b->frameManifold.size() < 1)
				b->wakeRecursive(visited,depth, maxDepth);
	}

	// wrapper
	void wakeIsland(const int maxDepth = 1)
	{
		std::unordered_set<GBBody*> visited;
		int depth = 0;
		wakeRecursive(visited, depth, maxDepth);
	}

	bool hasStaticAttachmentRecursive(std::unordered_set<const GBBody*>& visited, int depth, int maxDepth) const
	{
		if (visited.count(this) || depth>=maxDepth)
			return false;

		visited.insert(this);

		if (isStatic || isSleeping || staticGeometries.size()>0)
			return true;

		depth++;

		if (visited.size() < maxRecursiveDepth)
		{
			for (GBBody* b : dynamicBodies)
			{
				if (b && b->hasStaticAttachmentRecursive(visited, depth, maxDepth))
					return true;
			}
		}

		return false;
	}

	bool hasStaticAttachment(int maxDepth = 4) const
	{
		std::unordered_set<const GBBody*> visited;
		int depth = 0;
		return hasStaticAttachmentRecursive(visited, depth, maxDepth);
	}

	void wake()
	{
		isSleeping = false;
		sleepTimer = 0.0f;
		awakeTimer = 0.0f;
		isGrounded = false;
		staticGeometries.clear();

		std::unordered_set<GBBody*> contactBodies;
		for (int i = 0; i < frameManifold.numContacts; i++)
		{
			if (frameManifold.contacts[i].pIncident && frameManifold.contacts[i].pReference)
			{
				GBBody* pOther = frameManifold.getOtherBody(this);
				if(pOther)
					contactBodies.insert(pOther);
			}
		}
		for (GBBody* pOther : contactBodies)
			pOther->frameManifold.removeBodiesContact(pOther);
		frameManifold.reset();
	}


	void addDynamicContact(GBBody* other, bool awaken = false, bool doWakeIsland = false)
	{
		if (std::find(dynamicBodies.begin(),
			dynamicBodies.end(),
			other) == dynamicBodies.end())
		{
			dynamicBodies.push_back(other);
		}

		if (awaken && isSleeping)
		{
			if (doWakeIsland)
				wakeIsland();
			else
				wake();
		}

		if (awaken && other->isSleeping)
		{
			if (doWakeIsland)
				other->wakeIsland();
			else
				other->wake();
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
		if (isSleeping || isStatic || isTrigger)
		{
			updateColliders();
			clearForces(); // make sure forces aren't accumulating
			return;
		}

		// Linear
		GBVector3 acceleration = forceAccum * invMass;
		velocity += acceleration * dt;


		// Angular
		GBVector3 angularAcc = {
			torqueAccum.x * invInertia.x,
			torqueAccum.y * invInertia.y,
			torqueAccum.z * invInertia.z
		};
		angularVelocity += angularAcc * dt;


		// Clear accumulators
		clearForces();

		// Sync colliders
		updateColliders();
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
		staticGeometries.clear();
	}

	GBVector3 realVelocity(float dt)
	{
		return (transform.position - prevPosition) / dt;
	}

	GBVector3 realAngularVelocity(float dt)
	{
		return GBQuaternion::toAngularVelocity(prevRotation, transform.rotation, dt);
	}

	float getSupportArea() const
	{
		return aabb.halfExtents.x * aabb.halfExtents.y * 4.0f;
	}

	float getSupportRatio() const
	{
		return frameManifold.supportSize() / getSupportArea();
	}
};


inline void GBCollider::setPosition(const GBVector3& position, bool doUpdateAABB)
{
	transform.setPosition(position);
	if (pBody)
	{
		localTransform.position = transform.position - pBody->transform.position;
	}
	if (doUpdateAABB)
		updateAABB();
}

inline void GBCollider::setRotation(const GBQuaternion& rotation, bool doUpdateAABB)
{
	transform.setRotation(rotation);

	if (pBody)
	{
		localTransform = pBody->transform.inverse() * transform;
	}

	if (doUpdateAABB)
		updateAABB();
}

inline void GBCollider::translate(const GBVector3& translation, bool doUpdateAABB)
{
	transform.translate(translation);
	if (doUpdateAABB)
		updateAABB();
}
inline void GBCollider::rotate(const GBQuaternion& rotationDelta, bool doUpdateAABB)
{
	transform.rotate(rotationDelta);

	if (pBody)
	{
		localTransform = pBody->transform.inverse() * transform;
	}

	if (doUpdateAABB)
		updateAABB();
}

inline void GBCollider::setTransform(const GBTransform& newTransform, bool doUpdateAABB)
{
	transform = newTransform;

	if (pBody)
	{
		localTransform = pBody->transform.inverse() * transform;
	}

	if (doUpdateAABB)
		updateAABB();
}

inline void GBManifold::flipAndSwapIfOnTop()
{
	if (pIncident->transform.position.z < pReference->transform.position.z)
	{
		flipAndSwap();
	}
}

inline void GBManifold::flipAndSwapIfContactOnTop(GBVector3 manifoldNormal)
{
	if (pReference && pIncident)
	{
		if (GBDot(manifoldNormal, GBVector3::up()) < 0.00f)
			manifoldNormal *= -1.0f;
		float projRef = GBDot(manifoldNormal, pReference->transform.position);
		float projInc = GBDot(manifoldNormal, pIncident->transform.position);
		if (projInc < projRef)
		{
			flipAndSwap();
		}
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

struct GBCapsuleCollider : public GBCollider {
	float radius;
	float height;
	GBCapsuleCollider()
		: GBCollider(), radius(0.5f), height(1.0f)
	{
		type = ColliderType::Capsule;
		updateAABB();
	}
	GBCapsuleCollider(float radius, float height)
		: GBCollider(), radius(radius), height(height)
	{
		type = ColliderType::Capsule;
		updateAABB();
	}

	void extractSphereLocations(GBVector3& topSphere, GBVector3& bottomSphere, GBVector3* pUp = nullptr) const
	{
		GBVector3 up = transform.up();
		if(pUp)
			*pUp = up;
		topSphere = transform.position + up * (height * 0.5f);
		bottomSphere = transform.position - up * (height * 0.5f);
	}

	GBEdge getSphereToSphereEdge() const
	{
		GBVector3 lower, upper;
		extractSphereLocations(upper, lower);
		return GBEdge(lower, upper);
	}

	void updateAABB() override
	{
		GBVector3 topSphere, bottomSphere;
		extractSphereLocations(topSphere, bottomSphere);
		GBAABB top = GBAABB(topSphere, GBVector3(radius, radius, radius));
		GBAABB bot = GBAABB(bottomSphere, GBVector3(radius, radius, radius));
		aabb = top;
		aabb.includeAABB(bot);
	}
	float volume() const  override
	{
		return GB_PI * radius * radius * ((4.0f / 3.0f) * radius + height);
	}

	static GBCapsuleCollider capsuleFromEdge(const GBEdge& edge, float radius = 0.5f)
	{
		GBCapsuleCollider cc;
		cc.height = edge.length();
		cc.transform.position = edge.center();
		cc.transform.rotation = GBQuaternion::fromToRotation(
			GBVector3(0, 0, 1),
			edge.getBOutDirection()
		);
		cc.radius = radius;
		return cc;
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

void GBManifold::pruneBehindPlane(const GBPlane& plane)
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

void GBManifold::pruneOutsideAABB(GBAABB aabb, float epsilon)
{
	GBManifold pruned;
	pruned.useNormal(*this);
	aabb.halfExtents += {epsilon, epsilon, epsilon};
	for (int i = 0; i < numContacts; i++)
	{
		if (aabb.isPointContainedWithError(contacts[i].position, epsilon))
			pruned.addContact(contacts[i]);
	}
	clear();
	combine(pruned);
}

void GBManifold::alignNormalWithIncident()
{
	if (pIncident && pReference)
	{
		normal = GBAlign(normal, pIncident->transform.position - pReference->transform.position);
	}
}

void GBManifold::pruneWithNormal(float epsilon)
{
	GBManifold pruned;
	pruned.useNormal(*this);
	for (int i = 0; i < numContacts; i++)
	{
		if ((1 - GBDot(normal, contacts[i].normal) <= epsilon))
		{
			pruned.addContact(contacts[i]);
		}
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

	void expandArea(float epsilon = GBEpsilon)
	{
		xSize += epsilon;
		ySize += epsilon;
	}
};

struct GBTriangle : GBStaticGeometry
{
	GBVector3 vertices[3];
	GBEdge edges[3];
	GBVector3 normal;

	GBTriangle(const GBVector3& v0, const GBVector3& v1, const GBVector3& v2) :
		GBStaticGeometry(GBStaticGeometryType::TRIANGLE)
	{
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		normal = GBCross(v1 - v0, v2 - v0).normalized();

		edges[0] = GBEdge(vertices[0], vertices[1]);
		edges[1] = GBEdge(vertices[1], vertices[2]);
		edges[2] = GBEdge(vertices[2], vertices[0]);
	}

	GBPlane toPlane() const
	{
		return GBPlane(normal, vertices[0]);
	}



	GBEdge closestEdgeToPoint(const GBVector3& p) const
	{
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

bool GBManifold::containsSupportOrigin(GBVector3 pos, float tolerance, float minRestingSlope)
{
	bool negX = false;
	bool posX = false;
	bool posY = false;
	bool negY = false;
	float maxUp = -FLT_MAX;

	if (numContacts > 2)
	{
		for (int i = 2; i < numContacts; i++)
		{
			GBTriangle tri = { contacts[i - 2].position, contacts[i - 1].position, contacts[i].position };
			GBPlane p = tri.toPlane();
			if (p.normal.z < 0.0f)
				p.normal = -p.normal;
			maxUp = GBMax(maxUp, GBDot(p.normal, GBVector3::up()));
			GBVector3 right = GBCross(GBEdge(tri.vertices[i - 2], tri.vertices[i - 1]).getAOutDirection(), p.normal).normalized();
			GBVector3 forward = GBCross(right, p.normal).normalized();

			GBVector3 local = contacts[i].position - pos;
			float localRight = GBDot(local, right);
			float localForward = GBDot(local, forward);
			if (localRight < 0.0f)
				negX = true;
			else
				posX = true;

			if (localForward < 0.0f)
				negY = true;
			else
				posY = true;
		}
		return (negX && posX) && (negY && posY) && maxUp > minRestingSlope;
	}
	else if (numContacts == 2)
	{
		GBEdge supportEdge(contacts[0].position, contacts[1].position);
		GBVector3 pointOnLine = supportEdge.closestPointOnLine(pos) - pos;
		GBVector3 edgeDir = supportEdge.getAOutDirection();
		GBVector3 right = GBCross(edgeDir, contacts[0].normal).normalized();
		GBVector3 forward = GBCross(edgeDir, right).normalized();

		float localRight = GBDot(pointOnLine, right);
		float localForward = GBDot(pointOnLine, forward);
		GBVector3 up = GBCross(right, forward);
		if (up.z < 0.0f)
			up = -up;

		maxUp = GBMax(maxUp, GBDot(up, GBVector3::up()));
		return GBAbs(localRight) < tolerance && GBAbs(localForward) < tolerance && maxUp > minRestingSlope;
	}
	else if(numContacts == 1)
	{
		GBVector3 dp = pos - contacts[0].position;
		GBVector3 right; 
		GBVector3 forward;
		GBVector3 normDp = dp.normalized();
		if (normDp.z < 0.0f)
			normDp = -normDp;
		if (GBAbs(GBDot(dp, GBVector3::up()) > (1.0f - GBLargeEpsilon)))
		{
			right = GBCross(GBVector3::forward(), normDp).normalized();
		}
		else
		{
			right = GBCross(GBVector3::up(), normDp).normalized();
		}
		forward = GBCross(right, normDp).normalized();
		float localRight = GBDot(dp, right);
		float localForward = GBDot(dp, forward);

		
		maxUp = GBMax(maxUp, GBDot(normDp, GBVector3::up()));

		return GBAbs(localRight) < tolerance && GBAbs(localForward) < tolerance && maxUp > minRestingSlope;
		
	}
	return false;
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

	GBVector3 cardinalToDir(GBCardinal dir)
	{
		switch (dir)
		{
		case GBCardinal::NegX:
			return -transform.forward();
			break;
		case GBCardinal::PosX:
			return transform.forward();
			break;
		case GBCardinal::NegY:
			return -transform.right();
			break;
		case GBCardinal::PosY:
			return transform.right();
			break;
		case GBCardinal::NegZ:
			return -transform.up();
			break;
		case GBCardinal::PosZ:
			return transform.up();
			break;
		}
	}

	float volume() const  override
	{
		return halfExtents.x * halfExtents.y * halfExtents.z;
	}

	bool isPointContained(const GBVector3& point, float epsilon = 1e-4f)
	{
		const GBVector3 dp = point - transform.position;

		return
			GBAbs(GBDot(transform.forward(), dp)) <= halfExtents.x * (1.0f + epsilon) &&
			GBAbs(GBDot(transform.right(), dp)) <= halfExtents.y * (1.0f + epsilon) &&
			GBAbs(GBDot(transform.up(), dp)) <= halfExtents.z * (1.0f + epsilon);
	}

	static GBBoxCollider boxFromAABB(const GBAABB& aabb)
	{
		GBBoxCollider box;
		box.transform.position = aabb.center;
		box.transform.rotation = GBQuaternion();
		box.halfExtents = aabb.halfExtents;
		return box;
	}
};


struct GBParticle
{
	GBVector3 position;
	GBVector3 prevPosition;
	GBVector3 acceleration;
	float mass;
	bool pinned;

	GBParticle(GBVector3 position, float mass = 1.0f, bool pinned = false) :
		position(position), prevPosition(position), acceleration(GBVector3()),
		mass(mass), pinned(pinned)
	{

	}
};


struct GBSpring
{
	int p1;          // index of first particle
	int p2;          // index of second particle
	float restLength; // original distance between particles
	float stiffness;  // spring constant

	GBSpring(int p1, int p2, float restLength, float stiffness) :
		p1(p1), p2(p2), restLength(restLength), stiffness(stiffness)
	{

	}
};

struct GBCloth
{
	int width;
	int height;
	float spacing;
	float particleRadius;
	std::vector<GBParticle> particles;
	std::vector<GBSpring> springs;

	// Index will start moving along the horizontal 
	// creating rows.
	int index(int x, int y)
	{
		return y * width + x;
	}
};