#pragma once

struct GBEdge {
	GBVector3 a;
	GBVector3 b;

	GBEdge()
		: a(0, 0, 0), b(0, 0, 0)
	{
	}

	float length() const
	{
		return (b - a).length();
	}

	GBEdge(const GBVector3& a, const GBVector3& b)
		: a(a), b(b)
	{
	}

	bool epsilonEqual(const GBEdge& rhs, float epsilon = 1e-6f) const
	{
		// Check if endpoints match in the same order or reversed
		return (a.epsilonEqual(rhs.a, epsilon) && b.epsilonEqual(rhs.b, epsilon)) ||
			(a.epsilonEqual(rhs.b, epsilon) && b.epsilonEqual(rhs.a, epsilon));
	}

	// Unary minus operator
	GBEdge operator-() const
	{
		GBEdge reversed;
		reversed.a = b;
		reversed.b = a;
		return reversed;
	}


	GBVector3 getAOutDirection() const
	{
		return (a - b).normalized();
	}

	GBVector3 getBOutDirection() const
	{
		return (b - a).normalized();
	}

	GBVector3 center() const
	{
		return (a + b) * 0.5f;
	}

	void reverse()
	{
		GBVector3 temp = a;
		a = b;
		b = temp;
	}

	// Returns true if the closest point is strictly between a and b
	bool closestPointBetween(const GBVector3& P, GBVector3& outClosest) const
	{
		GBVector3 AB = b - a;
		float abLengthSq = AB.lengthSquared();

		// Degenerate edge (zero length)
		if (abLengthSq < 1e-8f)
		{
			outClosest = a;
			return false;
		}

		// Project P onto the line AB
		float t = (P - a).dot(AB) / abLengthSq;

		// Clamp t to [0,1]
		float clampedT = std::clamp(t, 0.0f, 1.0f);

		// Compute the closest point
		outClosest = a + AB * clampedT;

		// Return true only if the closest point is strictly between a and b
		// Allowing tiny epsilon to avoid floating point errors
		const float epsilon = 1e-6f;
		if (clampedT > epsilon && clampedT < 1.0f - epsilon)
			return true;

		return false;
	}

	// Returns the closest point on the infinite line defined by the edge (a->b)
// Does NOT require the point to be between a and b
	GBVector3 closestPointOnLine(const GBVector3& P) const
	{
		GBVector3 AB = b - a;
		float abLengthSq = AB.lengthSquared();

		// Degenerate edge (zero length)
		if (abLengthSq < 1e-8f)
			return a;

		// Project P onto the line (no clamping)
		float t = (P - a).dot(AB) / abLengthSq;

		// Compute the closest point
		return a + AB * t;
	}

	static inline bool closestEdgeBetween(
		const GBEdge& e1,
		const GBEdge& e2,
		GBEdge& outEdge,
		bool ensureCrossing = false)
	{
		// Direction vectors (NOT normalized)
		GBVector3 d1 = e1.b - e1.a;
		GBVector3 d2 = e2.b - e2.a;

		float len1Sq = d1.lengthSquared();
		float len2Sq = d2.lengthSquared();
		if (len1Sq < 1e-6f || len2Sq < 1e-6f)
			return false; // degenerate edges

		GBVector3 cross = GBCross(d1, d2);
		float denom = cross.lengthSquared();
		if (denom < 1e-6f)
			return false; // parallel or nearly parallel

		// Solve closest points on the infinite lines
		float s = GBDot(GBCross(e2.a - e1.a, d2), cross) / denom;
		float t = GBDot(GBCross(e1.a - e2.a, d1), -cross) / denom;

		if (ensureCrossing)
		{
			// s,t are now in [0,1] segment space
			if (s < 0.f || s > 1.f ||
				t < 0.f || t > 1.f)
				return false;
		}

		outEdge = GBEdge(
			e1.a + d1 * s,
			e2.a + d2 * t
		);

		return true;
	}

	void applyTransform(const GBTransform& transform)
	{
		a = transform.transformPoint(a);
		b = transform.transformPoint(b);
	}

};


struct GBAABB
{
	GBVector3 center;
	GBVector3 halfExtents;

	GBAABB()
		: center{ 0,0,0 }, halfExtents{ 0.5f,0.5f,0.5f }
	{
	}

	GBAABB(GBVector3 center, GBVector3 halfExtents)
		: center(center), halfExtents(halfExtents)
	{
	}

	static GBAABB fromLowAndHigh(GBVector3 low, GBVector3 high)
	{
		GBAABB aabb;
		aabb.center = (low + high) * 0.5f;
		aabb.halfExtents = (high - aabb.center);
		return aabb;
	}

	GBVector3 high() const { return center + halfExtents; }
	GBVector3 low() const { return center - halfExtents; }

	void setHigh(GBVector3 high)
	{
		GBVector3 oldLow = low();                // capture BEFORE changes
		center = (oldLow + high) * 0.5f;
		halfExtents = high - center;
	}

	void setLow(GBVector3 low)
	{
		GBVector3 oldHigh = high();              // capture BEFORE changes
		center = (low + oldHigh) * 0.5f;
		halfExtents = oldHigh - center;
	}

	void includePoint(GBVector3 point)
	{
		// Compute new bounds FIRST
		GBVector3 newLow = GBMin(point, low());
		GBVector3 newHigh = GBMax(point, high());

		// Rebuild the AABB
		center = (newLow + newHigh) * 0.5f;
		halfExtents = newHigh - center;
	}

	bool containsPoint(const GBVector3& p) const
	{
		GBVector3 l = low();
		GBVector3 h = high();
		return (p.x >= l.x && p.x <= h.x &&
			p.y >= l.y && p.y <= h.y &&
			p.z >= l.z && p.z <= h.z);
	}


	bool intersects(const GBAABB& other) const
	{
		GBVector3 aLow = low();
		GBVector3 aHigh = high();
		GBVector3 bLow = other.low();
		GBVector3 bHigh = other.high();

		return (aLow.x <= bHigh.x && aHigh.x >= bLow.x &&
			aLow.y <= bHigh.y && aHigh.y >= bLow.y &&
			aLow.z <= bHigh.z && aHigh.z >= bLow.z);
	}


	void includeAABB(const GBAABB& other)
	{
		GBVector3 newLow = GBMin(low(), other.low());
		GBVector3 newHigh = GBMax(high(), other.high());

		center = (newLow + newHigh) * 0.5f;
		halfExtents = newHigh - center;
	}


	GBVector3 overlap(const GBAABB& other) const
	{
		GBVector3 aLow = low();
		GBVector3 aHigh = high();
		GBVector3 bLow = other.low();
		GBVector3 bHigh = other.high();

		return {
			GBMin(aHigh.x, bHigh.x) - GBMax(aLow.x, bLow.x),
			GBMin(aHigh.y, bHigh.y) - GBMax(aLow.y, bLow.y),
			GBMin(aHigh.z, bHigh.z) - GBMax(aLow.z, bLow.z)
		};
	}

	GBVector3 size() const
	{
		return halfExtents * 2.0f;
	}

	void includePointFast(const GBVector3& p)
	{
		center = (center + p) * 0.5f;
		halfExtents = GBMax(halfExtents, GBAbs(p - center));
	}

	GBAABB& operator=(const GBAABB& other)
	{
		if (this == &other)
			return *this;  // self-assignment check (good practice)

		center = other.center;
		halfExtents = other.halfExtents;
		return *this;
	}

	GBCardinal directionToFace(GBVector3 dir) const
	{
		GBVector3 absDir = GBAbs(dir);
		if (absDir.x > absDir.y && absDir.x > absDir.z)
			return dir.x > 0 ? GBCardinal::PosX : GBCardinal::NegX;
		else if (absDir.y > absDir.z)
			return dir.y > 0 ? GBCardinal::PosY : GBCardinal::NegY;
		else
			return dir.z > 0 ? GBCardinal::PosZ : GBCardinal::NegZ;
	}

	void directionToClosestFaces(GBVector3 dir, GBCardinal outCardinals[3]) const
	{
		outCardinals[0] = dir.x > 0 ? GBCardinal::PosX : GBCardinal::NegX;
		outCardinals[1] = dir.y > 0 ? GBCardinal::PosY : GBCardinal::NegY;
		outCardinals[2] = dir.z > 0 ? GBCardinal::PosZ : GBCardinal::NegZ;
	}

	GBEdge dirToClosestEdge(const GBVector3& dir) const
	{
		GBVector3 p = dir;
		GBVector3 e = halfExtents;

		// Get absolute values
		float ax = GBAbs(p.x);
		float ay = GBAbs(p.y);
		float az = GBAbs(p.z);

		// Identify the axis along which the edge runs (smallest component)
		int edgeAxis; // 0=x, 1=y, 2=z
		if (ax <= ay && ax <= az)       edgeAxis = 0; // along X
		else if (ay <= ax && ay <= az)  edgeAxis = 1; // along Y
		else                             edgeAxis = 2; // along Z

		// Signs of the other two axes
		float sx = GBSign(p.x);
		float sy = GBSign(p.y);
		float sz = GBSign(p.z);

		GBEdge out;

		switch (edgeAxis)
		{
		case 0: // edge along X → varies X, fix Y/Z
			out.a = center + GBVector3(-e.x, sy * e.y, sz * e.z);
			out.b = center + GBVector3(e.x, sy * e.y, sz * e.z);
			break;

		case 1: // edge along Y → varies Y, fix X/Z
			out.a = center + GBVector3(sx * e.x, -e.y, sz * e.z);
			out.b = center + GBVector3(sx * e.x, e.y, sz * e.z);
			break;

		case 2: // edge along Z → varies Z, fix X/Y
			out.a = center + GBVector3(sx * e.x, sy * e.y, -e.z);
			out.b = center + GBVector3(sx * e.x, sy * e.y, e.z);
			break;
		}

		return out;
	}

	GBEdge pointToClosestEdge(GBVector3 point) const
	{
		GBVector3 p = point - center;
		GBVector3 e = halfExtents;

		float dx = e.x - GBAbs(p.x);
		float dy = e.y - GBAbs(p.y);
		float dz = e.z - GBAbs(p.z);

		GBEdge out;

		// Find the two closest faces
		if (dx <= dy && dx <= dz)
		{
			// Closest to ±X face, choose between Y/Z for second
			float sy = GBSign(p.y);
			float sz = GBSign(p.z);

			if (dy < dz)
			{
				// X + Y faces → edge parallel to Z
				out.a = center + GBVector3(e.x * GBSign(p.x), e.y * sy, -e.z);
				out.b = center + GBVector3(e.x * GBSign(p.x), e.y * sy, e.z);
			}
			else
			{
				// X + Z faces → edge parallel to Y
				out.a = center + GBVector3(e.x * GBSign(p.x), -e.y, e.z * sz);
				out.b = center + GBVector3(e.x * GBSign(p.x), e.y, e.z * sz);
			}
		}
		else if (dy <= dx && dy <= dz)
		{
			// Closest to ±Y face
			float sx = GBSign(p.x);
			float sz = GBSign(p.z);

			if (dx < dz)
			{
				// Y + X → edge parallel to Z
				out.a = center + GBVector3(e.x * sx, e.y * GBSign(p.y), -e.z);
				out.b = center + GBVector3(e.x * sx, e.y * GBSign(p.y), e.z);
			}
			else
			{
				// Y + Z → edge parallel to X
				out.a = center + GBVector3(-e.x, e.y * GBSign(p.y), e.z * sz);
				out.b = center + GBVector3(e.x, e.y * GBSign(p.y), e.z * sz);
			}
		}
		else
		{
			// Closest to ±Z face
			float sx = GBSign(p.x);
			float sy = GBSign(p.y);

			if (dx < dy)
			{
				// Z + X → edge parallel to Y
				out.a = center + GBVector3(e.x * sx, -e.y, e.z * GBSign(p.z));
				out.b = center + GBVector3(e.x * sx, e.y, e.z * GBSign(p.z));
			}
			else
			{
				// Z + Y → edge parallel to X
				out.a = center + GBVector3(-e.x, e.y * sy, e.z * GBSign(p.z));
				out.b = center + GBVector3(e.x, e.y * sy, e.z * GBSign(p.z));
			}
		}

		return out;
	}

	bool isPointOnPlane(const GBVector3& point, GBCardinal plane) const
	{
		GBVector3 dpoint = GBAbs(point - center);

		switch (plane)
		{
		case GBCardinal::PosX:
		case GBCardinal::NegX:
			return (dpoint.y <= halfExtents.y && dpoint.z <= halfExtents.z);

		case GBCardinal::PosY:
		case GBCardinal::NegY:
			return (dpoint.x <= halfExtents.x && dpoint.z <= halfExtents.z);

		case GBCardinal::PosZ:
		case GBCardinal::NegZ:
			return (dpoint.x <= halfExtents.x && dpoint.y <= halfExtents.y);
		}

		return false; // safety fallback
	}

	GBVector3 getPointOnPlaneFromCardinal(GBCardinal dir) const
	{
		// Face plane point (center of face)
		GBVector3 planePoint = center;
		switch (dir)
		{
		case GBCardinal::PosX: planePoint.x += halfExtents.x; break;
		case GBCardinal::NegX: planePoint.x -= halfExtents.x; break;
		case GBCardinal::PosY: planePoint.y += halfExtents.y; break;
		case GBCardinal::NegY: planePoint.y -= halfExtents.y; break;
		case GBCardinal::PosZ: planePoint.z += halfExtents.z; break;
		case GBCardinal::NegZ: planePoint.z -= halfExtents.z; break;
		}
		return planePoint;
	}

	float getSizeByCardinal(GBCardinal dir) const
	{

		switch (dir)
		{
		case GBCardinal::PosX: return halfExtents.x; break;
		case GBCardinal::NegX: return halfExtents.x; break;
		case GBCardinal::PosY: return halfExtents.y; break;
		case GBCardinal::NegY: return halfExtents.y; break;
		case GBCardinal::PosZ: return halfExtents.z; break;
		case GBCardinal::NegZ: return halfExtents.z; break;
		}

		return 0.0f;
	}


	GBVector3 forcePointOntoFacePlane(const GBVector3& point, GBCardinal face) const
	{
		GBVector3 forcedPoint = point;
		switch (face)
		{
		case GBCardinal::PosX: 
			forcedPoint.x = center.x + halfExtents.x;
			break;
		case GBCardinal::NegX: 
			forcedPoint.x = center.x - halfExtents.x;
			break;
		case GBCardinal::PosY: 
			forcedPoint.y = center.y + halfExtents.y;
			break;
		case GBCardinal::NegY:
			forcedPoint.y = center.y - halfExtents.y;
			break;
		case GBCardinal::PosZ: 
			forcedPoint.z = center.z + halfExtents.z;
			break;
		case GBCardinal::NegZ:
			forcedPoint.z = center.z - halfExtents.z;
			break;
		}
		return forcedPoint;
	}

	void extractVerts(GBVector3 outVerts[8])
	{
		GBVector3 min = low();
		GBVector3 extents = 2.0f * halfExtents;

		outVerts[0] = min;
		outVerts[1] = min + extents.xComponent();
		outVerts[2] = min + extents.yComponent();
		outVerts[3] = min + extents.zComponent();
		outVerts[4] = min + extents.xyComponent();
		outVerts[5] = min + extents.xzComponent();
		outVerts[6] = min + extents.yzComponent();
		outVerts[7] = min + extents;
	}

	bool isPointContainedWithError(const GBVector3& p, float epsilon = 1e-6f) const
	{
		GBVector3 l = low() - GBVector3(epsilon, epsilon, epsilon);
		GBVector3 h = high() + GBVector3(epsilon, epsilon, epsilon);

		return (p.x >= l.x && p.x <= h.x &&
			p.y >= l.y && p.y <= h.y &&
			p.z >= l.z && p.z <= h.z);
	}
};
