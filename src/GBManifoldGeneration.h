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
			GBVector3 local = rayOrigin - box.center;

			GBVector3 distToFace = box.halfExtents - GBAbs(local);

			// pick smallest distance to surface
			if (distToFace.x < distToFace.y && distToFace.x < distToFace.z)
				normal = GBVector3((local.x > 0) ? 1 : -1, 0, 0);
			else if (distToFace.y < distToFace.z)
				normal = GBVector3(0, (local.y > 0) ? 1 : -1, 0);
			else
				normal = GBVector3(0, 0, (local.z > 0) ? 1 : -1);
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


	// TODO TEST THIS****************************************************************
	static bool GBRaycastCapsule(
		const GBCapsuleCollider& capsule,
		const GBVector3& rayOrigin,
		const GBVector3& rayDir,
		GBContact& outContact,
		float maxDistance = 1e30f)
	{
		// --------------------------------------------------
		// Get capsule segment
		// --------------------------------------------------
		GBVector3 A, B;
		capsule.extractSphereLocations(A, B);

		GBVector3 AB = B - A;
		GBVector3 AO = rayOrigin - A;

		float ab2 = AB.dot(AB);
		float ao_ab = AO.dot(AB);
		float d_ab = rayDir.dot(AB);

		GBVector3 m = AO - AB * (ao_ab / ab2);
		GBVector3 n = rayDir - AB * (d_ab / ab2);

		float a = n.dot(n);
		float b = 2.0f * m.dot(n);
		float c = m.dot(m) - capsule.radius * capsule.radius;

		float bestT = maxDistance;
		bool hit = false;

		// --------------------------------------------------
		// Ray vs infinite cylinder
		// --------------------------------------------------
		float disc = b * b - 4 * a * c;
		if (disc >= 0.0f && fabsf(a) > 1e-6f)
		{
			float sqrtDisc = sqrtf(disc);
			float t = (-b - sqrtDisc) / (2 * a);

			if (t >= 0.0f && t <= bestT)
			{
				float y = ao_ab + t * d_ab;
				if (y >= 0.0f && y <= ab2)
				{
					bestT = t;
					hit = true;
				}
			}
		}

		// --------------------------------------------------
		// Ray vs end spheres
		// --------------------------------------------------
		GBSphereCollider s0, s1;
		s0.radius = capsule.radius;
		s1.radius = capsule.radius;
		s0.transform.position = A;
		s1.transform.position = B;

		GBContact c0, c1;

		if (GBRaycastSphere(s0, rayOrigin, rayDir, c0, bestT))
		{
			bestT = c0.penetrationDepth;
			outContact = c0;
			hit = true;
		}

		if (GBRaycastSphere(s1, rayOrigin, rayDir, c1, bestT))
		{
			bestT = c1.penetrationDepth;
			outContact = c1;
			hit = true;
		}

		// --------------------------------------------------
		// If cylinder was closest hit
		// --------------------------------------------------
		if (hit && bestT < maxDistance)
		{
			outContact.position = rayOrigin + rayDir * bestT;

			GBVector3 proj =
				A + AB * ((outContact.position - A).dot(AB) / ab2);

			outContact.normal =
				(outContact.position - proj).normalized();

			outContact.penetrationDepth = bestT;
			return true;
		}

		return false;
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
			outContact.penetrationDepth = GBAbs(radius - dist);
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

	static bool GBContactCapsulePoint(
		const GBCapsuleCollider& capsule,
		const GBVector3& point,
		GBContact& outContact)
	{
		GBVector3 upperS, lowerS, up;
		capsule.extractSphereLocations(upperS, lowerS, &up);

		GBVector3 dp = point - capsule.transform.position;
		float t = GBDot(up, dp);
		float tr = GBAbs(t) - capsule.height;
		if (GBAbs(t) <= 0.5f*capsule.height)
		{
			GBVector3 forward = capsule.transform.forward();
			GBVector3 right = capsule.transform.right();
			float forAmount = GBDot(forward, dp);
			float rightAmount = GBDot(right, dp);
			if (forAmount * forAmount + rightAmount * rightAmount <= capsule.radius * capsule.radius)
			{
				float dist = sqrt(forAmount * forAmount + rightAmount * rightAmount);
				GBVector3 normal = (forward * forAmount + right * rightAmount) / dist;

				outContact.penetrationDepth = GBAbs(capsule.radius - dist);
				outContact.normal = -normal;
				outContact.pIncident = (GBCollider*)&capsule;
				outContact.position = point;
				return true;
			}
		}
		else if (tr <= capsule.radius)
		{
			if (t >= 0.0f)
			{
				//Check upper sphere
				return GBManifoldGeneration::GBContactSpherePoint(upperS, capsule.radius, point, outContact);
			}
			else
			{
				//Check lower sphere
				return GBManifoldGeneration::GBContactSpherePoint(lowerS, capsule.radius, point, outContact);
			}
		}
		return false;
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

	static bool GBContactCapsuleEdge(
		const GBCapsuleCollider& capsule,
		const GBEdge& edge,
		GBContact& outContact,
		GBVector3* pClosestOnEdge = nullptr)
	{
		GBEdge connection;

		GBVector3 lower, upper, up;
		capsule.extractSphereLocations(upper, lower, &up);

		GBEdge capEdge = GBEdge(lower, upper);

		GBContact c;
		GBEdge outEdge;
		float s, t;
		if (GBEdge::closestEdgeBetween(capEdge, edge, outEdge, false, &s, &t))
		{
			if (pClosestOnEdge)
			{
				edge.isPointOnEdge(outEdge.b, pClosestOnEdge);
			}
			//drawEdge(GetWorld(), outEdge, 5, FColor::Red);
			if (s <= 0.0f)
			{
				// Closest to lower sphere
				if (GBManifoldGeneration::GBContactSphereEdge(lower, capsule.radius, edge, outContact))
				{
					return !capEdge.isPointOnEdge(outContact.position);
				}
			}
			else if (s >= 1.0f)
			{
				//Closest to upper sphere
				if (GBManifoldGeneration::GBContactSphereEdge(upper, capsule.radius, edge, outContact))
				{
					return !capEdge.isPointOnEdge(outContact.position);
				}
			}
			else
			{
				// Closest point is on cylinder portion
				GBVector3 capsulePoint = outEdge.a; // closest point on capsule axis
				GBVector3 edgePoint = outEdge.b; // closest point on edge
				GBVector3 offset = edgePoint - capsulePoint;
				float distSqr = offset.lengthSquared();

				GBVector3 clampedPoint;
				if (!edge.isPointOnEdge(outEdge.b, &clampedPoint))
				{
					return GBContactCapsulePoint(capsule, clampedPoint, outContact);
				}
				else if (distSqr <= capsule.radius * capsule.radius)
				{
					float dist = sqrtf(distSqr);

					//outContact.position = capsulePoint + offset * (capsule.radius / dist);
					outContact.position = outEdge.b;
					outContact.normal = -offset.normalized();  // flip normal so capsule is incident
					outContact.penetrationDepth = GBAbs(capsule.radius - dist);

					return true;
				}
			}
		}

		return false;
	}

	static bool GBManifoldCapsuleCapsule(
		const GBCapsuleCollider& reference,
		const GBCapsuleCollider& incident,
		GBManifold& outManifold)
	{
		outManifold.pReference = reference.pBody;
		outManifold.pIncident = incident.pBody;
		float s, t;
		GBVector3 rp, ip;
		GBVector3 rLower, rHigher, rUp, iLower, iHigher, iUp;
		reference.extractSphereLocations(rHigher, rLower, &rUp);
		incident.extractSphereLocations(iHigher, iLower, &iUp);
		GBEdge rEdge = { rLower, rHigher };
		GBEdge iEdge = { iLower, iHigher };
		GBEdge::closestPointsSegmentSegment(rEdge, iEdge, s, t, rp, ip);

		GBVector3 delta = ip - rp;
		float distSq = delta.lengthSquared();
		float radiusSum = reference.radius + incident.radius;

		if (distSq < radiusSum * radiusSum)
		{
			float dist = sqrt(distSq);
			GBVector3 normal = dist > 1e-6f ? delta / dist : GBVector3(0, 1, 0);
			float penetration = radiusSum - dist;

			outManifold.addContact(
				GBContact(rp + normal * reference.radius, normal, penetration)
			);

			outManifold.useNormal(outManifold.contacts[0]);
			return true;
		}

		if (outManifold.numContacts > 0)
		{
			if (GBDot(incident.transform.position - reference.transform.position, outManifold.normal) < 0)
				outManifold.normal *= -1.0f;
		}
		return outManifold.numContacts > 0;
	}

	static bool GBManifoldCapsulePlane(
		const GBCapsuleCollider& capsule,
		const GBPlane& plane,
		GBManifold& manifold,
		bool lockNormal = false)
	{
		GBVector3 normal1, normal2;
		normal1 = plane.normal;
		normal2 = plane.normal;

		GBVector3 upper, lower, up;
		capsule.extractSphereLocations(upper, lower, &up);
		GBVector3 dp1 = upper - plane.point;
		GBVector3 dp2 = lower - plane.point;

		if (GBDot(dp1, normal1) < 0.0f)
			normal1 = -normal1;
		if (GBDot(dp2, normal2) < 0.0f)
			normal2 = -normal2;

		// Adjust to where the actual hits will be
		upper -= normal1 * capsule.radius;
		lower -= normal2 * capsule.radius;
		float d_upper, d_lower;
		d_upper = GBDot(normal1, upper - plane.point);
		d_lower = GBDot(normal2, lower - plane.point);

		if (d_upper <= 0.0f)
		{
			manifold.addContact(GBContact(upper - normal1 * d_upper , normal1, GBAbs(d_upper)));
		}

		if (d_lower <= 0.0f)
		{
			manifold.addContact(GBContact(lower - normal2 * d_lower, normal2, GBAbs(d_lower)));
		}

		return manifold.numContacts > 0;
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
					outContact.pIncident = (GBCollider*)&sphere;
					return true;
				}
			}
			GBEdge closestEdge = triangle.closestEdgeToPoint(sphere.transform.position);
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

	static bool GBManifoldCapsuleTriangle(
		const GBCapsuleCollider& capsule,
		const GBTriangle& triangle,
		GBManifold& outManifold)
	{
		GBPlane plane = triangle.toPlane();
		GBVector3 normal = GBFixNormal(plane.normal, plane.point, capsule.transform.position);
		GBManifold planeTestMan, planeMan, edgeMan;
		if (GBManifoldCapsulePlane(capsule, plane, planeTestMan))
		{
			for (int i = 0; i < planeTestMan.numContacts; i++)
			{
				if (triangle.PointInTriangle(planeTestMan.contacts[i].position))
				{
					planeMan.addContact(planeTestMan.contacts[i]);
				}
			}
		}
		GBEdge edges[3];
		triangle.extractEdges(edges);
		GBVector3 center = triangle.center();
		bool useEdge = false;
		for (int i = 0; i < 3; i++)
		{
			GBContact c;
			if (GBContactCapsuleEdge(capsule, edges[i], c))
			{
				//Need to prune contacts that point inward...
				GBVector3 out = GBCross(edges[i].getAOutDirection(), normal);
				if (GBDot(out, center - edges[i].a) > 0)
					out = -out;

				if (GBDot(c.normal, out) < 0.0f)
					c.normal = -c.normal;

				GBVector3 dp = capsule.transform.position - c.position;
				if (GBDot(dp, c.normal) < 0)
					c.normal = -c.normal;


				edgeMan.addContact(c);

				if (planeMan.numContacts > 0)
				{
					if (GBAbs(planeMan.contacts[0].penetrationDepth) < GBAbs(c.penetrationDepth))
						useEdge = true;
				}
			}
		}

		if (planeMan.numContacts > 0)
		{
			if (useEdge)
			{
				outManifold.useNormal(edgeMan.contacts[0]);
				outManifold.combine(edgeMan);
			}
			else
			{
				outManifold.useNormal(planeMan.contacts[0]);
				outManifold.combine(planeMan);
			}
		}
		else if(edgeMan.numContacts > 0)
		{
			outManifold.useNormal(edgeMan.contacts[0]);
			outManifold.combine(edgeMan);
		}


		outManifold.pIncident = capsule.pBody;

		outManifold.pruneWithNormal(1e-4);
		outManifold.correctPenetrations();


		if (outManifold.numContacts > 0)
			outManifold.separation = GBAbs(outManifold.contacts[0].penetrationDepth);

		return outManifold.numContacts > 0;
	}

	static bool GBManifoldCapsuleQuad(
		const GBCapsuleCollider& capsule,
		const GBQuad& quad,
		GBManifold& outManifold, 
		bool lockNormal = false)
	{
		GBPlane plane = quad.toPlane();
		GBVector3 normal = GBFixNormal(plane.normal, plane.point, capsule.transform.position);
		GBManifold planeTestMan, planeMan, edgeMan;
		if (GBManifoldCapsulePlane(capsule, plane, planeTestMan, lockNormal))
		{
			for (int i = 0; i < planeTestMan.numContacts; i++)
			{
				if (quad.isPointContained(planeTestMan.contacts[i].position))
				{
					
					planeMan.addContact(planeTestMan.contacts[i]);
				}
			}
		}
		GBEdge edges[4];
		quad.extractEdges(edges);
		GBVector3 center = quad.position;
		bool useEdge = false;
		for (int i = 0; i < 4; i++)
		{
			GBContact c;
			if (GBContactCapsuleEdge(capsule, edges[i], c))
			{
				//Need to prune contacts that point inward...
				GBVector3 out = GBCross(edges[i].getAOutDirection(), normal);
				if (GBDot(out, center - edges[i].a) > 0)
					out = -out;

				if (GBDot(c.normal, out) < 0.0f)
					c.normal = -c.normal;

				GBVector3 dp = capsule.transform.position - c.position;
				if (GBDot(dp, c.normal) < 0)
					c.normal = -c.normal;

				edgeMan.addContact(c);


				if (planeMan.numContacts > 0)
				{
					if (GBAbs(planeMan.contacts[0].penetrationDepth) < GBAbs(c.penetrationDepth))
						useEdge = true;
				}
			}
		}

		if (planeMan.numContacts > 0)
		{
			if (useEdge)
			{
				outManifold.useNormal(edgeMan.contacts[0]);
				outManifold.combine(edgeMan);
			}
			else
			{
				outManifold.useNormal(planeMan.contacts[0]);
				outManifold.combine(planeMan);
			}
		}
		else if (edgeMan.numContacts > 0)
		{
			outManifold.useNormal(edgeMan.contacts[0]);
			outManifold.combine(edgeMan);
		}

		outManifold.pruneWithNormal(1e-4);
		outManifold.correctPenetrations();

		if (outManifold.numContacts > 0)
			outManifold.separation = GBAbs(outManifold.contacts[0].penetrationDepth);
		outManifold.pIncident = capsule.pBody;

		return outManifold.numContacts > 0;
	}

	static bool GBManifoldCapsuleBox(
		const GBCapsuleCollider& capsule,
		 GBBoxCollider& box,
		GBManifold& outManifold)
	{
		GBBoxCollider testBox(box.halfExtents + GBVector3{capsule.radius, capsule.radius, capsule.radius});
		testBox.setTransform(box.transform);
		testBox.setVerts();
		GBContact hit;
		GBVector3 upper, lower;
		capsule.extractSphereLocations(upper, lower);
		if (GBRaycastOBB(testBox, lower, (upper - lower).normalized(), hit))
		{
			GBQuad face = GBBoxDirectionToQuad(box, hit.normal);
			if (GBManifoldCapsuleQuad(capsule, face, outManifold, true))
			{
				outManifold.pIncident = capsule.pBody;
				outManifold.pReference = box.pBody;
				return true;
			}
		}

		return false;
	}

	static bool GBManifoldCapsuleSphere(
		const GBCapsuleCollider& capsule,
		const GBSphereCollider& sphere,
		GBManifold& outManifold)
	{
		GBVector3 closestPoint;
		GBVector3 upper, lower, up;
		capsule.extractSphereLocations(upper, lower, &up);
		GBEdge capsuleEdge = { lower, upper };
		GBVector3 pointOnEdge = capsuleEdge.closestPointOnLine(sphere.transform.position);

		float s = GBDot(pointOnEdge - lower, up);
		GBContact c;
		GBSphereCollider lowerS(capsule.radius);
		lowerS.transform.position = lower;
		GBSphereCollider upperS(capsule.radius);
		upperS.transform.position = upper;
		bool retValue = false;
		if (s < 0.0f)
		{
			if (GBManifoldGeneration::GBContactSphereSphere(lowerS, sphere, c))
			{
				c.position = sphere.transform.position - c.normal * sphere.radius;
				outManifold.addContact(c);
				retValue = true;
			}
		}
		else if (s > capsule.height)
		{
			if (GBManifoldGeneration::GBContactSphereSphere(upperS, sphere, c))
			{
				c.position = sphere.transform.position - c.normal * sphere.radius;
				outManifold.addContact(c);
				retValue = true;
			}
		}
		else
		{
			GBVector3 diff = sphere.transform.position - pointOnEdge;
			float dist = diff.length();

			float r = capsule.radius + sphere.radius;

			if (dist < r)
			{
				c.normal = (dist > 1e-6f) ? diff / dist : GBVector3::up();
				c.position = sphere.transform.position - c.normal * sphere.radius;
				c.penetrationDepth = r - dist;

				outManifold.addContact(c);
				retValue = true;
			}
		}

		if (retValue)
		{
			if (outManifold.numContacts > 0)
			{
				if (GBDot(capsule.transform.position - outManifold.contacts[0].position, outManifold.contacts[0].normal) < 0)
					outManifold.contacts[0].normal *= -1.0f;
				outManifold.useNormal(outManifold.contacts[0]);
				outManifold.separation = GBAbs(outManifold.contacts[0].penetrationDepth);
			}
			if (capsule.pBody)
				outManifold.pIncident = capsule.pBody;
			if (sphere.pBody)
				outManifold.pReference = sphere.pBody;
			return true;
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
			outManifold.pReference = a.pBody;
			outManifold.pIncident = b.pBody;
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
			outManifold.pReference = box.pBody;
			outManifold.pIncident = sphere.pBody;
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
			outManifold.pReference = boxA.pBody;
			outManifold.pIncident = boxB.pBody;
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

		GBManifold m;
		if (GBBuildQuadAABBManifold(incidentFace, referenceAABB, colData.bestCardinal, m))
		{
			GBPlane plane = incidentFace.toPlane();
			m.pruneBehindPlane(plane);
			m.applyTransformation(pReference->transform);


			outManifold.combine(m);
			outManifold.pReference = pReference->pBody;
			outManifold.pIncident = pIncident->pBody;
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

			m.reset();
			if (GBBuildQuadAABBManifold(referenceFace, incidentAABB, GBCardinalGetReverse(colData.bestCardinal), m, false))
			{
				
				GBPlane plane = referenceFace.toPlane();
				m.pruneBehindPlane(plane);
				m.applyTransformation(pIncident->transform);
				for (int i = 0; i < m.numContacts; i++)
				{
					m.contacts[i].normal *= -1.0f;
				}
			}
			outManifold.combine(m);
		}

		outManifold.pIncident = pIncident->pBody;
		outManifold.pReference = pReference->pBody;

		if (outManifold.pIncident && outManifold.pReference)
		{
			if (outManifold.pIncident->isStatic)
			{
				GBBody* temp = outManifold.pIncident;
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
			outManifold.pIncident = box.pBody;
			normal = outManifold.normal;
		}
		//else

		GBManifold edgeManifold;
		{
			GBQuad qa = GBManifoldGeneration::GBBoxDirectionToQuad(box, -colData.bestAxis);
			edgeManifold.pIncident = box.pBody;
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
		//if (GBDot(outManifold.normal, colData.bestAxis) < 0.0f)
		//	outManifold.normal *= -1.0f;
		if (GBDot(outManifold.normal, box.transform.position - triangle.center()) < 0.0f)
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