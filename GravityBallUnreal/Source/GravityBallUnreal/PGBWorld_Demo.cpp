// Fill out your copyright notice in the Description page of Project Settings.


#include "PGBWorld_Demo.h"

APGBWorld_Demo::APGBWorld_Demo()
{

}

void APGBWorld_Demo::BeginPlay()
{
	FString levelName = GetWorld()->GetMapName();
	levelName.RemoveFromStart(GetWorld()->StreamingLevelsPrefix);

	if (levelName == "BoxStack")
	{
		initSceneBoxStack();
	}
	else if (levelName == "Multibody")
	{
		initSceneMultibody();
	}
	else if (levelName == "FPS")
	{
		initSceneFPS();
	}
	else if (levelName == "Ragdoll")
	{
		initSceneRagdoll();
	}

	Super::BeginPlay();
	doDrawAllPhysicsColliders = false;

}

void APGBWorld_Demo::Tick(float DeltaTime)
{
	if (!didRunPostInit)
	{
		postInit();
		didRunPostInit = true;
	}
	switch (sceneEnum)
	{
	case SCENE_BOX:
	{
		updateSceneBoxStack();
	}
	break;
	case SCENE_FPS:
	{
		updateSceneFPS(DeltaTime);
	}
	break;
	}

	Super::Tick(DeltaTime);
}

void APGBWorld_Demo::initSceneMultibody()
{

	// --- G ---
	{
		GBBody* C = simulation.createBody(2.0f);

		float thickness = 0.25f;
		float radius = 1.0f;
		GBVector3 color = { 0,0.6,1 };
		for (int i = 0; i < 5; i++)
		{
			float portion = i / 5.0f;
			float nextPotion = ((i + 1) % 5) / 5.0f;
			const float heightModifier = 1.5f;

			if (i < 4)
			{
				GBVector3 p1 = GBVector3(0, radius * cos(portion * 2 * GB_PI), radius * heightModifier * sin(portion * 2 * GB_PI));
				GBVector3 p2 = GBVector3(0, radius * cos(nextPotion * 2 * GB_PI), radius * heightModifier * sin(nextPotion * 2 * GB_PI));
				GBCapsuleCollider testCap = GBCapsuleCollider::capsuleFromEdge({ p1, p2 });
				GBCapsuleCollider* pCap = simulation.attachCapsuleCollider(C, thickness, testCap.height,
					GBTransform(testCap.transform.position - C->transform.position, testCap.transform.rotation));
				setRenderableData(pCap, color);
			}
			else
			{
				GBVector3 p1 = GBVector3(0, radius * cos(portion * 2 * GB_PI), radius * heightModifier * sin(portion * 2 * GB_PI));
				GBVector3 p2 = p1 + GBVector3(0, 0, 1.0f);
				GBCapsuleCollider testCap = GBCapsuleCollider::capsuleFromEdge({ p1, p2 });
				GBCapsuleCollider* pCap = simulation.attachCapsuleCollider(C, thickness, testCap.height,
					GBTransform(testCap.transform.position - C->transform.position, testCap.transform.rotation));
				setRenderableData(pCap, color);
				p1 = p2;
				p2 = p1 + GBVector3(0, -0.5, 0.0f);

				testCap = GBCapsuleCollider::capsuleFromEdge({ p1, p2 });
				pCap = simulation.attachCapsuleCollider(C, thickness, testCap.height,
					GBTransform(testCap.transform.position - C->transform.position, testCap.transform.rotation));
				setRenderableData(pCap, color);
			}

		}

		GBVector3 min = C->aabb.low();
		setRenderableData(simulation.attachBoxCollider(C, { 0.25f, C->aabb.halfExtents.y,0.1f }, GBTransform(-C->transform.position
			+ min.zComponent() * (1.05f) + GBVector3::right() * 0.25f, GBQuaternion::fromAxisAngle({ 0,0,1 }, GB_PI * 0.25f))), color);
		setRenderableData(simulation.attachBoxCollider(C, { C->aabb.halfExtents.y, 0.25f,0.1f }, GBTransform(-C->transform.position
			+ min.zComponent() * (1.05f) + GBVector3::right() * 0.25f, GBQuaternion::fromAxisAngle({ 0,0,1 }, GB_PI * 0.25f))), color);

		C->transform.position = { 0, -2, 2 };
	}

	// --- B ---
	{
		GBBody* B = simulation.createBody(2.0f);

		float thickness = 0.25f;
		float radius = 0.80f;

		GBVector3 firstPoint, lastPoint;
		GBVector3 color = { 0.0f, 1.0f, 0.25f };
		for (int i = 0; i < 5; i++)
		{
			float portion = i / 6.0f;
			float nextPotion = ((i + 1) % 6) / 6.0f;
			const float widthModifier = 1.5f;
			GBVector3 p1 = GBVector3(0, radius * widthModifier * sin(portion * GB_PI * 1.25), radius * -cos(portion * GB_PI * 1.25));
			GBVector3 p2 = GBVector3(0, radius * widthModifier * sin(nextPotion * GB_PI * 1.25), radius * -cos(nextPotion * GB_PI * 1.25));
			GBCapsuleCollider testCap = GBCapsuleCollider::capsuleFromEdge({ p1, p2 });
			GBCapsuleCollider* pCap = simulation.attachCapsuleCollider(B, thickness, testCap.height,
				GBTransform(testCap.transform.position - B->transform.position, testCap.transform.rotation));
			setRenderableData(pCap, color);
			if (i == 4)
				lastPoint = p2;

		}
		for (int i = 0; i < 5; i++)
		{
			float portion = i / 6.0f;
			float nextPotion = ((i + 1) % 6) / 6.0f;
			const float widthModifier = 1.5f;
			GBVector3 offset = { 0,0,-2.0f * radius };
			GBVector3 p1 = offset + GBVector3(0, radius * widthModifier * sin(portion * GB_PI * 1.25), radius * -cos(portion * GB_PI * 1.25));
			GBVector3 p2 = offset + GBVector3(0, radius * widthModifier * sin(nextPotion * GB_PI * 1.25), radius * -cos(nextPotion * GB_PI * 1.25));
			GBCapsuleCollider testCap = GBCapsuleCollider::capsuleFromEdge({ p1, p2 });
			GBCapsuleCollider* pCap = simulation.attachCapsuleCollider(B, thickness, testCap.height,
				GBTransform(testCap.transform.position - B->transform.position, testCap.transform.rotation));
			setRenderableData(pCap, color);
			if (i == 0)
				firstPoint = p1;
			else if (i == 4)
			{
				p1 = firstPoint;
				p2 = lastPoint;
				testCap = GBCapsuleCollider::capsuleFromEdge({ p1, p2 });
				pCap = simulation.attachCapsuleCollider(B, thickness, testCap.height,
					GBTransform(testCap.transform.position - B->transform.position, testCap.transform.rotation));
				setRenderableData(pCap, color);
			}
		}

		GBVector3 min = B->aabb.low();
		setRenderableData(simulation.attachBoxCollider(B, { 0.25f, B->aabb.halfExtents.y,0.1f }, GBTransform(-B->transform.position
			+ min.zComponent() + GBVector3::right() * 0.25f, GBQuaternion::fromAxisAngle({ 0,0,1 }, GB_PI * 0.25f))), color);
		setRenderableData(simulation.attachBoxCollider(B, { B->aabb.halfExtents.y, 0.25f,0.1f }, GBTransform(-B->transform.position
			+ min.zComponent() + GBVector3::right() * 0.25f, GBQuaternion::fromAxisAngle({ 0,0,1 }, GB_PI * 0.25f))), color);

		B->transform.position = { 0, 2, 2 };
	}

	// Box
	{
		for (int start = 0; start < 4; start++)
		{
			GBVector3 startPos;
			GBVector3 color;
			switch (start)
			{
			case 0:
				startPos = { 5, 5, 5 };
				color = { 1,0,0 };
				break;
			case 1:
				startPos = { -5, 5, 5 };
				color = { 0,1,0 };
				break;
			case 2:
				startPos = { -5, -5, 5 };
				color = { 0,0,1 };
				break;
			case 3:
				startPos = { 5, -5, 5 };
				color = { 0.74,0.83, 0.94 };
				break;
			}
			GBBody* Box = simulation.createBody(5.0F);
			GBEdge edges[4];
			GBBoxCollider bc({ 1.0,1.0,1.0 });
			for (int i = 0; i < 6; i++)
			{
				GBCardinal faceDir = (GBCardinal)i;
				GBQuad face = GBManifoldGeneration::GBBoxCardinalToQuad(bc, faceDir);
				face.extractEdges(edges);	
				for (int j = 0; j < 4; j++)
				{
					if ((i == 4 || i == 5) || GBAbs(edges[j].a.z - edges[j].b.z) > 0.0f)
					{
						GBCapsuleCollider testCap = GBCapsuleCollider::capsuleFromEdge({ edges[j].a, edges[j].b });
						GBCapsuleCollider* pCap = simulation.attachCapsuleCollider(Box, 0.2f, testCap.height,
							GBTransform(testCap.transform.position - Box->transform.position, testCap.transform.rotation));
						setRenderableData(pCap, color);
					}
				}
			}
			simulation.recenterMass(Box);
			Box->transform.position = startPos;
		}
	}


	for (int i = 0; i < 10; i++)
	{
		shootableBody = simulation.createBody();
		shootableBody->transform.position = { -25.0f,-10.0f + 2.0f * i,0.5f };
		GBSphereCollider* pSphere = simulation.attachSphereCollider(shootableBody, 0.5f + (i * 0.1f) * 0.75);
		pSphere->pData = new RenderableCollider({ 0.1f * (i % 2),0.76f * (i + 1 % 2), 0.92f });
		shootableBody->setMass(5.0f * pSphere->volume());
		shootStack[i] = shootableBody;
	}

	shootableBody = shootStack[0];
	shootCount = 10;

	sceneEnum = SCENE_MULTIBODY;
}

void APGBWorld_Demo::initSceneBoxStack()
{
	GBBody* pBody;
	GBBoxCollider* pBox;

	const static int numBoxes = 10;
	const static float spacing = 1.25f;

	for (int i = 0; i < numBoxes; i++)
	{
		for (int j = 0; j < numBoxes; j++)
		{
			pBody = simulation.createBody();
			pBox = simulation.attachBoxCollider(pBody, { 0.5f,0.5f,0.5f });
			pBody->transform.position = { 0,j * 1.01f, 0.5f + i * spacing };
			pBox->pData = new RenderableCollider({ (float)((i + 1) % 2),
				(float)(j % 2), (float)((i + j) % 2) });
		}

	}


	for (int i = 0; i < 10; i++)
	{
		shootableBody = simulation.createBody();
		shootableBody->transform.position = { -25.0f,-10.0f + 2.0f * i,0.5f };
		GBSphereCollider* pSphere = simulation.attachSphereCollider(shootableBody, 0.5f + (i * 0.1f) * 0.75);
		pSphere->pData = new RenderableCollider({ 0.1f * (i % 2),0.76f * (i + 1 % 2), 0.92f });
		shootableBody->setMass(5.0f * pSphere->volume());
		shootStack[i] = shootableBody;
	}

	shootableBody = shootStack[0];
	shootCount = 10;

	sceneEnum = SCENE_BOX;
	doUpdateCamera = true;
}

void APGBWorld_Demo::onBulletEnter(const GBManifold& manifold, GBBody* pOther)
{
	manifold.getOtherBody(pOther)->transform.position = { 2000, 2000, 2000};
	manifold.getOtherBody(pOther)->updateColliders();
}

void APGBWorld_Demo::onTriggerEnter(const GBManifold& manifold, GBBody* pOther)
{
	if (pOther == shootableBody)
	{
		for (auto& door : doors)
		{
			if(!door.second.isActivated)
				door.second.isActivated = true;
			else
			{
				door.first->transform.position = door.second.initPos;
				door.second.isActivated = false;
			}
		}
	}
}

void APGBWorld_Demo::updateDoors(float dt)
{
	for (auto& door : doors)
	{
		if (door.second.isActivated)
		{
			const static float activationDistance = 10.0f;
			if (door.first->transform.position.z - door.second.initPos.z < activationDistance)
				door.first->transform.position += {0, 0, dt};
			else
				door.second.isActivated = false;
		}
	}
}

void APGBWorld_Demo::initSceneFPS()
{
	pPlayerBody = simulation.createBody();
	GBCapsuleCollider* pPlayerCap = simulation.attachCapsuleCollider(pPlayerBody, 0.5f, 1.0f);
	setRenderableData(pPlayerCap, { 1,1,1 }, true, false);
	pPlayerBody->isKinematic = true;
	pPlayerBody->transform.position = toGBVector(GetActorLocation());
	pPlayerBody->transform.rotation = toGBQuat(GetActorQuat());
	pPlayerBody->layer = 1 << 1;
	pPlayerBody->mask = 1 | (1 << 1);

	GBBody* pBody = simulation.createBody();
	GBSphereCollider* pSphere = simulation.attachSphereCollider(pBody, 0.1f);
	shootableBody = pBody;
	shootStack[0] = pBody;
	pBody->useGravity = false;
	shootCount = 1;
	pBody->layer = 1;
	pBody->mask = 1;
	shootSpeed = 5000;
	std::function<void(const GBManifold&, GBBody*)> fnEnter =
		[this](const GBManifold& m, GBBody* b)
		{
			onBulletEnter(m, b);
		};
	simulation.addEnterListener(pBody, fnEnter);

	sceneEnum = SCENE_FPS;
	doUpdateCamera = false;
	wasdMoveBindings = true;
	doDrawAllPhysicsColliders = true;
}
void APGBWorld_Demo::updateSceneBoxStack()
{

}
void APGBWorld_Demo::updateSceneMultibody()
{

}
void APGBWorld_Demo::updateSceneFPS(float dt)
{
	updateDoors(dt);

	moveCamera(dt, false);
	movePosition(dt, pPlayerBody->transform.position, 10.0f);


	// Get the player controller
	if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{

		GBVector3 cameraPos = pPlayerBody->transform.position + GBVector3(0, 0, 1);
		setCameraPosition(cameraPos);
		GBVector3 forward, right, up;
		extractCameraForwardAndRight(right, forward, up);
		GBQuaternion rot = GBQuaternion::fromAxes(forward, right, up);
		GBRay ray;
		GBBoxCollider gun = GBBoxCollider({ 0.5f, 0.2f,0.2f });
		GBBoxCollider gunHandle = GBBoxCollider({ 0.02f, 0.2f,0.45f });
		gun.transform.rotation = rot;
		gunHandle.transform.rotation = rot;
		gun.transform.position = cameraPos + right + forward * 2.0f;
		gunHandle.transform.position = cameraPos + right + forward * 1.5f + up * -0.25f;
		drawBox(GetWorld(), gun, FColor::Cyan);
		drawBox(GetWorld(), gunHandle, FColor::Emerald);
		if (simulation.raycast(cameraPos, forward, ray, 100.0f, 1))
		{
			drawLine(GetWorld(), cameraPos + right + forward * 2.0f, ray.position, 0.1f);
			drawPoint(GetWorld(), ray.position, 6.0f, FColor::Emerald);
			if (ray.pIncident && ray.distance < 5.0f)
			{
				RenderableCollider* prc = (RenderableCollider*)ray.pIncident->pData;
				if (prc->canGrab)
				{
					GBBody* pBody = ray.pIncident->pBody;
					grabbedBody = pBody;
				}
			}
		}
	}
}

void APGBWorld_Demo::postInit()
{

	switch (sceneEnum)
	{
	case SCENE_FPS:
	{
		std::vector<GBBody*> bs = getBodiesByTag(GetWorld(), "canGrab");
		for (GBBody* c : bs)
		{
			for (GBCollider* pCol : c->colliders)
			{
				RenderableCollider* prc = (RenderableCollider*)pCol->pData;
				if (prc)
					prc->canGrab = true;
			}
		}

		bs.clear();
		bs = getBodiesByTag(GetWorld(), "isTrigger");
		for (GBBody* c : bs)
		{
			std::function<void(const GBManifold&, GBBody*)> fnEnter =
				[this](const GBManifold& m, GBBody* b)
				{
					onTriggerEnter(m, b);
				};
			simulation.addEnterListener(c, fnEnter);
		}

		bs.clear();
		bs = getBodiesByTag(GetWorld(), "isDoor");

		for (GBBody* db : bs)
		{
			std::pair<GBBody*, Door> pair;
			pair.first = db;
			pair.second.isActivated = false;
			pair.second.initPos = db->transform.position;
			doors.insert(pair);
		}
	}
		break;
	}

}

void APGBWorld_Demo::initSceneRagdoll()
{
	GBBody* body = simulation.createBody();
	body->transform.position = { 0,0,6 };
	body->angularVelocity = { 100,10,10 };
	body->velocity = { 10,10,10 };
	GBCapsuleCollider* cc = simulation.attachCapsuleCollider(body, 0.25f, 0.8f);
	setRenderableData(cc, { 0,1,1 }, false, true);


	GBBody* arm1 = simulation.createBody();
	GBCapsuleCollider* cc1 = simulation.attachCapsuleCollider(arm1, 0.1f, 0.45f);
	arm1->transform.position = body->transform.position +
		GBVector3::right() * (cc->radius + cc1->radius + cc1->height * 0.5f)
		+ GBVector3::up() * 0.25f;
	arm1->transform.rotate(GBQuaternion::fromAxisAngle({ 1,0,0 }, GB_PI * 0.5f));
	arm1->updateColliders();
	setRenderableData(cc1, { 1, 0, 0 }, false, true);


	GBBody* arm11 = simulation.createBody();
	GBCapsuleCollider* cc11 = simulation.attachCapsuleCollider(arm11, 0.1f, 0.45f);
	arm11->transform.position = arm1->transform.position + GBVector3::right() * (cc1->height * 0.5f + cc1->radius + cc11->height * 0.5f + cc11->radius);
	arm11->transform.rotate(GBQuaternion::fromAxisAngle({ 1,0,0 }, GB_PI * 0.5f));
	arm11->updateColliders();
	setRenderableData(cc11, { 1, 1, 0 }, false, true);


	GBBody* arm2 = simulation.createBody();
	GBCapsuleCollider* cc2 = simulation.attachCapsuleCollider(arm2, 0.1f, 0.45f);
	arm2->transform.position = body->transform.position -
		GBVector3::right() * (cc->radius + cc2->radius + cc2->height * 0.5f)
		+ GBVector3::up() * 0.25f;
	arm2->transform.rotate(GBQuaternion::fromAxisAngle({ 1,0,0 }, GB_PI * 0.5f));
	arm2->updateColliders();
	setRenderableData(cc2, { 1, 0, 0 }, false, true);

	GBBody* arm22 = simulation.createBody();
	GBCapsuleCollider* cc22 = simulation.attachCapsuleCollider(arm22, 0.1f, 0.45f);
	arm22->transform.position = arm2->transform.position - GBVector3::right() * (cc2->height * 0.5f + cc2->radius + cc22->height * 0.5f + cc22->radius);
	arm22->transform.rotate(GBQuaternion::fromAxisAngle({ 1,0,0 }, GB_PI * 0.5f));
	arm22->updateColliders();
	setRenderableData(cc22, { 1, 1, 0 }, false, true);


	GBBody* leg = simulation.createBody();
	GBCapsuleCollider* cc3 = simulation.attachCapsuleCollider(leg, 0.1f, 0.5f);
	leg->transform.position = body->transform.position +
		GBVector3::up() * (-cc->height * 0.5f - cc->radius - cc3->height * 0.5f - cc3->radius) +
		GBVector3::right() * (cc3->radius);
	leg->updateColliders();
	setRenderableData(cc3, { 1, 0, 0 }, false, true);

	GBBody* foot = simulation.createBody();
	GBCapsuleCollider* cc33 = simulation.attachCapsuleCollider(foot, 0.1f, 0.5);
	foot->transform.position = leg->transform.position +
		GBVector3::up() * (-cc3->height * 0.5f - cc3->radius - cc33->height * 0.5f - cc33->radius);
	foot->updateColliders();
	setRenderableData(cc33, { 1, 0, 1 }, false, true);

	GBBody* leg1 = simulation.createBody();
	GBCapsuleCollider* cc4 = simulation.attachCapsuleCollider(leg1, 0.1f, 0.5);
	leg1->transform.position = body->transform.position +
		GBVector3::up() * (-cc->height * 0.5f - cc->radius - cc3->height * 0.5f - cc3->radius) +
		GBVector3::left() * (cc3->radius);
	leg1->updateColliders();
	setRenderableData(cc4, { 1, 0, 0 }, false, true);

	GBBody* foot1 = simulation.createBody();
	GBCapsuleCollider* cc44 = simulation.attachCapsuleCollider(foot1, 0.1f, 0.5);
	foot1->transform.position = leg1->transform.position +
		GBVector3::up() * (-cc4->height * 0.5f - cc4->radius - cc44->height * 0.5f - cc44->radius);
	foot1->updateColliders();
	setRenderableData(cc44, { 1, 0, 1 }, false, true);

	GBBody* head = simulation.createBody();
	GBCapsuleCollider* cc5 = simulation.attachCapsuleCollider(head, 0.35f, 0.0f);
	head->transform.position = body->transform.position +
		GBVector3::up() * (cc->height * 0.5f + cc->radius + cc5->radius);
	head->updateColliders();
	setRenderableData(cc5, { 0, 0, 1 }, false, true);


	GBBallJoint* s = simulation.attachCapsuleBallJoint(body, cc1);
	s = simulation.attachCapsuleBallJoint(arm1, cc11);
	s = simulation.attachCapsuleBallJoint(body, cc2);
	s = simulation.attachCapsuleBallJoint(arm2, cc22);
	s = simulation.attachCapsuleBallJoint(body, cc4);
	s = simulation.attachCapsuleBallJoint(leg1, cc44);
	s = simulation.attachCapsuleBallJoint(body, cc3);
	s = simulation.attachCapsuleBallJoint(leg, cc33);
	s = simulation.attachCapsuleBallJoint(body, cc5);

	sceneEnum = SCENE_RAGDOLL;
}

void APGBWorld_Demo::updateSceneRagdoll(float dt)
{

}