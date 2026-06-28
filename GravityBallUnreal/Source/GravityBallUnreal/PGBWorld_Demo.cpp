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

	Super::BeginPlay();

}

void APGBWorld_Demo::Tick(float DeltaTime)
{

	switch (sceneEnum)
	{
	case SCENE_BOX:
	{
		updateSceneBoxStack();
	}
	break;
	case SCENE_FPS:
	{
		updateSceneFPS();
	}
	break;
	}

	Super::Tick(DeltaTime);
}

void APGBWorld_Demo::initSceneMultibody()
{

	// --- G ---
	{
		GBBody* C = simulation.createBody(5.0f);

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
		GBBody* B = simulation.createBody(5.0f);

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
			GBBody* Box = simulation.createBody();
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
}

void APGBWorld_Demo::initSceneFPS()
{
	GBBody* pBody = simulation.createBody();
	GBSphereCollider* pSphere = simulation.attachSphereCollider(pBody, 0.5f);
}
void APGBWorld_Demo::updateSceneBoxStack()
{

}
void APGBWorld_Demo::updateSceneMultibody()
{

}
void APGBWorld_Demo::updateSceneFPS()
{

}