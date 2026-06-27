#pragma once
#include "GBSimulation.h"
#include <string>

struct RenderableCollider
{
	GBVector3 color;
	bool translucent;
	bool drawWireFrame;
	bool doRender;
	bool ignoreSpawn;
	std::string name;
	RenderableCollider(GBVector3 color, bool translucent = true, bool drawWireFrame = false, bool doRender = true, bool ignoreSpawn = false, std::string name = "") :
		color(color), translucent(translucent), drawWireFrame(drawWireFrame), doRender(doRender), ignoreSpawn(ignoreSpawn), name(name)
	{

	}
};

struct BodyTag
{
	std::string tag;

	BodyTag(std::string tag = "") :
		tag(tag)
	{

	}
};

enum SceneName
{
	BOX_STACK = 0
};

struct GBSceneSimulation
{
	// The ownership of the simulation is not held here
	GBSimulation* pSimulation;
	SceneName sceneName;

	GBSceneSimulation(GBSimulation* pSimulation) :
		pSimulation(pSimulation)
	{
		
	}

	virtual void start() = 0;
	virtual void update(float dt) = 0;
};


struct GBSceneBoxStack : GBSceneSimulation
{
	GBSceneBoxStack(GBSimulation* pSimulation) :
		GBSceneSimulation(pSimulation)
	{
		sceneName = BOX_STACK;
	}

	void start() override
	{
		GBBody* pBody;
		GBBoxCollider* pBox;
		
		const static int numBoxes = 10;
		const static float spacing = 1.01f;

		for (int i = 0; i<numBoxes; i++)
		{
			pBody = pSimulation->createBody();
			pBox = pSimulation->attachBoxCollider(pBody, { 0.5f,0.5f,0.5f });
			pBody->transform.position = { 0,0, 0.5f + i * spacing};
		}

		pBody = pSimulation->createBody();
		pBox = pSimulation->attachBoxCollider(pBody, { 20.0f,20.0f,0.1f });
		pBody->transform.position = { 0.0f,0.0f,-0.05f };
		pBody->isStatic = true;
	}

	void update(float dt) override
	{
		// Blank... already updarting physics else where
	}

};