// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PGBWorld.h"
#include "PGBWorld_Demo.generated.h"

enum SceneEnumerated
{
	SCENE_BOX = 0,
	SCENE_MULTIBODY = 1,
	SCENE_FPS,
	SCENE_RAGDOLL
};

/**
 * 
 */
UCLASS()
class GRAVITYBALLUNREAL_API APGBWorld_Demo : public APGBWorld
{
	GENERATED_BODY()
public:
	APGBWorld_Demo();

protected:
	virtual void BeginPlay() override;
	SceneEnumerated sceneEnum;

	bool didRunPostInit = false;

	GBBody* pPlayerBody = nullptr;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;



	void initSceneBoxStack();
	void updateSceneBoxStack();

	void initSceneMultibody();
	void updateSceneMultibody();

	void initSceneFPS();
	void updateSceneFPS(float dt);
	void onBulletEnter(const GBManifold& manifold, GBBody* pOther);

	void initSceneRagdoll();
	void updateSceneRagdoll(float dt);


	void postInit();
};
