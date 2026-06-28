// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PGBWorld.h"
#include "PGBWorld_Demo.generated.h"

enum SceneEnumerated
{
	SCENE_BOX = 0,
	SCENE_MULTIBODY = 1,
	SCENE_FPS
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


public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;



	void initSceneBoxStack();
	void initSceneMultibody();
	void initSceneFPS();
	void updateSceneBoxStack();
	void updateSceneMultibody();
	void updateSceneFPS();
};
