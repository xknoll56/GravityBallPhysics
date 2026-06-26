// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GB_Collider.generated.h"


UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class GRAVITYBALLUNREAL_API UGB_Collider : public UActorComponent
{
	GENERATED_BODY()

public:

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool wasDropped = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool isStatic = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool isSleeping = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool isTrigger = false;

	UPROPERTY(EditAnywhere)
	uint32 mask = 0b01;

	UPROPERTY(EditAnywhere)
	uint32 layer = ~0b01;
};
