#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/LineBatchComponent.h"
#include "GBInclude.h"
#include "AGBGrid.generated.h"

UCLASS()
class GRAVITYBALLUNREAL_API AAGBGrid : public AActor
{
	GENERATED_BODY()

public:
	AAGBGrid();

protected:
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid")
	FVector Anchor = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid")
	int32 CellsX = 10;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid")
	int32 CellsY = 10;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid")
	int32 CellsZ = 10;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid")
	float CellSize = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid")
	FLinearColor LineColor = FLinearColor::Green;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid")
	float LineThickness = 1.5f;

	GBGrid toGBGrid() const;

	void GenerateGridLines();

private:
};
