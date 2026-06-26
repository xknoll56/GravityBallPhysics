#include "AGBGrid.h"
#include "Engine/World.h"
#include "Components/LineBatchComponent.h"
#include "PGBWorld.h"

AAGBGrid::AAGBGrid()
{
	PrimaryActorTick.bCanEverTick = true;
}

void AAGBGrid::BeginPlay()
{
	Super::BeginPlay();
	//GenerateGridLines();
}

void AAGBGrid::GenerateGridLines()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	ULineBatchComponent* LineBatch = World->PersistentLineBatcher;
	if (!LineBatch)
	{
		return;
	}
	FVector Origin = APGBWorld::toFVector(GBVector3(Anchor.X, Anchor.Y, Anchor.Z));
	float adjustedCellSize = CellSize * 100.0f; // Convert to Unreal units

	const float SizeX = CellsX * adjustedCellSize;
	const float SizeY = CellsY * adjustedCellSize;
	const float SizeZ = CellsZ * adjustedCellSize;

	// X-axis lines
	for (int32 Y = 0; Y <= CellsY; ++Y)
	{
		for (int32 Z = 0; Z <= CellsZ; ++Z)
		{
			FVector A = Origin + FVector(0, Y * adjustedCellSize, Z * adjustedCellSize);
			FVector B = A + FVector(SizeX, 0, 0);

			LineBatch->DrawLine(A, B, LineColor, SDPG_World, LineThickness, 0.0f);
		}
	}

	// Y-axis lines
	for (int32 X = 0; X <= CellsX; ++X)
	{
		for (int32 Z = 0; Z <= CellsZ; ++Z)
		{
			FVector A = Origin + FVector(X * adjustedCellSize, 0, Z * adjustedCellSize);
			FVector B = A + FVector(0, SizeY, 0);

			LineBatch->DrawLine(A, B, LineColor, SDPG_World, LineThickness, 0.0f);
		}
	}

	// Z-axis lines
	for (int32 X = 0; X <= CellsX; ++X)
	{
		for (int32 Y = 0; Y <= CellsY; ++Y)
		{
			FVector A = Origin + FVector(X * adjustedCellSize, Y * adjustedCellSize, 0);
			FVector B = A + FVector(0, 0, SizeZ);

			LineBatch->DrawLine(A, B, LineColor, SDPG_World, LineThickness, 0.0f);
		}
	}
}

GBGrid AAGBGrid::toGBGrid() const
{
	return GBGrid(
		GBVector3(Anchor.X, Anchor.Y, Anchor.Z),
		CellSize,
		CellsX,
		CellsY,
		CellsZ,
		0
	);
}

void AAGBGrid::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	//APGBWorld::drawGrid(GetWorld(), toGBGrid(), FColor::Red);
}