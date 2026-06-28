#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Camera/CameraComponent.h"
#include "ProceduralMeshComponent.h" // Needed for procedural meshes
#include "GBSimulation.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include <string>


#include "PGBWorld.generated.h"

struct RenderableCollider
{
	GBVector3 color;
	bool translucent;
	bool drawWireFrame;
	bool doRender;
	bool ignoreSpawn;
	std::string name;
	RenderableCollider(GBVector3 color, bool translucent = false, bool drawWireFrame = false, bool doRender = true, bool ignoreSpawn = false, std::string name = "") :
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


UCLASS()
class GRAVITYBALLUNREAL_API APGBWorld : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	APGBWorld();
	~APGBWorld();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	void moveCamera(float deltaTime);
	void movePosition(float deltaTime, GBVector3& outPosition, float speed = 1.0f);

	static void drawAABB(UWorld* World, GBAABB aabb, FColor color = FColor::Green, float thickness = 2.0f);
	static void drawBox(UWorld* World, const GBBoxCollider& box, FColor color = FColor::Green, float thickness = 2.0f);
	static void drawGrid(UWorld* World, const GBGrid& grid, FColor color = FColor::Red);
	static void drawTriangle(UWorld* World, const GBVector3& v0, const GBVector3& v1, const GBVector3& v2,
		FColor Color = FColor::White, float Thickness = 2.0f, int Steps = 8, FColor fillColor = FColor::White);
	static void drawTriangle(UWorld* World, const GBTriangle& triangle,
		FColor Color = FColor::White, float Thickness = 2.0f, int Steps = 8, FColor fillColor = FColor::White);
	static void drawPoint(UWorld* World, const GBVector3& point, float Radius = 5.0f, FColor Color = FColor::Red);
	static void drawEdge(UWorld* World, GBEdge edge, float Radius = 5.0f, FColor Color = FColor::White);
	static void drawLine(UWorld* World, GBVector3 a, GBVector3 b, float Radius = 5.0f, FColor Color = FColor::Red);
	static void drawRay(UWorld* World, GBVector3 start, GBContact contact,
		float Radius = 5.0f, FColor Color = FColor::Red, bool doDrawContact = false, FColor contactColor = FColor::Emerald);
	static void drawSphere(UWorld* world, const GBVector3& center, float radius, FColor color = FColor::Blue, GBQuaternion rotation = GBQuaternion());
	static void drawSphereCollider(UWorld* world, const GBSphereCollider& sphere, FColor color = FColor::Blue);
	static void drawContact(UWorld* world, const GBContact& contact, float size = 1.0f, FColor color = FColor::Red);
	static void drawManifold(UWorld* world, const GBManifold& manifold, float size = 1.0f, FColor color = FColor::Red);
	static void drawQuad(UWorld* world, const GBQuad& quad, float size, FColor color, bool fill = false, float fillWidth = 1.0f);
	static void drawPlane(UWorld* world, GBPlane plane, FColor color = FColor::Yellow);
	static void drawFrame(UWorld* world, const GBFrame& frame, const GBVector3 position, float size = 1.0f);
	static void drawCapsule(UWorld* world, const GBCapsuleCollider& capsule, FColor color = FColor::Purple, float skin = 0.0f);
	static RenderableCollider* getRenderable(GBCollider* collider);

	void printVector(const GBVector3& vector, FColor color = FColor::White, float time = 5.0f);
	void printString(const std::string& string, FColor color = FColor::White, float time = 5.0f);

	void spawnAAABB(GBAABB aabb);
	void spawnGrid(const GBGrid& grid);
	void spawnBox(const GBBoxCollider& box, const GBVector3& color = GBVector3(1, 1, 1), bool translucent = false);
	void spawnSphere(const GBSphereCollider& sphere, const GBVector3& color = GBVector3(1, 1, 1), bool translucent = false);
	void spawnCapsule(const GBCapsuleCollider& capsule, const GBVector3& color = GBVector3(1, 1, 1), bool translucent = false);
	void setActorPosition(AActor* actor, GBVector3 position);
	void setActorRotation(AActor* Actor, const GBQuaternion& Rot);
	void updateSimulationActors();
	void setRenderableData(GBCollider* pCollider, GBVector3 color, bool translucent = false, bool drawWireFrame = false, bool doRender = true, bool ignore = false, std::string name = "");
	//void setRenderableData(GBBody* pBody, GBVector3 color, bool translucent = false, bool drawWireFrame = false, bool doRender = true, bool ignore = false, std::string name = "");
	GBCollider* CreateColliderFromMesh(AActor* Actor, ColliderType colType, GBBody* pBody);
	void RegisterExistingColliders(const FString path, ColliderType colType, GBBody** pBodyReference = nullptr, bool doUseBodyReference = false);
	void moveBody(UWorld* world, GBBody& body, float speed = 10.0f, float jumpSpeed = 10.0f);
	void controlBody(UWorld* world, GBBody& body, float speed = 10.0f, float jumpSpeed = 15.0f);
	void followCamera(UWorld* world, GBBody& body);
	GBBody* getBodyByName(UWorld* world, std::string name);
	std::vector<GBBody*> getBodiesByTag(UWorld* world, std::string tag);

	static FVector toFVector(GBVector3 v, bool scale = true);
	static FQuat toFQuat(const GBQuaternion& q);
	static GBVector3 toGBVector(FVector v);
	static GBQuaternion toGBQuat(const FQuat& q);


	UPROPERTY(VisibleAnywhere)
	UCameraComponent* CameraComponent;

	UPROPERTY()
	TMap<uint32, AActor*> BoxActors;

	UPROPERTY()
	TMap<uint32, AActor*> SphereActors;

	UPROPERTY()
	TMap<uint32, AActor*> CapsuleActors;


	TMap<uint32, UStaticMeshComponent*> SphereMeshes;
	TMap<uint32, UStaticMeshComponent*> BoxMeshes;
	TMap<uint32, UStaticMeshComponent*> CapsuleMeshes;
	TMap<uint32, UStaticMeshComponent*> CapsuleTopSphereMeshes;
	TMap<uint32, UStaticMeshComponent*> CapsuleBottomSphereMeshes;

	UPROPERTY(EditDefaultsOnly)
	UMaterialInterface* GBDefaultMaterial;

	UPROPERTY(EditDefaultsOnly)
	UMaterialInterface* GBTranslucentMaterial;

	GBSimulation simulation;
	GBCapsuleCollider* goalTriggerCapsule = nullptr;
	FString nextLevelString = "";

	void spawnActorsFromSimulation();
	bool doSpawnActors = true;
	bool doUpdateActors = true;
	bool doClearActorsAfterStart = false;
	bool doSpawnGrid = false;
	bool doAutoRegisterActor = true;
	bool doUpdateCamera = true;
	bool doDrawAllPhysicsColliders = false;
	void drawAllPhysicsColliders();
	bool doDrawAABBs = false;
	bool doDelayStart = false;
	void toggleDelayStart();

	GBBody* shootableBody = nullptr;
	float shootSpeed = 2000.0f;
	const static int shootStackAmount = 10;
	GBBody* shootStack[shootStackAmount];
	int shootIndex = 0;
	int shootCount = 1;



	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bDoPhysicsStep = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bControlGravityBall = false;

	//UPROPERTY()
	//AAGBTriangleMesh* MeshActor;
	bool doSpawnMeshActor = false;

	void deleteActorAndBody(GBBody* pBody);

	void setCameraTransform(const GBVector3& position, const GBQuaternion& rotation = GBQuaternion());
	void setCameraPosition(const GBVector3& position);

	// Procedural mesh component
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mesh")
	UProceduralMeshComponent* MeshComponent;

	// Arrays to hold vertex and triangle data
	UPROPERTY()
	TArray<FVector> Vertices;

	UPROPERTY()
	TArray<int32> Triangles;

	// Function to add a triangle
	void AddTriangle(const FVector& A, const FVector& B, const FVector& C, bool updateMesh = false);

	// Function to update the mesh
	void UpdateMesh();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh")
	UMaterialInterface* TriangleMaterial;

	using HeightFunc = std::function<bool(int32, int32, float&)>;
	bool GetHeight(int32 X, int32 Y, float& outHeight);
	bool GetEllipseHeight(int32 X, int32 Y, float& outHeight);
	std::vector<std::vector<GBTriangle>> createHeightmapTerrainTriangles(HeightFunc GetHeightFunc, const int sizeX = 100, const int sizeY = 100, float spacing = 1.0f, GBVector3 originOffset = GBVector3::zero());
	void updateTerrainMeshActor(const std::vector<std::vector<GBTriangle>>& triangles);
	bool wasdMoveBindings = false;


private:
	float Yaw = 0.f;
	float Pitch = 0.f;
	float runModifier = 5.0f;
};
