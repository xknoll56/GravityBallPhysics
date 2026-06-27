// Fill out your copyright notice in the Description page of Project Settings.


#include "PGBWorld.h"
#include "Components/BoxComponent.h"
#include "GB_Collider.h"
#include "AGBGrid.h"
#include "DrawDebugHelpers.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "EngineUtils.h" 
#include "CoreMinimal.h"

// Sets default values
APGBWorld::APGBWorld()
{
	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	AutoPossessPlayer = EAutoReceiveInput::Player0;

	CameraComponent = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	CameraComponent->SetupAttachment(RootComponent);
	CameraComponent->bUsePawnControlRotation = false; // We'll handle rotation manually

	static ConstructorHelpers::FObjectFinder<UMaterialInterface> DefaultMat(TEXT("/Game/Materials/GBDefault.GBDefault"));
	if (DefaultMat.Succeeded())
	{
		GBDefaultMaterial = DefaultMat.Object;
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to load GBDefault material!"));
	}

	static ConstructorHelpers::FObjectFinder<UMaterialInterface> TransMat(TEXT("/Game/Materials/GBTranslucent.GBTranslucent"));
	if (TransMat.Succeeded())
	{
		GBTranslucentMaterial = TransMat.Object;
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to load GBTranslucent material!"));
	}

	// Create the mesh component
	MeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh"));
}

// Called when the game starts or when spawned
void APGBWorld::BeginPlay()
{
	Super::BeginPlay();


	FString levelName = GetWorld()->GetMapName();
	levelName.RemoveFromStart(GetWorld()->StreamingLevelsPrefix);

	if (levelName == "BoxStack")
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
			shootableBody->transform.position = { -25.0f,-10.0f + 2.0f*i,0.5f };
			GBSphereCollider* pSphere = simulation.attachSphereCollider(shootableBody, 0.5f + (i*0.1f)*0.75);
			pSphere->pData = new RenderableCollider({ 0.1f*(i%2),0.76f*(i+1%2), 0.92f});
			shootableBody->setMass(5.0f * pBox->volume());
			shootStack[i] = shootableBody;
		}

		shootableBody = shootStack[0];
		shootCount = 10;

		pBody = simulation.createBody();
		pBox = simulation.attachBoxCollider(pBody, { 50.0f,50.0f,0.1f });
		pBody->transform.position = { 0.0f,0.0f,-0.05f };
		pBody->isStatic = true;
		sceneEnum = SCENE_BOX;

	}


	bDoPhysicsStep = true;
	doSpawnActors = true;
	doDrawAllPhysicsColliders = true;


	//bool autoRegister = true;
	if (doSpawnActors)
	{
		spawnActorsFromSimulation();


		//autoRegister = false;
	}

	if (doAutoRegisterActor)
	{
		//RegisterExistingGameBoxes(TEXT("Blueprint'/Game/Colliders/Box.Box_C'"));
		RegisterExistingColliders(TEXT("Blueprint'/Game/Colliders/Box.Box_C'"), ColliderType::Box);
		RegisterExistingColliders(TEXT("Blueprint'/Game/Colliders/Sphere.Sphere_C'"), ColliderType::Sphere);
		RegisterExistingColliders(TEXT("Blueprint'/Game/Colliders/Capsule.Capsule_C'"), ColliderType::Capsule);

	}

	//if (doSpawnMeshActor)
	//{
	//	MeshActor = GetWorld()->SpawnActor<AAGBTriangleMesh>(
	//		AAGBTriangleMesh::StaticClass(),
	//		FVector(),
	//		FRotator::ZeroRotator
	//	);
	//}

	if (doClearActorsAfterStart)
	{
		for (auto& bod : simulation.rigidBodies)
		{
			GBBody* pBody = bod.get();
			deleteActorAndBody(pBody);
		}
	}

	shootStack[0] = shootableBody;


	if (TriangleMaterial)
	{
		MeshComponent->SetMaterial(0, TriangleMaterial);
	}


}

void APGBWorld::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
}


GBCollider* APGBWorld::CreateColliderFromMesh(AActor* Actor, ColliderType colType, GBBody* pBody)
{
	switch (colType)
	{
	case ColliderType::Box:
	{
		UStaticMeshComponent* MeshComp =
			Actor->FindComponentByClass<UStaticMeshComponent>();
		if (!MeshComp)
		{
			return nullptr;
		}

		MeshComp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		MeshComp->SetGenerateOverlapEvents(false);
		MeshComp->SetNotifyRigidBodyCollision(false);
		MeshComp->SetSimulatePhysics(false);

		// Unreal bounds are in cm
		FVector HalfExtentsCM =
			MeshComp->GetStaticMesh()->GetBounds().BoxExtent *
			MeshComp->GetComponentScale();

		GBBoxCollider* col = simulation.attachBoxCollider(pBody, toGBVector(HalfExtentsCM), GBTransform());
		int32 Id = col->id;
		BoxActors.Add(Id, Actor);
		BoxMeshes.Add(Id, MeshComp);

		return col;
	}

	break;
	case ColliderType::Sphere:
	{

		UStaticMeshComponent* MeshComp =
			Actor->FindComponentByClass<UStaticMeshComponent>();
		if (!MeshComp)
		{
			return nullptr;
		}

		MeshComp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		MeshComp->SetGenerateOverlapEvents(false);
		MeshComp->SetNotifyRigidBodyCollision(false);
		MeshComp->SetSimulatePhysics(false);

		// Unreal bounds are in cm
		FVector HalfExtentsCM =
			MeshComp->GetStaticMesh()->GetBounds().BoxExtent *
			MeshComp->GetComponentScale();


		GBSphereCollider* col = simulation.attachSphereCollider(pBody, HalfExtentsCM.X / 100.0f, GBTransform());
		int32 Id = col->id;
		SphereActors.Add(Id, Actor);
		SphereMeshes.Add(Id, MeshComp);
		return col;
	}
	break;
	case ColliderType::Capsule:
	{
		UStaticMeshComponent* Cylinder = nullptr;
		UStaticMeshComponent* TopSphere = nullptr;
		UStaticMeshComponent* BottomSphere = nullptr;

		TArray<UStaticMeshComponent*> Meshes;
		Actor->GetComponents<UStaticMeshComponent>(Meshes);

		UStaticMesh* SphereMesh =
			LoadObject<UStaticMesh>(
				nullptr,
				TEXT("/Engine/BasicShapes/Sphere.Sphere")
			);

		for (UStaticMeshComponent* Comp : Meshes)
		{
			if (!Comp) continue;

			if (Comp->GetFName() == TEXT("Cylinder"))
				Cylinder = Comp;
			else if (Comp->GetFName() == TEXT("TopSphere"))
				TopSphere = Comp;
			else if (Comp->GetFName() == TEXT("BottomSphere"))
				BottomSphere = Comp;
		}

		if (!Cylinder)
		{
			return nullptr;
		}

		// Cylinder dimensions
		FVector HalfExtentsCM =
			Cylinder->GetStaticMesh()->GetBounds().BoxExtent *
			Cylinder->GetComponentScale();

		float Radius = HalfExtentsCM.X / 100.0f;
		float Height = HalfExtentsCM.Z / 50.0f;

		// Auto-create spheres if missing
		UMaterialInterface* CylinderMat =
			Cylinder->GetMaterial(0);

		float HalfHeightCM = HalfExtentsCM.Z;
		USceneComponent* Comp = Actor->FindComponentByClass<USceneComponent>();

		if (!TopSphere)
		{
			TopSphere =
				NewObject<UStaticMeshComponent>(
					Actor,
					TEXT("TopSphere")
				);

			TopSphere->RegisterComponent();

			TopSphere->AttachToComponent(
				Comp,
				FAttachmentTransformRules::KeepRelativeTransform
			);

			TopSphere->SetStaticMesh(SphereMesh);

			if (CylinderMat)
				TopSphere->SetMaterial(0, CylinderMat);

			TopSphere->SetRelativeScale3D(
				FVector(
					Cylinder->GetComponentScale().X,
					Cylinder->GetComponentScale().X,
					Cylinder->GetComponentScale().X
				)
			);

			TopSphere->SetRelativeLocation(
				FVector(0, 0, HalfHeightCM)
			);
		}

		if (!BottomSphere)
		{
			BottomSphere =
				NewObject<UStaticMeshComponent>(
					Actor,
					TEXT("BottomSphere")
				);

			BottomSphere->RegisterComponent();

			BottomSphere->AttachToComponent(
				Comp,
				FAttachmentTransformRules::KeepRelativeTransform
			);

			BottomSphere->SetStaticMesh(SphereMesh);

			if (CylinderMat)
				BottomSphere->SetMaterial(0, CylinderMat);

			BottomSphere->SetRelativeScale3D(
				FVector(
					Cylinder->GetComponentScale().X,
					Cylinder->GetComponentScale().X,
					Cylinder->GetComponentScale().X
				)
			);

			BottomSphere->SetRelativeLocation(
				FVector(0, 0, -HalfHeightCM)
			);
		}

		Cylinder->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Cylinder->SetGenerateOverlapEvents(false);
		Cylinder->SetNotifyRigidBodyCollision(false);
		Cylinder->SetSimulatePhysics(false);

		TopSphere->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		TopSphere->SetGenerateOverlapEvents(false);
		TopSphere->SetNotifyRigidBodyCollision(false);
		TopSphere->SetSimulatePhysics(false);

		BottomSphere->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		BottomSphere->SetGenerateOverlapEvents(false);
		BottomSphere->SetNotifyRigidBodyCollision(false);
		BottomSphere->SetSimulatePhysics(false);

		GBCapsuleCollider* col =
			simulation.attachCapsuleCollider(
				pBody,
				Radius,
				Height,
				GBTransform()
			);
		if (Cylinder)
		{
			CapsuleMeshes.Add(col->id, Cylinder);
			CapsuleActors.Add(col->id, Actor);
		}
		return col;
	}
	break;
	}
	return nullptr;
}

void APGBWorld::RegisterExistingColliders(const FString path, ColliderType colType, GBBody** pBodyReference, bool doUseBodyReference)
{
	UClass* ColBPClass = StaticLoadClass(AActor::StaticClass(), nullptr, *path);
	if (!ColBPClass) return;

	for (TActorIterator<AActor> It(GetWorld(), ColBPClass); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor) continue;

		Actor->SetActorEnableCollision(false);

		AActor* parent = Actor->GetParentActor();

		UGB_Collider* ColliderComp = nullptr;

		if (parent)
		{
			ColliderComp = parent->FindComponentByClass<UGB_Collider>();
			parent->SetActorEnableCollision(false);
		}

		if (!ColliderComp)
			ColliderComp = Actor->FindComponentByClass<UGB_Collider>();

		if (!ColliderComp || !ColliderComp->wasDropped)
			continue;


		GBBody* body = simulation.createBody();
		body->transform.position = toGBVector(It->GetActorLocation());
		body->transform.rotation = toGBQuat(It->GetActorQuat());

		GBCollider* pCol = CreateColliderFromMesh(Actor, colType, body);
		if (!pCol)
			continue;

		body->isStatic = ColliderComp->isStatic;
		body->isSleeping = ColliderComp->isSleeping;
		body->isTrigger = ColliderComp->isTrigger;
		body->mask = ColliderComp->mask;
		body->layer = ColliderComp->layer;

		// Editor label (editor-only)
		FString Label = Actor->GetName();
		// FString -> std::string
		std::string StdStr = TCHAR_TO_UTF8(*Label);

		if(!pCol->pData)
			pCol->pData = new RenderableCollider({ 1,1,1 }, true, false, true, true, StdStr);

		FString Tag;

		if (Actor->Tags.Num() > 0)
		{
			Tag = Actor->Tags[0].ToString();
		}
		else
		{
			Tag = Actor->GetName();
		}
		std::string StdTag = TCHAR_TO_UTF8(*Tag);
		body->pData = new BodyTag(StdTag);

		if (doUseBodyReference)
			*pBodyReference = body;
	}
}

void APGBWorld::spawnBox(
	const GBBoxCollider& box,
	const GBVector3& color,
	bool translucent
)
{
	FString Path = TEXT("Blueprint'/Game/Colliders/Box.Box_C'");
	UClass* BoxBPClass = StaticLoadClass(AActor::StaticClass(), nullptr, *Path);

	if (!BoxBPClass)
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to load Box Blueprint!"));
		return;
	}

	FActorSpawnParameters SpawnParams;
	SpawnParams.Owner = this;

	AActor* NewBox = GetWorld()->SpawnActor<AActor>(
		BoxBPClass,
		toFVector(box.transform.position),
		FRotator::ZeroRotator,
		SpawnParams
	);

	if (!NewBox) return;

	BoxActors.Add(box.id, NewBox);
	NewBox->SetActorEnableCollision(false);

	UStaticMeshComponent* MeshComp = NewBox->FindComponentByClass<UStaticMeshComponent>();
	if (!MeshComp) return;

	BoxMeshes.Add(box.id, MeshComp);
	// Scale (half-extents → full size, cm)
	FVector Scale = toFVector(box.halfExtents) * 2.f / 100.f;
	MeshComp->SetWorldScale3D(Scale);

	UMaterialInterface* BaseMat =
		translucent ? GBTranslucentMaterial : GBDefaultMaterial;

	if (!BaseMat)
	{
		UE_LOG(LogTemp, Warning, TEXT("Base material not assigned!"));
		return;
	}

	UMaterialInstanceDynamic* DynMat =
		UMaterialInstanceDynamic::Create(BaseMat, this);

	MeshComp->SetMaterial(0, DynMat);

	// --- Parameters ---
	if (DynMat)
	{
		DynMat->SetVectorParameterValue(
			TEXT("Color"),
			FLinearColor(color.x, color.y, color.z)
		);
	}

	if (NewBox)
	{
		UGB_Collider* ColliderComp =
			NewBox->FindComponentByClass<UGB_Collider>();

		if (ColliderComp)
			ColliderComp->wasDropped = false;

		box.pBody->mask = ColliderComp->mask;
		box.pBody->layer = ColliderComp->layer;
	}
}


void APGBWorld::spawnSphere(
	const GBSphereCollider& sphere,
	const GBVector3& color,
	bool translucent
)
{
	FString Path = TEXT("Blueprint'/Game/Colliders/Sphere.Sphere_C'");
	UClass* SphereBPClass = StaticLoadClass(AActor::StaticClass(), nullptr, *Path);

	if (!SphereBPClass)
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to load Sphere Blueprint!"));
		return;
	}

	FActorSpawnParameters SpawnParams;
	SpawnParams.Owner = this;

	AActor* NewSphere = GetWorld()->SpawnActor<AActor>(
		SphereBPClass,
		toFVector(sphere.transform.position),
		FRotator::ZeroRotator,
		SpawnParams
	);

	if (!NewSphere) return;

	SphereActors.Add(sphere.id, NewSphere);
	NewSphere->SetActorEnableCollision(false);

	UStaticMeshComponent* MeshComp = NewSphere->FindComponentByClass<UStaticMeshComponent>();
	if (!MeshComp) return;

	MeshComp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	MeshComp->SetGenerateOverlapEvents(false);
	MeshComp->SetNotifyRigidBodyCollision(false);
	MeshComp->SetSimulatePhysics(false);

	SphereMeshes.Add(sphere.id, MeshComp);
	// Scale
	FVector Scale = toFVector(
		GBVector3(sphere.radius, sphere.radius, sphere.radius)
	) * 2.f / 100.f;

	MeshComp->SetWorldScale3D(Scale);


	UMaterialInterface* BaseMat =
		translucent ? GBTranslucentMaterial : GBDefaultMaterial;

	if (!BaseMat)
	{
		UE_LOG(LogTemp, Warning, TEXT("Base material not assigned!"));
		return;
	}

	UMaterialInstanceDynamic* DynMat =
		UMaterialInstanceDynamic::Create(BaseMat, this);

	MeshComp->SetMaterial(0, DynMat);

	// --- Parameters ---
	if (DynMat)
	{
		DynMat->SetVectorParameterValue(
			TEXT("Color"),
			FLinearColor(color.x, color.y, color.z)
		);
	}

	if (NewSphere)
	{
		UGB_Collider* ColliderComp =
			NewSphere->FindComponentByClass<UGB_Collider>();

		if (ColliderComp)
			ColliderComp->wasDropped = false;
		sphere.pBody->mask = ColliderComp->mask;
		sphere.pBody->layer = ColliderComp->layer;
	}
}

void APGBWorld::spawnCapsule(const GBCapsuleCollider& capsule, const GBVector3& color, bool translucent)
{
	FString Path = TEXT("Blueprint'/Game/Colliders/Capsule.Capsule_C'");
	UClass* CapsuleBPClass = StaticLoadClass(AActor::StaticClass(), nullptr, *Path);

	if (!CapsuleBPClass)
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to load Capsule Blueprint!"));
		return;
	}

	FActorSpawnParameters SpawnParams;
	SpawnParams.Owner = this;

	AActor* NewCapsule = GetWorld()->SpawnActor<AActor>(
		CapsuleBPClass,
		toFVector(capsule.transform.position),
		FRotator::ZeroRotator,
		SpawnParams
	);

	if (!NewCapsule) return;

	CapsuleActors.Add(capsule.id, NewCapsule);
	NewCapsule->SetActorEnableCollision(false);

	// 🔥 Get all mesh components and filter by name
	TArray<UStaticMeshComponent*> MeshComponents;
	NewCapsule->GetComponents<UStaticMeshComponent>(MeshComponents);

	UStaticMeshComponent* Cylinder = nullptr;
	UStaticMeshComponent* TopSphere = nullptr;
	UStaticMeshComponent* BottomSphere = nullptr;

	for (UStaticMeshComponent* Comp : MeshComponents)
	{
		if (!Comp) continue;

		if (Comp->GetFName() == TEXT("Cylinder"))
			Cylinder = Comp;
		else if (Comp->GetFName() == TEXT("TopSphere"))
			TopSphere = Comp;
		else if (Comp->GetFName() == TEXT("BottomSphere"))
			BottomSphere = Comp;
	}

	// Store if needed
	if (Cylinder) CapsuleMeshes.Add(capsule.id, Cylinder);

	// --- Scale ---
	float RadiusScale = capsule.radius * 2.f;
	float HeightScale = capsule.height;

	if (Cylinder)
	{
		Cylinder->SetWorldScale3D(FVector(RadiusScale, RadiusScale, HeightScale));

		Cylinder->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Cylinder->SetGenerateOverlapEvents(false);
		Cylinder->SetNotifyRigidBodyCollision(false);
		Cylinder->SetSimulatePhysics(false);
	}


	float HalfHeight = capsule.height * 0.5f * 100.0f;

	if (TopSphere)
	{
		TopSphere->SetWorldScale3D(FVector(RadiusScale));
		TopSphere->SetRelativeLocation(FVector(0, 0, HalfHeight));
		TopSphere->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		TopSphere->SetGenerateOverlapEvents(false);
		TopSphere->SetNotifyRigidBodyCollision(false);
		TopSphere->SetSimulatePhysics(false);
		CapsuleTopSphereMeshes.Add(capsule.id, TopSphere);
	}

	if (BottomSphere)
	{
		BottomSphere->SetWorldScale3D(FVector(RadiusScale));
		BottomSphere->SetRelativeLocation(FVector(0, 0, -HalfHeight));
		BottomSphere->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		BottomSphere->SetGenerateOverlapEvents(false);
		BottomSphere->SetNotifyRigidBodyCollision(false);
		BottomSphere->SetSimulatePhysics(false);
		CapsuleBottomSphereMeshes.Add(capsule.id, BottomSphere);
	}



	UMaterialInterface* BaseMat =
		translucent ? GBTranslucentMaterial : GBDefaultMaterial;

	if (!BaseMat)
	{
		UE_LOG(LogTemp, Warning, TEXT("Base material not assigned!"));
		return;
	}

	UMaterialInstanceDynamic* DynMat =
		UMaterialInstanceDynamic::Create(BaseMat, this);

	if (Cylinder) Cylinder->SetMaterial(0, DynMat);
	if (TopSphere) TopSphere->SetMaterial(0, DynMat);
	if (BottomSphere) BottomSphere->SetMaterial(0, DynMat);

	if (DynMat)
	{
		DynMat->SetVectorParameterValue(
			TEXT("Color"),
			FLinearColor(color.x, color.y, color.z)
		);
	}

	if (NewCapsule)
	{
		UGB_Collider* ColliderComp =
			NewCapsule->FindComponentByClass<UGB_Collider>();

		if (ColliderComp)
			ColliderComp->wasDropped = false;

		capsule.pBody->mask = ColliderComp->mask;
		capsule.pBody->layer = ColliderComp->layer;
	}
}


void APGBWorld::spawnActorsFromSimulation()
{
	for (GBCollider* Col : simulation.colliders)
	{
		if (Col)
		{
			if (Col->pBody != nullptr && Col->pBody->ignoreSample)
				continue;
			GBVector3 color(1.0f, 1.0f, 1.0f);
			bool translucent = false;
			if (Col->pData != nullptr)
			{
				RenderableCollider* rc = (RenderableCollider*)Col->pData;
				if (rc->ignoreSpawn)
					continue;
				color = rc->color;
				translucent = rc->translucent;
			}
			switch (Col->type)
			{
			case ColliderType::Sphere:
				spawnSphere(*static_cast<GBSphereCollider*>(Col), color, translucent);
				break;

			case ColliderType::Box:
				spawnBox(*static_cast<GBBoxCollider*>(Col), color, translucent);
				break;
			case ColliderType::Capsule:
				spawnCapsule(*static_cast<GBCapsuleCollider*>(Col), color, translucent);
				break;
			}
		}
	}

	if (doSpawnGrid)
	{
		for (auto it = simulation.gridMap.grids.begin(); it != simulation.gridMap.grids.end(); it++)
		{
			spawnGrid(it->second);
		}
	}

}

void APGBWorld::setActorPosition(AActor* actor, GBVector3 position)
{
	if (actor)
	{
		FVector NewLocation = toFVector(position);
		actor->SetActorLocation(NewLocation);
	}
}

void APGBWorld::setActorRotation(AActor* Actor, const GBQuaternion& Rot)
{
	if (!Actor)
	{
		return;
	}

	// Convert GBQuaternion to Unreal FQuat
	FQuat UnrealQuat = toFQuat(Rot);

	// Apply rotation
	Actor->SetActorRotation(UnrealQuat);
}

RenderableCollider* APGBWorld::getRenderable(GBCollider* collider)
{
	if (collider->pData)
	{
		return (RenderableCollider*)collider->pData;
	}
	return nullptr;
}

void APGBWorld::updateSimulationActors()
{

	for (int i = 0; i < simulation.rigidBodies.size(); i++)
	{
		GBBody* pBody = simulation.rigidBodies[i].get();
		if (pBody->ignoreSample)
			continue;
		for (GBCollider* col : pBody->colliders)
		{
			AActor* actor;
			RenderableCollider* pRenderable = (RenderableCollider*)col->pData;
			bool renderFlag = true;
			if (col->pData)
			{
				pRenderable = (RenderableCollider*)col->pData;
				renderFlag = pRenderable->doRender;
			}
			switch (col->type)
			{
			case ColliderType::Sphere:

				if (pRenderable && pRenderable->drawWireFrame)
				{
					GBSphereCollider* pSphere = (GBSphereCollider*)col;
					drawSphereCollider(GetWorld(), *pSphere);
				}
				if (renderFlag)
				{
					if (SphereMeshes.Contains(col->id))
					{
						if (UStaticMeshComponent* MeshComp = SphereMeshes[col->id])
						{
							actor = SphereActors[col->id];
							setActorPosition(actor, col->transform.position);
							setActorRotation(actor, col->transform.rotation);
							MeshComp->SetVisibility(renderFlag);
						}
					}
				}
				else
				{
					if (UStaticMeshComponent** MeshCompPtr = SphereMeshes.Find(col->id))
					{
						if (*MeshCompPtr)
						{
							(*MeshCompPtr)->SetVisibility(renderFlag);
						}
					}
				}
				break;

			case ColliderType::Box:
				if (pRenderable && pRenderable->drawWireFrame)
				{
					GBBoxCollider* pBox = (GBBoxCollider*)col;
					drawBox(GetWorld(), *pBox);
				}

				if (renderFlag)
				{
					if (BoxMeshes.Contains(col->id))
					{
						if (UStaticMeshComponent* MeshComp = BoxMeshes[col->id])
						{
							actor = BoxActors[col->id];
							setActorPosition(actor, col->transform.position);
							setActorRotation(actor, col->transform.rotation);
						}
					}
				}
				else
				{
					if (UStaticMeshComponent** MeshCompPtr = BoxMeshes.Find(col->id))
					{
						if (*MeshCompPtr)
						{
							(*MeshCompPtr)->SetVisibility(renderFlag);
						}
					}
				}
				break;

			case ColliderType::Capsule:
				if (pRenderable && pRenderable->drawWireFrame)
				{
					GBCapsuleCollider* pCapsule = (GBCapsuleCollider*)col;
					drawCapsule(GetWorld(), *pCapsule);
				}

				if (renderFlag)
				{
					if (CapsuleMeshes.Contains(col->id))
					{
						if (UStaticMeshComponent* MeshComp = CapsuleMeshes[col->id])
						{
							actor = CapsuleActors[col->id];
							setActorPosition(actor, col->transform.position);
							setActorRotation(actor, col->transform.rotation);
							MeshComp->SetVisibility(renderFlag);
						}
					}
					if ((CapsuleTopSphereMeshes.Contains(col->id)))
					{
						if (UStaticMeshComponent* MeshComp = CapsuleTopSphereMeshes[col->id])
						{
							MeshComp->SetVisibility(renderFlag);
						}
					}
					if ((CapsuleBottomSphereMeshes.Contains(col->id)))
					{
						if (UStaticMeshComponent* MeshComp = CapsuleBottomSphereMeshes[col->id])
						{
							MeshComp->SetVisibility(renderFlag);
						}
					}
				}
				else
				{
					if (UStaticMeshComponent** MeshCompPtr = CapsuleMeshes.Find(col->id))
					{
						if (*MeshCompPtr)
						{
							(*MeshCompPtr)->SetVisibility(renderFlag);
						}
					}
					if (UStaticMeshComponent** MeshCompPtr = CapsuleTopSphereMeshes.Find(col->id))
					{
						if (*MeshCompPtr)
						{
							(*MeshCompPtr)->SetVisibility(renderFlag);
						}
					}
					if (UStaticMeshComponent** MeshCompPtr = CapsuleBottomSphereMeshes.Find(col->id))
					{
						if (*MeshCompPtr)
						{
							(*MeshCompPtr)->SetVisibility(renderFlag);
						}
					}
				}
				break;
			}
		}
	}
}

void APGBWorld::drawAllPhysicsColliders()
{
	for (int i = 0; i < simulation.rigidBodies.size(); i++)
	{
		GBBody* pBody = simulation.rigidBodies[i].get();
		for (GBCollider* col : pBody->colliders)
		{
			switch (col->type)
			{
			case ColliderType::Sphere:

			{
				GBSphereCollider* pSphere = (GBSphereCollider*)col;
				drawSphereCollider(GetWorld(), *pSphere, pSphere->pBody->isSleeping ? FColor::Green : FColor::Blue);
			}
			break;

			case ColliderType::Box:
			{
				GBBoxCollider* pBox = (GBBoxCollider*)col;
				drawBox(GetWorld(), *pBox, pBox->pBody->isSleeping ? FColor::Green : FColor::Blue);
			}
			break;

			case ColliderType::Capsule:
			{
				GBCapsuleCollider* pCapsule = (GBCapsuleCollider*)col;
				drawCapsule(GetWorld(), *pCapsule, pCapsule->pBody->isSleeping ? FColor::Green : FColor::Blue);
			}
			}
			if (doDrawAABBs)
				drawAABB(GetWorld(), col->aabb, col->pBody->isSleeping ? FColor::Cyan : FColor::Purple);
		}
	}
}

void APGBWorld::setRenderableData(GBCollider* pCollider, GBVector3 color, bool translucent, bool drawWireFrame, bool doRender, bool ignore, std::string name)
{
	pCollider->pData = (void*)(new RenderableCollider(color, translucent, drawWireFrame, doRender, ignore, name));

}


void APGBWorld::movePosition(float deltaTime, GBVector3& outPosition, float speed)
{
	// Get the player controller
	if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{
		// --- MOVEMENT ---
		FVector MoveDirection = FVector::ZeroVector;

		// Get camera forward and right vectors
		FVector Forward = CameraComponent->GetForwardVector();
		FVector Right = CameraComponent->GetRightVector();

		// Project onto horizontal plane (Z = 0)
		Forward.Z = 0.0f;
		Right.Z = 0.0f;

		Forward.Normalize();
		Right.Normalize();

		// Horizontal movement (arrows/WASD)
		if (!wasdMoveBindings)
		{
			if (PC->IsInputKeyDown(EKeys::Up)) MoveDirection += Forward;
			if (PC->IsInputKeyDown(EKeys::Down)) MoveDirection -= Forward;
			if (PC->IsInputKeyDown(EKeys::Right)) MoveDirection += Right;
			if (PC->IsInputKeyDown(EKeys::Left)) MoveDirection -= Right;

			// Vertical movement (Shift/Control)
			if (PC->IsInputKeyDown(EKeys::RightShift)) MoveDirection += FVector::UpVector;
			if (PC->IsInputKeyDown(EKeys::RightControl)) MoveDirection -= FVector::UpVector;
		}
		else
		{
			if (PC->IsInputKeyDown(EKeys::W)) MoveDirection += Forward;
			if (PC->IsInputKeyDown(EKeys::S)) MoveDirection -= Forward;
			if (PC->IsInputKeyDown(EKeys::D)) MoveDirection += Right;
			if (PC->IsInputKeyDown(EKeys::A)) MoveDirection -= Right;

			// Vertical movement (Shift/Control)
			if (PC->IsInputKeyDown(EKeys::E)) MoveDirection += FVector::UpVector;
			if (PC->IsInputKeyDown(EKeys::Q)) MoveDirection -= FVector::UpVector;
		}



		// Apply movement if there is input
		if (!MoveDirection.IsNearlyZero())
		{
			MoveDirection.Normalize();

			float MoveSpeed = speed; // units/sec
			outPosition += GBVector3(MoveDirection.X, MoveDirection.Y, MoveDirection.Z) * MoveSpeed * deltaTime;
		}
	}
}

void APGBWorld::moveCamera(float deltaTime)
{
	// Get the player controller
	if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{
		// --- MOUSE LOOK ---
		float MouseX, MouseY;
		PC->GetInputMouseDelta(MouseX, MouseY);

		float MouseSensitivity = 1.0f; // tweak to your liking
		Yaw += MouseX * MouseSensitivity;
		Pitch += MouseY * MouseSensitivity;

		// Clamp pitch to avoid flipping
		Pitch = FMath::Clamp(Pitch, -89.f, 89.f);

		// Apply rotation
		FRotator NewRotation(Pitch, Yaw, 0.f);
		CameraComponent->SetWorldRotation(NewRotation);

		// --- MOVEMENT ---
		FVector MoveDirection = FVector::ZeroVector;

		// Use camera rotation for movement
		FVector Forward = CameraComponent->GetForwardVector();
		FVector Right = CameraComponent->GetRightVector();

		if (PC->IsInputKeyDown(EKeys::W)) MoveDirection += Forward;
		if (PC->IsInputKeyDown(EKeys::S)) MoveDirection -= Forward;
		if (PC->IsInputKeyDown(EKeys::D)) MoveDirection += Right;
		if (PC->IsInputKeyDown(EKeys::A)) MoveDirection -= Right;

		if (!MoveDirection.IsNearlyZero())
		{
			MoveDirection.Normalize();
			float MoveSpeed = 300.f; // units/sec
			if (PC->IsInputKeyDown(EKeys::LeftShift))
			{
				MoveSpeed *= runModifier;
			}
			AddActorWorldOffset(MoveDirection * MoveSpeed * deltaTime, true);
		}
		if (PC->WasInputKeyJustPressed(EKeys::SpaceBar))
		{
			if (shootCount > 0)
			{
				shootableBody = shootStack[shootIndex];
				if (shootableBody)
				{
					shootableBody->reset();
					shootableBody->transform.position = toGBVector(this->GetActorLocation() + Forward * 100.0f);
					shootableBody->velocity = toGBVector(Forward) * shootSpeed;
				}

				shootIndex++;
				if (shootIndex >= shootCount)
				{
					shootIndex = 0;
				}
			}
		}

	}
}

// Called to bind functionality to input
void APGBWorld::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

}

void APGBWorld::spawnGrid(const GBGrid& grid)
{
	if (!GetWorld()) return;

	FActorSpawnParameters SpawnParams;
	SpawnParams.Owner = this;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	// Spawn location and rotation (anchor as world location)
	FVector SpawnLocation = toFVector(grid.origin, false); // convert GBVector3 to FVector
	FRotator SpawnRotation = FRotator::ZeroRotator;

	// Spawn the grid actor
	AAGBGrid* GridActor = GetWorld()->SpawnActor<AAGBGrid>(
		AAGBGrid::StaticClass(),
		SpawnLocation,
		SpawnRotation,
		SpawnParams
	);

	if (GridActor)
	{
		// Set grid parameters
		GridActor->Anchor = toFVector(grid.origin, false);
		GridActor->CellsX = grid.cellsX;
		GridActor->CellsY = grid.cellsY;
		GridActor->CellsZ = grid.cellsZ;
		GridActor->CellSize = grid.cellSize;

		// Optional: set visual properties if you want
		GridActor->LineColor = FLinearColor::Green;
		GridActor->LineThickness = 1.5f;

		// Generate the grid lines
		GridActor->GenerateGridLines(); // make sure this function is public
	}
}


FVector APGBWorld::toFVector(GBVector3 v, bool scale)
{
	float s = scale ? 100.0f : 1.0f;
	return FVector(v.x * s, v.y * s, v.z * s);
}

FQuat APGBWorld::toFQuat(const GBQuaternion& q)
{
	return FQuat(q.x, q.y, q.z, q.w);
}

GBVector3 APGBWorld::toGBVector(FVector v)
{
	return GBVector3(v.X / 100.0f, v.Y / 100.0f, v.Z / 100.0f);
}

GBQuaternion APGBWorld::toGBQuat(const FQuat& q)
{
	return GBQuaternion(q.W, q.X, q.Y, q.Z);
}

void APGBWorld::drawAABB(UWorld* World, GBAABB aabb, FColor color, float thickness)
{
	FVector center = toFVector(aabb.center);
	FVector extents = toFVector(aabb.halfExtents);

	// Draw a wireframe debug box
	DrawDebugBox(
		World,
		center,         // Center of the box
		extents,         // Half-size
		color,    // Box color
		false,          // Persistent? false = only one frame
		0.f,            // Lifetime (0 = only one frame)
		0,              // Depth priority
		thickness             // Line thickness
	);
}

void APGBWorld::drawBox(UWorld* World, const GBBoxCollider& box, FColor color, float thickness)
{
	FVector position = toFVector(box.transform.position);
	FQuat rotation = toFQuat(box.transform.rotation);
	FVector extents = toFVector(box.halfExtents);


	// Draw a wireframe debug box
	DrawDebugBox(World, position, extents, rotation, color, false, 0.f, 0, thickness);
}

void APGBWorld::drawCapsule(UWorld* world, const GBCapsuleCollider& capsule, FColor color, float skin)
{
	FVector position = toFVector(capsule.transform.position);
	FQuat rotation = toFQuat(capsule.transform.rotation);
	float radius = capsule.radius * 100.0f; // convert to cm
	float halfHeight = capsule.height * 0.5f * 100.0f; // convert to cm
	// Draw a wireframe debug capsule
	DrawDebugCapsule(
		world,
		position,       // Center of the capsule
		halfHeight + radius,     // Half-height (cylinder part)
		radius + skin,         // Radius of the hemispheres
		rotation,       // Rotation of the capsule
		color,          // Capsule color
		false,          // Persistent? false = only one frame
		0.f,            // Lifetime (0 = one frame)
		0,              // Depth priority
		2.f             // Line thickness
	);
}

void APGBWorld::drawTriangle(UWorld* World, const GBVector3& v0, const GBVector3& v1, const GBVector3& v2, FColor Color, float Thickness, int Steps, FColor fillColor)
{
	FVector A = APGBWorld::toFVector(v0);
	FVector B = APGBWorld::toFVector(v1);
	FVector C = APGBWorld::toFVector(v2);
	DrawDebugLine(World, A, B, Color, false, 0.f, 0, Thickness);
	DrawDebugLine(World, B, C, Color, false, 0.f, 0, Thickness);
	DrawDebugLine(World, C, A, Color, false, 0.f, 0, Thickness);

	for (int32 i = 1; i < Steps; ++i)
	{
		float t = (float)i / (float)Steps;

		FVector P0 = FMath::Lerp(A, B, t);
		FVector P1 = FMath::Lerp(A, C, t);

		DrawDebugLine(World, P0, P1, fillColor, false, 0.f, 0, Thickness);
	}
}

void APGBWorld::drawTriangle(UWorld* World, const GBTriangle& triangle, FColor Color, float Thickness, int Steps, FColor fillColor)
{
	drawTriangle(World, triangle.vertices[0], triangle.vertices[1], triangle.vertices[2], Color, Thickness, Steps, fillColor);
}

void APGBWorld::drawGrid(UWorld* World, const GBGrid& g, FColor color)
{
	for (int i = 0; i < g.cells.size(); i++)
	{
		GBAABB cellAABB = g.cells[i].toAABB();
		drawAABB(World, cellAABB, color);
	}
}

void APGBWorld::drawEdge(UWorld* World, GBEdge edge, float Radius, FColor Color)
{
	FVector A = toFVector(edge.a);
	FVector B = toFVector(edge.b);
	DrawDebugDirectionalArrow(
		World,
		A,
		B,
		Radius,   // arrow head size
		Color,
		false,
		0.f,
		0,
		2.f
	);
}

void APGBWorld::drawLine(UWorld* World, GBVector3 a, GBVector3 b, float Radius, FColor Color)
{
	FVector A = toFVector(a);
	FVector B = toFVector(b);
	DrawDebugDirectionalArrow(
		World,
		A,
		B,
		Radius,   // arrow head size
		Color,
		false,
		0.f,
		0,
		2.f
	);
}

void APGBWorld::drawRay(UWorld* World, GBVector3 start, GBContact contact, float Radius, FColor Color,
	bool doDrawContact, FColor contactColor)
{
	drawLine(World, start, contact.position, Radius, Color);
	if (doDrawContact)
		drawPoint(World, contact.position, Radius, contactColor);
}

void APGBWorld::drawSphere(UWorld* world, const GBVector3& center, float radius, FColor color, GBQuaternion rotation)
{
	FVector Center = APGBWorld::toFVector(center);
	DrawDebugCapsule(
		world,
		Center,
		50.f,        // half height (can be small)
		radius * 100.f,
		toFQuat(rotation),
		color,
		false,
		0.f,
		0,
		2.f
	);
}

void APGBWorld::drawSphereCollider(UWorld* world, const GBSphereCollider& sphere, FColor color)
{
	drawSphere(world, sphere.transform.position, sphere.radius, color, sphere.transform.rotation);
}

void APGBWorld::drawManifold(UWorld* world, const GBManifold& manifold, float size, FColor color)
{
	for (int i = 0; i < manifold.numContacts; i++)
	{
		drawContact(world, manifold.contacts[i], size, color);
	}
}

void APGBWorld::drawContact(UWorld* world, const GBContact& contact, float size, FColor color)
{
	FVector point = toFVector(contact.position);
	FVector normal = toFVector(contact.normal);
	DrawDebugSphere(
		world,
		point,
		size * 10.0f,   // convert meters → cm
		12,              // more segments
		color,
		false,
		0.f,
		0,
		2.f              // thickness
	);
	DrawDebugDirectionalArrow(
		world,
		point,
		point + normal * size,
		size * 2.0f,   // arrow head size
		color,
		false,
		0.f,
		0,
		2.f
	);
}

void APGBWorld::drawPoint(UWorld* World, const GBVector3& point, float Radius, FColor Color)
{
	FVector Location = toFVector(point);

	DrawDebugSphere(
		World,
		Location,      // Center of the sphere
		Radius,        // Sphere radius
		12,            // Segments (makes it look round enough)
		Color,         // Sphere color
		false,         // Persistent? false = only one frame
		0.f,           // Lifetime (0 = one frame)
		0,             // Depth priority
		2.f            // Line thickness
	);
}

void APGBWorld::drawQuad(UWorld* world, const GBQuad& quad, float size, FColor color, bool fill, float fillWidth)
{
	GBVector3 verts[4];
	quad.extractVerts(verts);
	FVector uVerts[4];
	for (int i = 0; i < 4; i++)
	{
		uVerts[i] = toFVector(verts[i]);
	}
	for (int i = 0; i < 4; i++)
	{
		DrawDebugLine(world, uVerts[i % 4], uVerts[(i + 1) % 4], color, false, -1.0f, 0u, size);
	}
	if (fill)
	{
		GBVector3 edge = verts[1] - verts[0];
		GBVector3 edgeDir = edge.normalized();
		GBVector3 tangent = verts[2] - verts[1];
		GBVector3 tangentDir = tangent.normalized();
		int segments = edge.length();
		for (int i = 1; i < segments; i++)
		{
			GBVector3 offset = edgeDir * i;
			GBVector3 point1 = verts[0] + offset;
			GBVector3 point2 = verts[0] + offset + tangent;

			DrawDebugLine(world, toFVector(point1), toFVector(point2), color, false, -1.0f, 0u, size);
		}
		segments = tangent.length();
		for (int i = 1; i < segments; i++)
		{
			GBVector3 offset = tangentDir * i;
			GBVector3 point1 = verts[0] + offset;
			GBVector3 point2 = verts[0] + offset + edge;

			DrawDebugLine(world, toFVector(point1), toFVector(point2), color, false, -1.0f, 0u, size);
		}
	}
}

void APGBWorld::drawFrame(UWorld* world, const GBFrame& frame, const GBVector3 position, float size)
{

	FVector unrealPosition = toFVector(position);
	DrawDebugLine(world, unrealPosition, unrealPosition + toFVector(frame.forward), FColor::Red, false, -1.0f, 0u, size);
	DrawDebugLine(world, unrealPosition, unrealPosition + toFVector(frame.right), FColor::Green, false, -1.0f, 0u, size);
	DrawDebugLine(world, unrealPosition, unrealPosition + toFVector(frame.up), FColor::Blue, false, -1.0f, 0u, size);
}

void APGBWorld::printVector(const GBVector3& vector, FColor color, float time)
{
	if (GEngine) // Make sure the engine pointer is valid
	{
		FString msg = FString::Printf(TEXT("Vector: X=%.2f, Y=%.2f, Z=%.2f"), vector.x, vector.y, vector.z);
		GEngine->AddOnScreenDebugMessage(-1, time, color, msg);
	}
}

void APGBWorld::printString(const std::string& string, FColor color, float time)
{
	if (GEngine)
	{
		FString msg(string.c_str());
		GEngine->AddOnScreenDebugMessage(-1, time, color, msg);
	}
}

// Called every frame
void APGBWorld::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	switch (sceneEnum)
	{
	case SCENE_BOX:
	{

	}
	break;
	}
	if (doUpdateCamera)
		moveCamera(DeltaTime);
	if (bDoPhysicsStep && !doDelayStart)
		simulation.step(DeltaTime);
	if (doUpdateActors)
		updateSimulationActors();
	if (doDrawAllPhysicsColliders)
		drawAllPhysicsColliders();


}

void APGBWorld::moveBody(UWorld* world, GBBody& body, float speed, float jumpSpeed)
{
	// Get the player controller
	if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{
		// --- MOVEMENT ---
		FVector MoveDirection = FVector::ZeroVector;

		// Get camera forward and right vectors
		FVector Forward = CameraComponent->GetForwardVector();
		FVector Right = CameraComponent->GetRightVector();

		// Project onto horizontal plane (Z = 0)
		Forward.Z = 0.0f;
		Right.Z = 0.0f;

		Forward.Normalize();
		Right.Normalize();

		// Horizontal movement (arrows/WASD)
		if (PC->IsInputKeyDown(EKeys::W)) MoveDirection += Forward;
		if (PC->IsInputKeyDown(EKeys::S)) MoveDirection -= Forward;
		if (PC->IsInputKeyDown(EKeys::D)) MoveDirection += Right;
		if (PC->IsInputKeyDown(EKeys::A)) MoveDirection -= Right;

		// Vertical movement (Shift/Control)
		if (PC->IsInputKeyDown(EKeys::RightShift)) MoveDirection += FVector::UpVector;
		if (PC->IsInputKeyDown(EKeys::RightControl)) MoveDirection -= FVector::UpVector;

		// Apply movement if there is input
		if (!MoveDirection.IsNearlyZero())
		{
			MoveDirection.Normalize();
			float MoveSpeed = 1.0f;
			if (PC->IsInputKeyDown(EKeys::LeftShift))
			{
				MoveSpeed *= runModifier;
			}
			GBVector3 move = { (float)MoveDirection.X, (float)MoveDirection.Y, (float)MoveDirection.Z };
			body.wake();
			body.addForce(move * speed);
		}

		if (body.isGrounded)
		{
			if (PC->IsInputKeyDown(EKeys::SpaceBar))
			{
				body.velocity += jumpSpeed * GBVector3::up();
			}
		}
	}
}

void APGBWorld::controlBody(UWorld* world, GBBody& body, float speed, float jumpSpeed)
{
	// Get the player controller
	if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{

		FVector Forward = CameraComponent->GetForwardVector();
		FVector Right = CameraComponent->GetRightVector();

		// gamepad movement (left stick)
		FVector LX = PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftX) * Right;
		FVector LY = PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftY) * Forward;

		FVector MoveDirection = LX + LY;


		if (PC->IsInputKeyDown(EKeys::W)) MoveDirection += Forward;
		if (PC->IsInputKeyDown(EKeys::S)) MoveDirection -= Forward;
		if (PC->IsInputKeyDown(EKeys::D)) MoveDirection += Right;
		if (PC->IsInputKeyDown(EKeys::A)) MoveDirection -= Right;

		if (!MoveDirection.IsNearlyZero())
		{
			MoveDirection.Normalize();

			float MoveSpeed = 1.0f;
			if (PC->IsInputKeyDown(EKeys::LeftShift))
			{
				MoveSpeed *= runModifier;
			}
			GBVector3 move = {
				(float)MoveDirection.X * MoveSpeed,
				(float)MoveDirection.Y * MoveSpeed,
				(float)MoveDirection.Z * MoveSpeed
			};

			body.wake();
			body.addForce(move.xyComponent() * speed * body.mass);
		}

		// jump (A button)
		bool jumped = PC->IsInputKeyDown(EKeys::SpaceBar) || PC->WasInputKeyJustPressed(EKeys::Gamepad_FaceButton_Bottom);
		if (body.isGrounded && jumped)
		{
			body.velocity.z = jumpSpeed;
		}
	}
}

void APGBWorld::followCamera(UWorld* world, GBBody& body)
{
	if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{
		// --- MOUSE LOOK ---
		float MouseX, MouseY;
		PC->GetInputMouseDelta(MouseX, MouseY);

		float MouseSensitivity = 1.0f; // tweak to your liking
		Yaw += MouseX * MouseSensitivity;
		Pitch += MouseY * MouseSensitivity;
		float ThumbstickSensitivity = 2.0f;
		Yaw += PC->GetInputAnalogKeyState(EKeys::Gamepad_RightX) * ThumbstickSensitivity;
		Pitch -= PC->GetInputAnalogKeyState(EKeys::Gamepad_RightY) * ThumbstickSensitivity;

		// Clamp pitch to avoid flipping
		Pitch = FMath::Clamp(Pitch, -89.f, 89.f);

		// Apply rotation
		FRotator NewRotation(Pitch, Yaw, 0.f);
		CameraComponent->SetWorldRotation(NewRotation);
	}

	FVector BodyPos = toFVector(body.transform.position);

	// Clamp pitch to avoid flipping
	Pitch = FMath::Clamp(Pitch, -80.0f, 80.0f);

	FRotator CamRot(Pitch, Yaw, 0.0f);

	FVector Offset = CamRot.Vector() * 500.0f;
	FVector CamPos = BodyPos - Offset;

	CameraComponent->SetWorldLocation(CamPos);
	CameraComponent->SetWorldRotation((BodyPos - CamPos).Rotation());
}

GBBody* APGBWorld::getBodyByName(UWorld* world, std::string name)
{
	for (auto& it : simulation.rigidBodies)
	{
		GBBody* pBody = it.get();
		if (pBody->pData)
		{
			BodyTag* tag = (BodyTag*)pBody->pData;
			if (tag && tag->tag == name)
			{
				return pBody;
			}
		}
	}
	return nullptr;
}

std::vector<GBBody*> APGBWorld::getBodiesByTag(UWorld* world, std::string tag)
{
	std::vector<GBBody*> taggedBodies;
	for (auto& it : simulation.rigidBodies)
	{
		GBBody* pBody = it.get();
		if (pBody->pData)
		{
			BodyTag* bt = (BodyTag*)pBody->pData;
			if (bt && bt->tag == tag)
			{
				taggedBodies.push_back(pBody);
			}
		}
	}
	return taggedBodies;
}

void APGBWorld::deleteActorAndBody(GBBody* pBody)
{
	if (!pBody)
		return;

	for (GBCollider* col : pBody->colliders)
	{
		if (!col)
			continue;

		const uint32 id = col->id;

		switch (col->type)
		{
		case ColliderType::Box:
		{
			if (AActor* Actor = BoxActors.FindRef(id))
			{
				if (IsValid(Actor))
				{
					Actor->Destroy();
				}
			}
			BoxActors.Remove(id);

			if (UStaticMeshComponent* Mesh = BoxMeshes.FindRef(id))
			{
				if (IsValid(Mesh))
				{
					Mesh->DestroyComponent();
				}
			}
			BoxMeshes.Remove(id);

			break;
		}

		case ColliderType::Sphere:
		{
			if (AActor* Actor = SphereActors.FindRef(id))
			{
				if (IsValid(Actor))
				{
					Actor->Destroy();
				}
			}
			SphereActors.Remove(id);

			if (UStaticMeshComponent* Mesh = SphereMeshes.FindRef(id))
			{
				if (IsValid(Mesh))
				{
					Mesh->DestroyComponent();
				}
			}
			SphereMeshes.Remove(id);

			break;
		}

		case ColliderType::Capsule:
		{
			if (AActor* Actor = CapsuleActors.FindRef(id))
			{
				if (IsValid(Actor))
				{
					Actor->Destroy();
				}
			}
			CapsuleActors.Remove(id);

			if (UStaticMeshComponent* Mesh = CapsuleMeshes.FindRef(id))
			{
				if (IsValid(Mesh))
				{
					Mesh->DestroyComponent();
				}
			}
			CapsuleMeshes.Remove(id);

			break;
		}
		}
	}
}


APGBWorld::~APGBWorld()
{

}

void APGBWorld::setCameraTransform(const GBVector3& position, const GBQuaternion& rotation)
{
	if (!CameraComponent) return;

	CameraComponent->SetWorldLocation(toFVector(position));
	CameraComponent->SetWorldRotation(toFQuat(rotation));
}

void APGBWorld::setCameraPosition(const GBVector3& position)
{
	if (!CameraComponent) return;

	FVector UnrealPos = toFVector(position);
	CameraComponent->SetWorldLocation(UnrealPos);
}

bool APGBWorld::GetHeight(int32 X, int32 Y, float& outHeight)
{
	outHeight = FMath::PerlinNoise2D(FVector2D(X * 0.1f, Y * 0.1f));
	//outHeight = 0.0f;
	//GBVector3 midpoint = { 25.0f * 0.25f, 25.0f * 0.25f, 0 };
	//GBVector3 p = { X * 0.25f ,Y * 0.25f, 0 };
	//if((p-midpoint).length() > midpoint.x)
	//	return false;
	return true;
}

bool APGBWorld::GetEllipseHeight(int32 X, int32 Y, float& outHeight)
{
	GBVector3 midpoint = { 25.0f * 0.25f, 25.0f * 0.25f, 0 };
	GBVector3 p = { X * 0.25f ,Y * 0.25f, 0 };
	GBVector3 d = p - midpoint;

	float rx = 10.0f;
	float ry = 5.0f;

	float v = (d.x * d.x) / (rx * rx) +
		(d.y * d.y) / (ry * ry);
	outHeight = 0.0f;
	if (v > 1.0f)
		return false;
	return true;
}

std::vector<std::vector<GBTriangle>> APGBWorld::createHeightmapTerrainTriangles(HeightFunc GetHeightFunc,
	const int sizeX, const int sizeY, float spacing, GBVector3 originOffset)
{
	const int32 SizeX = sizeX;
	const int32 SizeY = sizeY;
	float xStart = -SizeX * spacing * 0.5f;
	float yStart = -SizeY * spacing * 0.5f;
	std::vector<std::vector<GBTriangle>> triangles;
	bool set[4];

	for (int32 x = 0; x < SizeX; x++)
	{
		triangles.push_back(std::vector<GBTriangle>());
		for (int32 y = 0; y < SizeY; y++)
		{
			GBVector3 a, b, c, d;
			float height;
			memset(set, false, 4 * sizeof(bool));

			if (GetHeightFunc(x, y, height))
			{
				set[0] = true;
				a = { x * spacing + xStart, y * spacing + yStart, height };
				a += originOffset;
			}
			if (GetHeightFunc(x + 1, y, height))
			{
				set[1] = true;
				b = { (x + 1) * spacing + xStart, y * spacing + yStart, height };
				b += originOffset;
			}
			if (GetHeightFunc(x, y + 1, height))
			{
				set[2] = true;
				c = { x * spacing + xStart, (y + 1) * spacing + yStart, height };
				c += originOffset;
			}
			if (GetHeightFunc(x + 1, y + 1, height))
			{
				set[3] = true;
				d = { (x + 1) * spacing + xStart, (y + 1) * spacing + yStart, height };
				d += originOffset;
			}
			if (set[0] && set[1] && set[2])
				triangles.back().push_back(GBTriangle(a, b, c));

			if (set[1] && set[3] && set[2])
				triangles.back().push_back(GBTriangle(b, d, c));
		}
	}

	return triangles;
}

void APGBWorld::AddTriangle(const FVector& A, const FVector& B, const FVector& C, bool updateMesh)
{
	int32 IndexOffset = Vertices.Num();

	// Front face
	Vertices.Add(A);
	Vertices.Add(B);
	Vertices.Add(C);

	Triangles.Add(IndexOffset);
	Triangles.Add(IndexOffset + 1);
	Triangles.Add(IndexOffset + 2);

	// Back face (reversed winding)
	Vertices.Add(A);
	Vertices.Add(C);
	Vertices.Add(B);

	Triangles.Add(IndexOffset + 3);
	Triangles.Add(IndexOffset + 4);
	Triangles.Add(IndexOffset + 5);

	if (updateMesh)
		UpdateMesh();
}

void APGBWorld::UpdateMesh()
{
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<FProcMeshTangent> Tangents;
	TArray<FLinearColor> VertexColors;

	Normals.SetNum(Vertices.Num());
	UVs.SetNum(Vertices.Num());
	for (int32 i = 0; i < UVs.Num(); i++)
	{
		UVs[i] = FVector2D(0, 0); // dummy UVs
	}

	// Calculate normals for front and back faces
	for (int32 i = 0; i < Triangles.Num(); i += 3)
	{
		FVector Edge1 = Vertices[Triangles[i + 1]] - Vertices[Triangles[i]];
		FVector Edge2 = Vertices[Triangles[i + 2]] - Vertices[Triangles[i]];
		FVector Normal = FVector::CrossProduct(Edge1, Edge2).GetSafeNormal();

		// Detect backface: assume every second triangle is backface
		if ((i / 3) % 2 == 1)
			Normal = -Normal;

		Normals[Triangles[i]] = Normal;
		Normals[Triangles[i + 1]] = Normal;
		Normals[Triangles[i + 2]] = Normal;
	}

	MeshComponent->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UVs, VertexColors, Tangents, true);
}

void APGBWorld::updateTerrainMeshActor(const std::vector<std::vector<GBTriangle>>& triangles)
{
	for (int i = 0; i < triangles.size(); i++)
	{
		for (int j = 0; j < triangles[i].size(); j++)
		{
			FVector A = toFVector(triangles[i][j].vertices[0]);
			FVector B = toFVector(triangles[i][j].vertices[1]);
			FVector C = toFVector(triangles[i][j].vertices[2]);

			AddTriangle(A, B, C, false);
		}
	}

	UpdateMesh();
}

void APGBWorld::toggleDelayStart()
{
	if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{

		if (PC->WasInputKeyJustPressed(EKeys::Enter)) {
			doDelayStart = !doDelayStart;
		}
	}
}