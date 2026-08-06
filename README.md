## Physics Engine Overview

<p align="center">
  <img src="https://github.com/user-attachments/assets/6f796b78-c998-458c-bd7e-05c1f626edeb"
       alt="GravityBall Physics Logo"
       width="400">
</p>

- A modern, header-only C++ 3D rigid body physics engine built for games.

🚀 Header-only design <br>
⚡ Real-time sequential impulse solver <br>
📦 Stable box stacking and resting contacts <br>
🎯 Deterministic simulation <br>
🌍 Uniform-grid broad phase <br>
🎮 Integrated into Unreal Engine (rendering only) <br>

# Features
### Physics
Sequential impulse constraint solver <br>
Persistent contact manifolds <br>
Warm starting <br>
Restitution and Coulomb friction <br>
Sleeping / wake propagation <br>
Continuous grounded contact handling <br>
Position correction with penetration slop <br>
Position correction with penetration slop <br>
Stack stabilization <br>
### Collision
Sphere <br>
Capsule <br>
Oriented Box (OBB) <br>
Quad <br>
Triangle <br>
### Queries
Raycasting <br>
Closest hit <br>
Surface normals <br>
Distance queries <br>
### Broad Phase
Uniform spatial grid <br>
Deterministic traversal <br>
Fast nearby collider extraction <br>

```cpp
GBSimulation simulation;

GBBody* pBody = simulation.createBody();
GBSphereCollider* pSphere = simulation.attachSphereCollider(pBody, 0.5f);

while (running)
{
    simulation.Step(dt);
}
```
