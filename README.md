Physics Engine Overview



This engine uses a sequential impulse–style solver with contact-based constraint resolution, rather than a fully global constraint system. Rigid body interactions are resolved through local pairwise collision responses, which behave like implicit constraints when iterated over multiple solver passes.



Contacts are accumulated per frame and processed using an iterative solver, where corrections are applied sequentially across contact points. Stability in stacks and complex interaction scenarios is achieved through repeated relaxation steps, warm-started impulses, and careful friction handling.



Instead of performing strict island separation with independent solvers, the engine maintains a dynamic contact connectivity graph between bodies. During collision detection, interacting bodies are linked, forming clusters of connected constraints. These clusters are not solved independently; instead, position corrections are propagated through the contact graph during penetration resolution.



When a correction is applied to a body, it can recursively affect connected bodies through existing contact links. This creates a cascading propagation effect that stabilizes stacks and clustered interactions, especially when static bodies act as anchors in the system.



Rather than explicitly constructing and solving a global constraint matrix, the system relies on:



Local collision response acting as constraints

Iterative sequential solving

Implicit contact graph propagation

Contact persistence across frames



This results in a deterministic and stable simulation where local interactions naturally propagate through connected bodies, producing emergent global behavior such as stacking, resting contacts, and force transmission.



Collision Shapes



The engine currently supports a set of core primitive collision shapes designed for stability and performance:



Spheres: used for fast symmetric collision tests and stable stacking approximations

Axis-aligned / oriented Boxes: support full 3D rigid body interaction using local axis projection tests

Capsules: used for smooth character-like bodies and rounded collision responses



All shapes are implemented using analytic intersection tests, allowing for deterministic collision detection without reliance on mesh-based geometry.



Raycasting



A built-in raycasting system provides fast spatial queries across the physics world.



Raycasts support:



Sphere and box intersection tests

Closest hit point, surface normal, and distance reporting

Use in grounding checks, visibility queries, and interaction systems



Raycasts operate directly on the same collision primitives used by the solver, ensuring consistency between queries and physical simulation.



Spatial Partitioning (Uniform Grid)



Broad-phase collision detection is accelerated using a uniform spatial grid (spatial hash).



Key characteristics:



The world is divided into fixed-size grid cells

Bodies are inserted into cells based on their bounds

Collision checks are restricted to nearby cells only



This reduces pairwise collision complexity while maintaining deterministic behavior and predictable performance characteristics.



Interaction Propagation (Contact Graph)



Rather than performing a strict island-based separation with independent solvers, the engine maintains a contact connectivity graph between bodies.



During collision detection, bodies that share contacts are linked, forming a dynamic interaction graph each frame. Instead of solving each group in isolation, position correction is propagated through this graph recursively during penetration resolution.



When a correction is applied to a body (e.g., to resolve interpenetration with a static surface or another dynamic body), that positional change is propagated to connected bodies through existing contact links. This creates a cascading effect where motion and correction naturally spread through stacks and clusters of objects.



This approach produces many of the benefits of island-based solving—such as stable stacking and coherent group behavior—without explicitly partitioning or separately solving islands. Static bodies act as anchors in the graph, allowing corrections to propagate through connected dynamic bodies and stabilizing resting contact scenarios.

