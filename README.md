Physics Engine Overview



The engine is a custom real-time rigid body physics system built around an iterative sequential impulse solver with stack-aware contact stabilization and recursive contact graph propagation.



Rather than solving a fully global constraint system, the engine resolves rigid body interactions locally through contact manifolds and pairwise impulse constraints. These local constraints are iteratively relaxed across multiple solver passes, allowing stable emergent behavior such as stacking, resting contacts, frictional support, and force propagation through connected bodies.



The solver combines velocity-based impulse resolution with positional penetration correction, using a hybrid approach optimized for determinism, stability, and gameplay-oriented behavior.



Sequential Impulse Solver



Rigid body collisions are resolved using an iterative sequential impulse method.



For each contact manifold:



Relative velocity is computed at each contact point

Normal impulses are applied to prevent interpenetration

Angular response is generated through rotational inertia

Friction impulses are solved along the contact tangent



Contacts are processed iteratively across multiple solver passes, allowing constraints to converge over time rather than being solved globally in a single step.



This approach produces stable and scalable rigid body interaction while remaining efficient for real-time simulation.



The solver supports:



Linear and angular impulse response

Effective mass calculation

Restitution and friction

Multi-contact manifold resolution

Dynamic and kinematic body interaction

Resting contact stabilization

Hybrid Static / Dynamic Contact Resolution



The engine distinguishes between:



Dynamic body interactions

Resting or support-like contacts

Static environment collisions



When relative motion between contacting bodies falls below configurable thresholds, contacts may transition into a resting-contact style solve path optimized for stability.



This hybrid approach improves:



Object stacking

Grounded stability

Slope handling

Resting contact jitter

Character-like movement behavior



Static and support-oriented contacts are treated differently from high-energy collisions, allowing the engine to maintain stable stacks without requiring a fully global constraint solve.



Contact Graph Propagation



Instead of partitioning bodies into isolated solver islands, the engine maintains a dynamic contact connectivity graph between interacting bodies.



During collision detection:



Bodies sharing contacts become linked

Contact relationships form a directed interaction graph

Static bodies naturally act as anchors within the graph



During penetration resolution, positional corrections propagate recursively through connected bodies.



When one body is corrected due to penetration or support constraints, connected bodies may inherit portions of that correction through existing contact relationships. This recursive propagation stabilizes stacks and clustered interactions by preserving relative support relationships between connected objects.



The propagation system behaves similarly to shock propagation and support-directed constraint solving commonly used in real-time rigid body engines.



Key properties include:



Recursive support propagation

Stack-aware correction ordering

Contact graph traversal

Support-direction filtering

Limited-depth recursive stabilization

Deterministic correction flow



This produces coherent stack behavior without explicitly constructing or solving a global constraint matrix.



Positional Stabilization



Penetration correction is handled through direct positional projection combined with iterative velocity constraint solving.



The engine applies:



Penetration slop thresholds

Maximum correction clamping

Recursive propagation through support chains

Contact-aware damping for stacked bodies



These stabilization systems reduce:



Jitter

Constraint oscillation

Stack collapse

Penetration drift

Energy amplification



The solver intentionally prioritizes stable real-time behavior and gameplay feel over strict physical exactness.



Persistent Contact Handling



Collision manifolds persist across solver iterations and frames.



Per-frame manifold accumulation allows the solver to:



Preserve grounded state

Maintain stable resting contacts

Improve friction consistency

Reduce contact instability across frames



Bodies maintain dynamic contact connectivity information used for:



Sleep/wake propagation

Recursive correction traversal

Contact graph stabilization

Grounding detection

Sleeping and Island Activation



The engine includes a dynamic sleeping system to reduce unnecessary simulation work.



Bodies transition into sleep states when both:



Linear velocity

Angular velocity



remain below configurable thresholds for a sustained period.



Connected bodies can recursively wake each other through contact relationships, allowing dynamic interaction clusters to react coherently when disturbed.



This preserves simulation stability while improving runtime performance in large scenes.



Collision Shapes



The engine currently supports several analytic primitive collision shapes optimized for deterministic rigid body interaction.



Supported shapes include:



Spheres

Oriented boxes

Capsules



Collision detection uses analytic narrow-phase tests and manifold generation rather than mesh-based collision solving.



This provides:



Stable contact generation

Predictable manifold behavior

Efficient collision detection

Deterministic solver input

Raycasting



The engine provides a built-in raycasting system for spatial queries and gameplay interaction.



Raycasts support:



Sphere intersection

Box intersection

Closest hit queries

Surface normal extraction

Distance reporting



Raycasts operate directly on the same collision primitives used by the physics solver, ensuring consistency between gameplay queries and physical simulation.



Broad-Phase Collision Detection



Broad-phase collision detection is accelerated using a uniform spatial grid (spatial hash).



The world is partitioned into fixed-size grid cells, and bodies are inserted according to their spatial bounds.



Collision tests are restricted to nearby occupied cells, significantly reducing pairwise collision checks while maintaining deterministic behavior and predictable performance scaling.



The broad phase supports:



Dynamic body insertion and movement

Efficient nearby collider extraction

Stable deterministic traversal

Real-time update performance

Simulation Philosophy



The engine is designed around practical real-time rigid body behavior rather than strict physically exact simulation.



The architecture prioritizes:



Stable stacking

Deterministic behavior

Responsive gameplay interaction

Robust resting contacts

Efficient real-time performance

Coherent force propagation through connected bodies



Rather than relying on a single mathematically global solve, the engine achieves complex emergent behavior through iterative local constraints, recursive contact propagation, and layered stabilization systems.

