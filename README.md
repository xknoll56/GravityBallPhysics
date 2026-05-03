Physics Engine Overview



This engine uses a sequential impulse–style solver with contact-based constraint resolution, rather than a fully global constraint system. Rigid body interactions are resolved through local pairwise collision responses, which behave like implicit constraints when iterated over multiple solver passes.



Contacts are accumulated per frame and processed using an iterative impulse solver, where corrections are applied sequentially across contact points. Stability in stacks and complex interaction scenarios is achieved through repeated relaxation steps, warm-started impulses, and careful friction handling.



To improve stability and performance, the engine organizes bodies into dynamic interaction islands. Islands are formed by tracking contact connectivity, allowing independent clusters of interacting objects to be solved separately. When islands include static geometry, constraints are effectively propagated through the cluster, producing stable resting behavior and realistic stacking.



Rather than explicitly constructing and solving a global constraint matrix, the system relies on:



Local collision response acting as constraints

Iterative sequential solving

Island-based decomposition

Contact persistence and propagation



This results in a deterministic and stable simulation where small local interactions propagate naturally through contact networks, producing emergent global behavior such as stacking, resting contacts, and force transmission.

