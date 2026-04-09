- [[#**Overview**|**Overview**]]
- [[#**Core Idea**|**Core Idea**]]
		- [[#Purpose & Shape of Messages|Purpose & Shape of Messages]]
		- [[#Life Cycle Example: From Q to R to D|Life Cycle Example: From Q to R to D]]
- [[#**Global Order by ID**|**Global Order by ID**]]
- [[#**Three-Thread Pipeline**|**Three-Thread Pipeline**]]
		- [[#Example: Why Separation Matters|Example: Why Separation Matters]]
		- [[#1.  [[Vocabulary & Primitives of the infrastructure]]|
		- [[#2.  [[Tree Node]]|
		- [[#3.  [[Processor.py]]|
		- [[#4.  [[Queues.py]]|
		- [[#5.  [[Agent_Hub_Robot.py]]
		- [[#6.  [[Robot_node.py]]|
		- [[#7.  [[Master Project Readme]]|
		- [[#7.  [[High level documentation]]|

## **Overview**

The Communication infrastructure is a **distributed message-passing infrastructure** built on top of **ROS 2**, designed to connect multiple computing units called **Tree Nodes**  into a **hierarchical tree topology** (`root ↔ children ↔ agents`).  
The architecture ensures that every **data message (D)**, regardless of where it originates, is **delivered in strict global order** across the entire tree.  
This guarantee is achieved through a **three-threaded internal pipeline** that separates control tasks, routing, and data delivery into well-defined concurrent flows.

Every node can act as:

- **Root:** grants message ids and starts new conversations (Q → R → D flow).
    
- **Intermediate node:** relays and enforces global order for all data.
    
- **Leaf node:** hosts **local agents** that produce or consume messages.


Agents interact with their local Tree Node using ROS 2 services and topics, they never communicate directly with other nodes.  
This design isolates local behavior from the rest of the tree, while maintaining globally consistent ordering and delivery.

The system’s goal is to preserve global order for all messages propagated through the infrastructure. That is achived with multiple steps at the infrastructure level.

## **Core Idea**

All communication happens through **three logical kinds of messages:**

| Kind  | Meaning                        | Flow                           | Handled by                |
| ----- | ------------------------------ | ------------------------------ | ------------------------- |
| **Q** | _Question / Request for an id_ | upward (agent → root)          | `_handle_get_msg_id_work` |
| **R** | _Reply with granted id_        | downward (root → agent)        | `_pipeline_handle_reply`  |
| **D** | _Data payload with that id_    | both directions (strict order) | `_pipeline_handle_data`   |
#### Purpose & Shape of Messages
Every message type has a purpose and a canonical shape.  
It all starts from an **agent** (`AgentX`) connected to a **Tree Node** (`Nx`) inside the infrastructure

| Type | Shape                                                        |
| ---- | ------------------------------------------------------------ |
| Q    | {Q: True, Route (Agent1 , N5), dest (N3)}                    |
| R    | {R: True, id = xx,  Route (Agent1), dest (N5)}               |
| D    | {D : True, id = xx, src (Agent1), dest (N5), "hello world" } |
#### Life Cycle Example: From Q to R to D

-  **Agent Requests an ID (Q)**

	An agent cannot send a data message without a valid **ID**.  
	So it first sends a **Q message** upward:

	`{ "Q": true, "route": ["Agent1"], "dest": "rootA" }`

	Each intermediate node appends its own name to the `route`:

	`{ "Q": true, "route": ["Agent1", "NodeB"], "dest": "rootA" }`

	When the **root** receives it, the full path is visible:  
	`["Agent1","NodeB"]`.

-  **Root Grants the ID (R)**

	The root allocates a new ID = 42 and creates a **reply (R)** message:
	
	`{ "R": true, "id": 42, "route": ["Agent1"], "dest": "NodeB" }`
	
	Each hop downward removes itself from the `route` and rewrites the `dest`  
	until the message reaches the agent:
	
	`{ "R": true, "id": 42, "route": [], "dest": "Agent1" }`
	
	The agent now owns **ID 42** and can publish a corresponding **data message (D)**.

 - **Agent Publishes Data (D)**

	The agent sends:
	
	`{ "D": true, "id": 42, "src": "Agent1", "dest": "rootA", "msg": { "payload": "hello world" } }`
	
	When `NodeB` (the agent’s parent) receives it, it detects  
	`src="Agent1"` → came from below, so it:
	
	1. Delivers locally to other agents.
	    
	2. Broadcasts down to its children (if any).
	    
	3. Forwards up to its parent (`rootA`).
	    
	
	Before forwarding up, it rewrites:
	
	`{ "D": true, "id": 42, "src": "NodeB", "dest": "rootA", "msg": { "payload": "hello world" } }`
	
	At the root, the same logic applies:  
	deliver locally, broadcast down to all children, and rewrite `src` again to `"rootA"`.
	
	Rather than tagging messages with static “up” or “down” labels, each node **infers the direction dynamically**:

	- If `src` ∈ children or local agents → message came **from below**.
	    
	- If `src` == parent → message came **from above**.
    

	This inference keeps routing context always correct and eliminates stale directional metadata.

## **Global Order by ID**

Every **data message (D)** carries a unique integer `id`.  
These IDs are **issued exclusively by the root** during the **Q → R** exchange.  
All nodes use the ID to enforce **strict sequencing**:

- Each node maintains a counter called **next expected id**.
    
- When a new **D** arrives:
    
    - If `D.id == next_expected_id` → deliver and forward.
        
    - If `D.id > next_expected_id` → hold it in a priority queue until missing IDs arrive.
        
- After successful delivery, increment `next_expected_id += 1`.
    

**Example**

|Event|NodeB.expected_id|Incoming D.id|Action|
|---|---|---|---|
|Start|1|—|waiting|
|D(id = 1) arrives|1|1|deliver → expected = 2|
|D(id = 3) arrives|2|3|queued (wait for 2)|
|D(id = 2) arrives|2|2|deliver 2 then 3 → expected = 4|

## **Three-Thread Pipeline**

The data messages types Q/R/D are managed by threads operating inside each node. 
Each node has 3 threads that deals with the different messages type a node receives
Check this table 

| Thread              | Role                                                                                                          |
| ------------------- | ------------------------------------------------------------------------------------------------------------- |
| **T1 – Collector**  | Gathers control work and incoming messages from ROS 2 callbacks into one FIFO queue.                          |
| **T2 – Classifier** | Extracts Data messages from the FIFO and inserts them into a PriorityQueue ordered by ID.                     |
| **T3 – Arbiter**    | Delivers Data messages in order (when `id == expected_id`) and processes other work (Q/R) from the FIFO head. |
#### Example: Why Separation Matters

Imagine these two events happening at once:

|Time|Incoming message|Type|ID|What must happen|
|---|---|---|---|---|
|t₀|DataMsg|D|12|should wait until IDs 1–11 delivered|
|t₁|GetMsgId request|Q|–|should be answered immediately|
Without separate threads, the node could get stuck behind the queued D(12) waiting for IDs 1–11, delaying unrelated control requests.  
With T1/T2/T3:

- T1 collects both in FIFO.
    
- T2 moves D(12) into PQ and leaves Q in FIFO.
    
- T3 sees D(12) can’t deliver yet, so it processes the Q right away.
    

Result → **no blocking, deterministic order preserved**

# How to run
For detailed instructions about how to run the project please follow this [[How to run]]


# Understanding the code


For deeper understanding for each component used in this project please read and navigate in the following order 

#### 1.  [[Vocabulary & Primitives of the infrastructure]]

#### 2.  [[Tree Node]]
#### 3.  [[Processor.py]]
#### 4.  [[Queues.py]]
#### 5.  [[Agent_Hub_Robot.py]]
#### 6.  [[Robot_node.py]]
#### 7.  [[Master Project Readme]]
#### 7.  [[High level documentation]]







