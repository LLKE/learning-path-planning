descriptions = {
    "A*": """
    ## A*  
    The A* algorithm is one of the most widely used and efficient algorithms for path finding and graph traversal. It was also the first path finding algorithm I heard of when entering the realm of autonomous systems. A* offers a great balance of performance and accuracy. Here’s a quick dive into why it’s so powerful:

    🔍 What is A?* 

    A* (pronounced “A-star”) is a search algorithm that finds the shortest path between a start point and a goal point. It combines Dijkstra’s algorithm (which finds the shortest path) with Greedy Best-First-Search (which quickly evaluates potential paths) by using heuristics.

    📊 How it Works: 

    A* uses a cost function f(n) = g(n) + h(n):

    g(n) is the exact cost from the start node to the current node.
    h(n) is the estimated cost from the current node to the goal, typically using a heuristic (an optimistic estimate, not considering, e.g., obstacles) like Manhattan distance or Euclidean distance. More info on those in the comments!

    By evaluating both the current cost and the potential future cost, A* ensures that it always finds the optimal path, or the best possible solution based on the available information. Practically, this means that the algorithm prioritizes exploring the path with the lowest estimated total cost to the goal. If a path becomes too costly, the algorithm shifts focus to a more promising alternative.

    Note: If h(n) = 0 for all n, the A* algorithm reduces to Dijkstra's algorithm, which I covered last week! I'll leave a link in the comments ↓

    ⚡ Why is it so Effective?

    ✅ Optimal & Complete: It guarantees finding the shortest path, provided the heuristic is admissible (never overestimates the cost).

    ✅ Efficiency: By focusing on promising paths first using the heuristic, A* reduces the number of nodes it needs to explore compared to brute-force methods.

    ✅ Flexibility: It can be adapted to different use cases with custom heuristics, making it a go-to for applications ranging from robotics to gaming AI to route planning in logistics.

    💡 Key Takeaways:

    A* strikes a balance between optimality and efficiency, making it a good choice for path finding problems.

    Its flexibility and adaptability ensure it can be customized for a variety of applications.
        
    """,

    "Theta*": """
    ## Theta*
    If you've worked with A* for path-finding, you know it's powerful—but have you ever noticed how it sticks to the grid, even when a shortcut is possible? 🤔

    That’s where Theta* comes in! 🎯

    🔍 What’s the Problem with A?*

    A* finds optimal paths, but it only moves along graph edges. Even if a direct diagonal shortcut exists, A* won’t take it unless the graph explicitly allows it. This results in unnatural, zigzag paths, especially in robotics and game AI.

    🔧 How Theta Fixes This*

    Theta* improves A* by introducing line-of-sight checks between nodes. Instead of blindly following the grid, it skips unnecessary waypoints and allows more realistic, smooth paths.

    ✅ More direct movement (no rigid grid constraints)
    ✅ Smoother paths for robots, AI, and navigation systems
    ✅ Minimal extra computation compared to A*

    🔥 Where is Theta Used?*

    Theta* is widely used in:

    🚗 Autonomous robots & drones (efficient movement in open spaces)
    🎮 Game AI path-finding (realistic movement without grid artifacts)
    📍 Navigation systems (more human-like routes)

    """,

    "Hybrid A*": """
    ## Hybrid A* 
    is an extension of the A* algorithm designed for non-holonomic vehicles, such as cars. It takes into account the vehicle's kinematic constraints, allowing it to generate feasible and smooth paths that can be followed by the vehicle.
    """
}
