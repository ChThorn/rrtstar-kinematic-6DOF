#ifndef PATH_RETURN_H
#define PATH_RETURN_H

#include <vector>
#include <memory>

// Forward declarations
struct Node;

/**
 * Creates a simple return path by reversing the forward path.
 * Each node in the return path copies the configuration from the corresponding node
 * in the forward path, and establishes parent relationships in reverse order.
 * 
 * @param forward_path The original path from start to goal
 * @return A new path from goal to start
 */
std::vector<std::shared_ptr<Node>> createReturnPathSimple(
    const std::vector<std::shared_ptr<Node>>& forward_path);

#endif // PATH_RETURN_H