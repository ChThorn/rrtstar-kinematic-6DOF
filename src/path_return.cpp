#include "path_return.h"
#include "tree_management.h"  // For Node definition
#include <vector>
#include <memory>

std::vector<std::shared_ptr<Node>> createReturnPathSimple(
    const std::vector<std::shared_ptr<Node>>& forward_path) {
    
    // Create a reversed copy of the forward path
    std::vector<std::shared_ptr<Node>> return_path;
    
    // Simply add nodes in reverse order
    for (auto it = forward_path.rbegin(); it != forward_path.rend(); ++it) {
        auto node = std::make_shared<Node>((*it)->q);
        
        if (!return_path.empty()) {
            node->parent = return_path.back();
        }
        
        return_path.push_back(node);
    }
    
    return return_path;
}