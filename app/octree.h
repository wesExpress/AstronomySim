#ifndef __OCTREE_H__
#define __OCTREE_H__

#include "dm.h"

#define OCTREE_MAX_DEPTH      7
#define OCTREE_CHILDREN_COUNT 8

#define OCTREE_MAX_NODE_COUNT 500000
#define OCTREE_INVALID_CHILD  (OCTREE_MAX_NODE_COUNT-1)
#define OCTREE_NO_OBJECT      UINT_MAX

typedef struct bh_node_t
{
    dm_vec3 center_of_mass;
    float   total_mass;
    
    int32_t first_child;
    int32_t object_index;
} bh_node;

typedef struct bh_tree_t
{
    uint32_t node_count, max_depth;
    
    float com_x[OCTREE_MAX_NODE_COUNT];
    float com_y[OCTREE_MAX_NODE_COUNT];
    float com_z[OCTREE_MAX_NODE_COUNT];
    float total_mass[OCTREE_MAX_NODE_COUNT];
    
    uint32_t parent[OCTREE_MAX_NODE_COUNT];
    uint32_t first_child[OCTREE_MAX_NODE_COUNT];
    uint32_t object_index[OCTREE_MAX_NODE_COUNT];
} bh_tree;

void bh_tree_init(bh_tree* tree);
void bh_tree_insert(const uint32_t object_index, const uint32_t node_index, float cen_x, float cen_y, float cen_z, float half_extents, const uint32_t depth, bh_tree* tree, const float* pos_x, const float* pos_y, const float* pos_z, const float* mass);
void bh_tree_render(const float cen_x, const float cen_y, const float cen_z, const float half_extents, const bh_tree* tree, void** debug_render_data, dm_context* context);

#endif
