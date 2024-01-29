#ifndef __OCTREE_H__
#define __OCTREE_H__

#include "dm.h"

#ifdef  OCTREE_LIMIT_DEPTH
#define OCTREE_MAX_DEPTH        6
#endif

typedef struct octree_node_t
{
    dm_vec3   center;
    float     half_extents;
    
    uint16_t  obj_count;
    int32_t   first_child;
    
    uint32_t* obj_index_array;
} octree_node;

bool octree_insert(const uint32_t object_index, const uint32_t node_index, const uint32_t depth, uint32_t* max_depth, octree_node** octree, uint32_t* node_count, const float* pos_x, const float* pos_y, const float* pos_z);
void octree_render(const uint32_t node_index, const uint32_t depth, octree_node* octree, void** render_data, dm_context* context);
void octree_node_destroy(const uint32_t node_index, octree_node** octree);

#endif
