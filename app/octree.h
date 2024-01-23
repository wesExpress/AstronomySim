#ifndef __OCTREE_H__
#define __OCTREE_H__

#include <assert.h>

#define OCTREE_MAX_DEPTH      8
#define OCTREE_CHILDREN_COUNT 8

typedef struct octree_node_t
{
    dm_vec3 center;
    float   half_extents;
    
    int32_t first_child;
    int32_t obj_index;
} octree_node;

DM_INLINE
void octree_node_init_children(uint32_t node_index, octree_node** octree, uint32_t* octree_node_count)
{
    const uint32_t old_count = *octree_node_count;
    const uint32_t new_count = old_count + OCTREE_CHILDREN_COUNT;
    *octree = dm_realloc(*octree, sizeof(octree_node) * new_count);
    
    octree_node* node = &((*octree)[node_index]);
    
    node->first_child  = old_count;
    *octree_node_count = new_count;
    
    // set children's indices to -1
    // set children's center and extents
    octree_node* child = NULL;
    const float new_half_extents = node->half_extents * 0.5f;
    
    child = &((*octree)[node->first_child]);
    child->first_child  = -1;
    child->obj_index    = -1;
    child->half_extents = new_half_extents;
    child->center[0] = node->center[0] - new_half_extents;
    child->center[1] = node->center[1] - new_half_extents;
    child->center[2] = node->center[2] - new_half_extents;
    
    child = &((*octree)[node->first_child + 1]);
    child->first_child = -1;
    child->obj_index   = -1;
    child->half_extents = new_half_extents;
    child->center[0] = node->center[0] + new_half_extents;
    child->center[1] = node->center[1] - new_half_extents;
    child->center[2] = node->center[2] - new_half_extents;
    
    child = &((*octree)[node->first_child + 2]);
    child->first_child = -1;
    child->obj_index   = -1;
    child->half_extents = new_half_extents;
    child->center[0] = node->center[0] - new_half_extents;
    child->center[1] = node->center[1] + new_half_extents;
    child->center[2] = node->center[2] - new_half_extents;
    
    child = &((*octree)[node->first_child + 3]);
    child->first_child = -1;
    child->obj_index   = -1;
    child->half_extents = new_half_extents;
    child->center[0] = node->center[0] + new_half_extents;
    child->center[1] = node->center[1] + new_half_extents;
    child->center[2] = node->center[2] - new_half_extents;
    
    child = &((*octree)[node->first_child + 4]);
    child->first_child = -1;
    child->obj_index   = -1;
    child->half_extents = new_half_extents;
    child->center[0] = node->center[0] - new_half_extents;
    child->center[1] = node->center[1] - new_half_extents;
    child->center[2] = node->center[2] + new_half_extents;
    
    child = &((*octree)[node->first_child + 5]);
    child->first_child = -1;
    child->obj_index   = -1;
    child->half_extents = new_half_extents;
    child->center[0] = node->center[0] + new_half_extents;
    child->center[1] = node->center[1] - new_half_extents;
    child->center[2] = node->center[2] + new_half_extents;
    
    child = &((*octree)[node->first_child + 6]);
    child->first_child = -1;
    child->obj_index   = -1;
    child->half_extents = new_half_extents;
    child->center[0] = node->center[0] - new_half_extents;
    child->center[1] = node->center[1] + new_half_extents;
    child->center[2] = node->center[2] + new_half_extents;
    
    child = &((*octree)[node->first_child + 7]);
    child->first_child = -1;
    child->obj_index   = -1;
    child->half_extents = new_half_extents;
    child->center[0] = node->center[0] + new_half_extents;
    child->center[1] = node->center[1] + new_half_extents;
    child->center[2] = node->center[2] + new_half_extents;
}

DM_INLINE
bool octree_node_contains_object(const dm_vec3 pos, octree_node* node)
{
    dm_vec3 min, max;
    
    dm_vec3_sub_scalar(node->center, node->half_extents, min);
    dm_vec3_add_scalar(node->center, node->half_extents, max);
    
    const bool in_x = (pos[0] >= min[0]) && (pos[0] < max[0]);
    const bool in_y = (pos[1] >= min[1]) && (pos[1] < max[1]);
    const bool in_z = (pos[2] >= min[2]) && (pos[2] < max[2]);
    
    return in_x && in_y && in_z;
}

void octree_insert(const uint32_t object_index, const uint32_t node_index, uint16_t depth, uint16_t* max_depth, octree_node** octree, uint32_t* octree_node_count, const float* pos_x, const float* pos_y, const float* pos_z)
{
    assert(octree);
    
    *max_depth = depth > *max_depth ? depth : *max_depth;
    
    octree_node* node = &((*octree)[node_index]);
    
    const dm_vec3 new_object_pos = {
        pos_x[object_index],
        pos_y[object_index],
        pos_z[object_index],
    };
    
    if(!octree_node_contains_object(new_object_pos, node)) return;
    
    // do we have children?
    if(node->first_child != -1)
    {
        // find child to insert into
        octree_node* child = NULL;
        octree_node children[OCTREE_CHILDREN_COUNT];
        dm_memcpy(children, *octree + node->first_child, sizeof(octree_node) * OCTREE_CHILDREN_COUNT);
        
        for(uint32_t i=0; i<OCTREE_CHILDREN_COUNT; i++)
        {
            child = &(*octree)[node->first_child + i];
            
            if(!octree_node_contains_object(new_object_pos, child)) continue;
            
            octree_insert(object_index, node->first_child + i, depth + 1, max_depth, octree, octree_node_count, pos_x, pos_y, pos_z);
            
            return;
        }
        
        DM_LOG_FATAL("Octree insertion failed somehow");
        assert(false);
    }
    
    // are we empty?
    if(node->obj_index == -1)
    {
        node->obj_index = object_index;
        
        return;
    }
    
    // init children
    assert(node->first_child==-1);
    octree_node_init_children(node_index, octree, octree_node_count);
    node = &((*octree)[node_index]);
    assert(node->first_child!=-1);
    
    // reinsert old index
    const dm_vec3 old_object_pos = {
        pos_x[node->obj_index],
        pos_y[node->obj_index],
        pos_z[node->obj_index],
    };
    
    octree_node* child = NULL;
    for(uint32_t i=0; i<OCTREE_CHILDREN_COUNT; i++)
    {
        child = &(*octree)[node->first_child + i];
        
        if(!octree_node_contains_object(old_object_pos, child)) continue;
        
        octree_insert(node->obj_index, node->first_child + i, depth + 1, max_depth, octree, octree_node_count, pos_x, pos_y, pos_z);
        
        break;
    }
    
    // insert new index
    child = NULL;
    for(uint32_t i=0; i<OCTREE_CHILDREN_COUNT; i++)
    {
        child = &(*octree)[node->first_child + i];
        
        if(!octree_node_contains_object(new_object_pos, child)) continue;
        
        octree_insert(object_index, node->first_child + i, depth + 1, max_depth, octree, octree_node_count, pos_x, pos_y, pos_z);
        
        break;
    }
    
    node->obj_index = -1;
}


#endif
