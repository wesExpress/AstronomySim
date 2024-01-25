#ifndef __OCTREE_H__
#define __OCTREE_H__

#include <assert.h>

#ifdef  OCTREE_LIMIT_DEPTH
#define OCTREE_MAX_DEPTH        6
#endif

#define OCTREE_CHILDREN_COUNT   8
#define OCTREE_PRECISION_THRESH 1e-6f

typedef struct octree_node_t
{
    dm_vec3   center;
    float     half_extents;
    
    uint16_t  obj_count;
    int32_t   first_child;
    
    uint32_t* obj_index_array;
} octree_node;

DM_INLINE
bool octree_node_contains_object(const dm_vec3 pos, const uint32_t node_index, octree_node* octree)
{
    octree_node node = octree[node_index];
    
    dm_vec3 min, max;
    dm_vec3_sub_scalar(node.center, node.half_extents, min);
    dm_vec3_add_scalar(node.center, node.half_extents, max);
    
    return dm_vec3_leq_vec3(pos, max) && dm_vec3_gt_vec3(pos, min);
}

DM_INLINE
void octree_node_init_children(const uint32_t node_index, octree_node** node, octree_node** octree, uint32_t* node_count)
{
    const uint32_t old_count = *node_count;
    const uint32_t new_count = old_count + OCTREE_CHILDREN_COUNT;
    *node_count = new_count;
    
    *octree = dm_realloc(*octree, sizeof(octree_node) * new_count);
    dm_memzero(*octree + old_count, sizeof(octree_node) * OCTREE_CHILDREN_COUNT);
    
    *node = &(*octree)[node_index];
    (*node)->first_child = old_count;
    
    const float new_half_extents = (*node)->half_extents * 0.5f;
    const float offset = new_half_extents;
    
    const uint32_t first_index = (*node)->first_child;
    
    octree_node* child  = &(*octree)[first_index];
    child->first_child  = -1;
    child->center[0]    = (*node)->center[0] - offset;
    child->center[1]    = (*node)->center[1] - offset;
    child->center[2]    = (*node)->center[2] - offset;
    child->half_extents = new_half_extents;
    
    child++;
    child->first_child  = -1;
    child->obj_count    = 0;
    child->center[0]    = (*node)->center[0] + offset;
    child->center[1]    = (*node)->center[1] - offset;
    child->center[2]    = (*node)->center[2] - offset;
    child->half_extents = new_half_extents;
    
    child++;
    child->first_child  = -1;
    child->obj_count    = 0;
    child->center[0]    = (*node)->center[0] - offset;
    child->center[1]    = (*node)->center[1] + offset;
    child->center[2]    = (*node)->center[2] - offset;
    child->half_extents = new_half_extents;
    
    child++;
    child->first_child  = -1;
    child->obj_count    = 0;
    child->center[0]    = (*node)->center[0] + offset;
    child->center[1]    = (*node)->center[1] + offset;
    child->center[2]    = (*node)->center[2] - offset;
    child->half_extents = new_half_extents;
    
    child++;
    child->first_child  = -1;
    child->obj_count    = 0;
    child->center[0]    = (*node)->center[0] - offset;
    child->center[1]    = (*node)->center[1] - offset;
    child->center[2]    = (*node)->center[2] + offset;
    child->half_extents = new_half_extents;
    
    child++;
    child->first_child  = -1;
    child->obj_count    = 0;
    child->center[0]    = (*node)->center[0] + offset;
    child->center[1]    = (*node)->center[1] - offset;
    child->center[2]    = (*node)->center[2] + offset;
    child->half_extents = new_half_extents;
    
    child++;
    child->first_child  = -1;
    child->obj_count    = 0;
    child->center[0]    = (*node)->center[0] - offset;
    child->center[1]    = (*node)->center[1] + offset;
    child->center[2]    = (*node)->center[2] + offset;
    child->half_extents = new_half_extents;
    
    child++;
    child->first_child  = -1;
    child->obj_count    = 0;
    child->center[0]    = (*node)->center[0] + offset;
    child->center[1]    = (*node)->center[1] + offset;
    child->center[2]    = (*node)->center[2] + offset;
    child->half_extents = new_half_extents;
}

bool octree_insert(const uint32_t object_index, const uint32_t node_index, const uint32_t depth, uint32_t* max_depth, octree_node** octree, uint32_t* node_count, const float* pos_x, const float* pos_y, const float* pos_z)
{
    assert(octree);
    
    const dm_vec3 obj_pos = {
        pos_x[object_index],
        pos_y[object_index],
        pos_z[object_index]
    };
    
    // early out
    if(!octree_node_contains_object(obj_pos, node_index, *octree)) return false;
    
    octree_node* node = &(*octree)[node_index];
    
#ifdef OCTREE_LIMIT_DEPTH
    // are we at max depth?
    if(depth >= OCTREE_MAX_DEPTH)
    {
        if(!node->obj_index_array) node->obj_index_array = dm_alloc(sizeof(uint32_t));
        else                       node->obj_index_array = dm_realloc(node->obj_index_array, sizeof(uint32_t) * (node->obj_count + 1));
        
        node->obj_index_array[node->obj_count++] = object_index;
        
        return true;
    }
#endif
    
    // otherwise continue down
    *max_depth = depth > *max_depth ? depth : *max_depth;
    
    // we have children, so go down the tree
    if(node->first_child!=-1)
    {
        // check children
        uint32_t child_index = node->first_child;
        for(uint32_t i=0; i<OCTREE_CHILDREN_COUNT; i++)
        {
            if(octree_insert(object_index, child_index, depth+1, max_depth, octree, node_count, pos_x, pos_y, pos_z)) return true;
            
            child_index++;
        }
        
        // can't fit in children for some reason, so object must go here
        node = &(*octree)[node_index];
        
        if(!node->obj_index_array) node->obj_index_array = dm_alloc(sizeof(uint32_t));
        else                       node->obj_index_array = dm_realloc(node->obj_index_array, sizeof(uint32_t) * (node->obj_count + 1));
        
        node->obj_index_array[node->obj_count++] = object_index;
        
        return true;
    }
    else
    {
        // are we empty?
        if(node->obj_count==0)
        {
            assert(!node->obj_index_array);
            node->obj_index_array = dm_alloc(sizeof(uint32_t));
            node->obj_index_array[node->obj_count++] = object_index;
            return true;
        }
        
        octree_node_init_children(node_index, &node, octree, node_count);
        assert(node->first_child!=-1);
        assert(node->obj_index_array);
        
        uint32_t child_index = node->first_child;
        
        // reinsert any objects here
        // SHOULD be at most one, but you never know...
        for(uint32_t i=0; i<node->obj_count; i++)
        {
            for(uint32_t j=0; j<OCTREE_CHILDREN_COUNT; j++)
            {
                if(octree_insert(node->obj_index_array[i], child_index++, depth+1, max_depth, octree, node_count, pos_x, pos_y, pos_z)) 
                {
                    node->obj_count--;
                    break;
                }
            }
        }
        
        child_index = node->first_child;
        bool failed = true;
        for(uint32_t i=0; i<OCTREE_CHILDREN_COUNT; i++)
        {
            if(!octree_insert(object_index, child_index++, depth+1, max_depth, octree, node_count, pos_x, pos_y, pos_z)) continue;
            
            failed = false;
            return true;
        }
        
        if(failed)
        {
            node->obj_index_array[node->obj_count++] = object_index;
            return true;
        }
        
    }
    
}

void octree_render(const uint32_t node_index, const uint32_t depth, octree_node* octree, void** render_data, dm_context* context)
{
    octree_node node = octree[node_index];
    
    // are we full?
    if(node.obj_count>0)
    {
        dm_vec3 extents = { node.half_extents * 2.f, node.half_extents * 2.f, node.half_extents * 2.f };
        
        float color = 1 - dm_powf(0.8f, (float)depth);
        
        dm_vec4 c = { color,color,color,color };
        debug_render_aabb(node.center, extents, c, render_data, context);
        
        return;
    }
    
    if(node.first_child==-1) return;
    
    uint32_t child_index = node.first_child;
    for(uint32_t i=0; i<OCTREE_CHILDREN_COUNT; i++)
    {
        octree_render(child_index++, depth+1, octree, render_data, context);
    }
}

void octree_node_destroy(const uint32_t node_index, octree_node** octree)
{
    octree_node* node = &(*octree)[node_index];
    
    if(node->obj_index_array) dm_free(node->obj_index_array);
    if(node->first_child==-1) return;
    
    uint32_t child_index = node->first_child;
    for(uint32_t i=0; i<OCTREE_CHILDREN_COUNT; i++)
    {
        octree_node_destroy(child_index, octree);
        child_index++;
    }
}

#endif
