#include "octree.h"
#include "debug_render.h"

#include <assert.h>

void bh_tree_insert(const uint32_t object_index, const uint32_t node_index, float cen_x, float cen_y, float cen_z, float half_extents, const uint32_t depth, bh_tree* tree, const float* pos_x, const float* pos_y, const float* pos_z, const float * mass)
{
    tree->max_depth = depth > tree->max_depth ? depth : tree->max_depth;
    
    const float m = mass[object_index];
    
    tree->com_x[node_index] += pos_x[object_index] * m;
    tree->com_y[node_index] += pos_y[object_index] * m;
    tree->com_z[node_index] += pos_z[object_index] * m;
    
    tree->total_mass[node_index] += m;
    
    // empty with no children
    if(tree->object_index[node_index]==OCTREE_NO_OBJECT && tree->first_child[node_index]==OCTREE_INVALID_CHILD)
    {
        tree->object_index[node_index] = object_index;
        return;
    }
    
    half_extents *= 0.5f;
    const float x_min = cen_x - half_extents;
    const float x_max = cen_x + half_extents;
    const float y_min = cen_y - half_extents;
    const float y_max = cen_y + half_extents;
    const float z_min = cen_z - half_extents;
    const float z_max = cen_z + half_extents;
    
    // no children 
    if(tree->first_child[node_index]==OCTREE_INVALID_CHILD)
    {
        tree->first_child[node_index] = tree->node_count;
        tree->node_count += OCTREE_CHILDREN_COUNT;
        assert(tree->node_count < OCTREE_MAX_NODE_COUNT);
        
        uint32_t child_index = tree->first_child[node_index];
        tree->parent[child_index++] = node_index;
        tree->parent[child_index++] = node_index;
        tree->parent[child_index++] = node_index;
        tree->parent[child_index++] = node_index;
        
        tree->parent[child_index++] = node_index;
        tree->parent[child_index++] = node_index;
        tree->parent[child_index++] = node_index;
        tree->parent[child_index++] = node_index;
    }
    
    // insert old object if needed
    uint32_t child_index = tree->first_child[node_index];
    if(tree->object_index[node_index]!=OCTREE_NO_OBJECT)
    {
        const int old_index = tree->object_index[node_index];
        
        float child_cen_x;
        float child_cen_y;
        float child_cen_z;
        
        child_index = pos_x[old_index] <= cen_x ? child_index : child_index + 1;
        child_index = pos_y[old_index] <= cen_y ? child_index : child_index + 2;
        child_index = pos_z[old_index] <= cen_z ? child_index : child_index + 4;
        
        child_cen_x = pos_x[old_index] <= cen_x ? x_min : x_max;
        child_cen_y = pos_y[old_index] <= cen_y ? y_min : y_max;
        child_cen_z = pos_z[old_index] <= cen_z ? z_min : z_max;
        
        bh_tree_insert(old_index, child_index, child_cen_x, child_cen_y, child_cen_z, half_extents, depth+1, tree, pos_x, pos_y, pos_z, mass);
        tree->object_index[node_index] = OCTREE_NO_OBJECT;
    }
    
    // insert new object
    child_index = tree->first_child[node_index];
    
    child_index = pos_x[object_index] <= cen_x ? child_index : child_index + 1;
    child_index = pos_y[object_index] <= cen_y ? child_index : child_index + 2;
    child_index = pos_z[object_index] <= cen_z ? child_index : child_index + 4;
    
    cen_x = pos_x[object_index] <= cen_x ? x_min : x_max;
    cen_y = pos_y[object_index] <= cen_y ? y_min : y_max;
    cen_z = pos_z[object_index] <= cen_z ? z_min : z_max;
    
    bh_tree_insert(object_index, child_index, cen_x, cen_y, cen_z, half_extents, depth+1, tree, pos_x, pos_y, pos_z, mass);
}

void bh_tree_init(bh_tree* tree)
{
    static const float_array_size = sizeof(float) * OCTREE_MAX_NODE_COUNT;
    static const int_array_size   = sizeof(uint32_t) * OCTREE_MAX_NODE_COUNT;
    
    for(uint32_t i=0; i<OCTREE_MAX_NODE_COUNT; i++)
    {
        tree->com_x[i] = 0;
        tree->com_y[i] = 0;
        tree->com_z[i] = 0;
        tree->total_mass[i] = 0;
        
        tree->parent[i]       = OCTREE_INVALID_CHILD;
        tree->first_child[i]  = OCTREE_INVALID_CHILD;
        tree->object_index[i] = OCTREE_NO_OBJECT;
    }
    
    tree->node_count = 1;
}

void bh_tree_render_node(const uint32_t node_index, const float cen_x, const float cen_y, const float cen_z, const float half_extents, const uint32_t depth, const bh_tree* tree, void** render_data, dm_context* context)
{
    if(tree->object_index[node_index]==OCTREE_NO_OBJECT && tree->first_child[node_index]==OCTREE_INVALID_CHILD) return;
    
    const float new_half_extents = half_extents * 0.5f;
    
    if(tree->object_index[node_index]==OCTREE_NO_OBJECT)
    {
        uint32_t child_index = tree->first_child[node_index];
        
        const float x_min = cen_x - new_half_extents;
        const float x_max = cen_x + new_half_extents;
        const float y_min = cen_y - new_half_extents;
        const float y_max = cen_y + new_half_extents;
        const float z_min = cen_z - new_half_extents;
        const float z_max = cen_z + new_half_extents;
        
        bh_tree_render_node(child_index++, x_min,y_min,z_min,new_half_extents, depth+1, tree, render_data, context);
        bh_tree_render_node(child_index++, x_max,y_min,z_min,new_half_extents, depth+1, tree, render_data, context);
        bh_tree_render_node(child_index++, x_min,y_max,z_min,new_half_extents, depth+1, tree, render_data, context);
        bh_tree_render_node(child_index++, x_max,y_max,z_min,new_half_extents, depth+1, tree, render_data, context);
        
        bh_tree_render_node(child_index++, x_min,y_min,z_max,new_half_extents, depth+1, tree, render_data, context);
        bh_tree_render_node(child_index++, x_max,y_min,z_max,new_half_extents, depth+1, tree, render_data, context);
        bh_tree_render_node(child_index++, x_min,y_max,z_max,new_half_extents, depth+1, tree, render_data, context);
        bh_tree_render_node(child_index++, x_max,y_max,z_max,new_half_extents, depth+1, tree, render_data, context);
    }
    else
    {
        const dm_vec4 c = {
            1,1,1,dm_powf(0.75f,(float)depth)
        };
        dm_vec3 d = {
            half_extents,
            half_extents,
            half_extents
        };
        dm_vec3_scale(d,2.f,d);
        
        const dm_vec3 center = { cen_x,cen_y,cen_z };
        
        debug_render_aabb(center, d, c, render_data, context);
        
        dm_vec3 com = { 
            tree->com_x[node_index]/tree->total_mass[node_index], 
            tree->com_y[node_index]/tree->total_mass[node_index],
            tree->com_z[node_index]/tree->total_mass[node_index] };
        dm_vec3 dim = { 0.05f,0.05f,0.05f };
        dm_vec4 color = { 1,0,0,1 };
        debug_render_aabb(com, dim, color, render_data, context);
    }
}

void bh_tree_render(const float cen_x, const float cen_y, const float cen_z, const float half_extents, const bh_tree* tree, void** render_data, dm_context* context)
{
    bh_tree_render_node(0, cen_x,cen_y,cen_z,half_extents,0, tree, render_data, context);
}