#include "dm.h"
#include "camera.h"
#include <float.h>

typedef struct hit_payload_t
{
    float hit_distance;
    
    dm_vec3 world_position;
    dm_vec3 world_normal;
    
    uint32_t obj_index;
} hit_payload;

typedef struct ray_t
{
    dm_vec3 origin;
    dm_vec3 direction;
    
    float    hit_distance;
    uint32_t hit_index;
} ray;

typedef struct app_handles_t
{
    dm_render_handle vb, shader, pipeline;
    dm_render_handle texture;
    dm_render_handle compute_shader;
} app_handles;

typedef struct app_image_t
{
    uint32_t  w,h;
    size_t    data_size;
    
    uint32_t frame_index;
    
    uint32_t* data;
    dm_vec4*  accumulated_data;
    uint32_t* random_numbers;
} app_image;

#define MAX_MATERIAL_COUNT 100
typedef struct material_data_t
{
    DM_ALIGN(16) float albedo_r[MAX_MATERIAL_COUNT];
    DM_ALIGN(16) float albedo_g[MAX_MATERIAL_COUNT];
    DM_ALIGN(16) float albedo_b[MAX_MATERIAL_COUNT];
    
    DM_ALIGN(16) float roughness[MAX_MATERIAL_COUNT];
    DM_ALIGN(16) float metallic[MAX_MATERIAL_COUNT];
    
    DM_ALIGN(16) float emission_r[MAX_MATERIAL_COUNT];
    DM_ALIGN(16) float emission_g[MAX_MATERIAL_COUNT];
    DM_ALIGN(16) float emission_b[MAX_MATERIAL_COUNT];
    
    DM_ALIGN(16) float emission_power[MAX_MATERIAL_COUNT];
    
    uint32_t count;
} material_data;

#define MAX_SPHERE_COUNT 100
typedef struct sphere_data_t
{
    DM_ALIGN(16) float x[MAX_SPHERE_COUNT];
    DM_ALIGN(16) float y[MAX_SPHERE_COUNT];
    DM_ALIGN(16) float z[MAX_SPHERE_COUNT];
    DM_ALIGN(16) float radius[MAX_SPHERE_COUNT];
    DM_ALIGN(16) float radius_2[MAX_SPHERE_COUNT];
    
    DM_ALIGN(16) uint32_t material_id[MAX_SPHERE_COUNT];
    
    DM_ALIGN(16) float aabb_min_x[MAX_SPHERE_COUNT];
    DM_ALIGN(16) float aabb_min_y[MAX_SPHERE_COUNT];
    DM_ALIGN(16) float aabb_min_z[MAX_SPHERE_COUNT];
    
    DM_ALIGN(16) float aabb_max_x[MAX_SPHERE_COUNT];
    DM_ALIGN(16) float aabb_max_y[MAX_SPHERE_COUNT];
    DM_ALIGN(16) float aabb_max_z[MAX_SPHERE_COUNT];
    
    uint32_t count;
} sphere_data;

typedef struct bvh_node_t
{
    dm_vec3 aabb_min;
    dm_vec3 aabb_max;
    
    uint32_t start, count;
    
    uint32_t children[2];
} bvh_node;

typedef struct bvh_t
{
    bvh_node nodes[MAX_SPHERE_COUNT * 2 - 1];
    uint32_t indices[MAX_SPHERE_COUNT];
    uint32_t num_levels, invalid_index;
} bvh;

typedef struct mt_data_t
{
    uint32_t y_incr, width;
    dm_vec4  sky_color;
    dm_vec3  ray_pos;
    uint32_t frame_index;
    
    uint32_t rays_processed;
    
    bvh      b;
    
    sphere_data*   spheres;
    material_data* materials;
    
    uint32_t*    image_data;
    dm_vec4*     accumulated_data;
    uint32_t*    random_numbers;
    
    dm_vec4*     ray_dirs;
} mt_data;

#define NUM_TASKS 100
typedef struct application_data_t
{
    app_handles handles;
    app_image   image;
    
    basic_camera camera;
    
    dm_vec4 sky_color;
    
    // timings
    dm_timer timer;
    double image_creation_t, ray_creation_t;
    double seconds_timer;
    uint32_t rays_processed;
    
    // mt stuff
    dm_threadpool  threadpool;
    dm_thread_task tasks[NUM_TASKS];
    mt_data        thread_data[NUM_TASKS];
    
    bool accumulate;
    
    // ray directions
    dm_vec4* ray_dirs;
    
    // objects
    sphere_data spheres;
    
    // materials
    material_data materials;
    
    // bvh
    bvh b;
} application_data;

/*************
INTERSECTIONS
***************/
bool ray_intersects_aabb(const ray* r, const dm_vec3 aabb_min, const dm_vec3 aabb_max)
{
    float t_min, t_max;
    
    dm_vec3 mins, maxes;
    
    dm_vec3_sub_vec3(aabb_min, r->origin, mins);
    dm_vec3_div_vec3(mins, r->direction, mins);
    
    dm_vec3_sub_vec3(aabb_max, r->origin, maxes);
    dm_vec3_div_vec3(maxes, r->direction, maxes);
    
    t_min = DM_MIN(mins[0], maxes[0]);
    t_max = DM_MAX(mins[0], maxes[0]);
    
    t_min = DM_MIN(mins[1], maxes[1]);
    t_max = DM_MAX(mins[1], maxes[1]);
    
    t_min = DM_MIN(mins[2], maxes[2]);
    t_max = DM_MAX(mins[2], maxes[2]);
    
    bool result = t_max >= t_min;
    result = result && (t_min < r->hit_distance);
    result = result && (t_max > 0);
    
    return result;
}

void ray_intersects_sphere(ray* r, const uint32_t index, sphere_data* spheres)
{
    dm_vec3 origin;
    
    origin[0] = r->origin[0] - spheres->x[index];
    origin[1] = r->origin[1] - spheres->y[index];
    origin[2] = r->origin[2] - spheres->z[index];
    
    float b = 2.0f * dm_vec3_dot(origin, r->direction);
    float c = dm_vec3_dot(origin, origin) - spheres->radius_2[index];
    
    float dis = b * b - 4.0f * c;
    
    // don't hit this sphere
    if(dis < 0) return;
    
    // are we the closest sphere?
    float closest_t = (-b - dm_sqrtf(dis)) * 0.5f;
    if (closest_t > r->hit_distance || closest_t < 0) return;
    
    r->hit_distance = closest_t;
    r->hit_index    = index;
}

void ray_intersect_bvh(ray* r, uint32_t index, sphere_data* spheres, bvh* b)
{
    if(!ray_intersects_aabb(r, b->nodes[index].aabb_min, b->nodes[index].aabb_max)) return;
    
    if(b->nodes[index].count > 0)
    {
        for(uint32_t i=0; i<b->nodes[index].count; i++)
        {
            ray_intersects_sphere(r, b->indices[b->nodes[index].start + i], spheres);
        }
    }
    else
    {
        ray_intersect_bvh(r, b->nodes[index].children[0], spheres, b);
        ray_intersect_bvh(r, b->nodes[index].children[1], spheres, b);
    }
}

/***
BVH
*****/
void bvh_node_init(uint32_t* index, uint32_t current_level, bvh* b)
{
    uint32_t current_index = *index;
    
    // left side
    b->nodes[current_index].children[0] = ++(*index);
    
    if(current_level < b->num_levels-1)
    {
        bvh_node_init(index, current_level + 1, b);
    }
    else
    {
        b->nodes[b->nodes[current_index].children[0]].children[0] = b->invalid_index;
        b->nodes[b->nodes[current_index].children[0]].children[1] = b->invalid_index;
    }
    
    // right side
    b->nodes[current_index].children[1] = ++(*index);
    
    if(current_level < b->num_levels-1)
    {
        bvh_node_init(index, current_level + 1, b);
    }
    else
    {
        b->nodes[b->nodes[current_index].children[1]].children[0] = b->invalid_index;
        b->nodes[b->nodes[current_index].children[1]].children[1] = b->invalid_index;
    }
}

void bvh_init(uint32_t num_spheres, bvh* b)
{
    b->num_levels    = dm_ceil(dm_log2f((float)num_spheres));
    b->invalid_index = num_spheres * 2 - 1;
    
    for(uint32_t i=0; i<num_spheres; i++)
    {
        b->indices[i] = i;
    }
    
    uint32_t index = 0;
    bvh_node_init(&index, 0, b);
}

void bvh_update_node_bounds(uint32_t index, sphere_data* spheres, bvh* b)
{
    float aabb_min_x, aabb_min_y, aabb_min_z;
    float aabb_max_x, aabb_max_y, aabb_max_z;
    
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    
    aabb_min_x = aabb_min_y = aabb_min_z = FLT_MAX;
    aabb_max_x = aabb_max_y = aabb_max_z = -FLT_MAX;
    
    uint32_t s_index;
    for(uint32_t i=0; i<b->nodes[index].count; i++)
    {
        s_index = b->indices[i + b->nodes[index].start];
        
        min_x = spheres->aabb_min_x[s_index];
        min_y = spheres->aabb_min_y[s_index];
        min_z = spheres->aabb_min_z[s_index];
        
        max_x = spheres->aabb_max_x[s_index];
        max_y = spheres->aabb_max_y[s_index];
        max_z = spheres->aabb_max_z[s_index];
        
        aabb_min_x = (min_x < aabb_min_x) ? min_x : aabb_min_x;
        aabb_min_y = (min_y < aabb_min_y) ? min_y : aabb_min_y;
        aabb_min_z = (min_z < aabb_min_z) ? min_z : aabb_min_z;
        
        aabb_max_x = (max_x > aabb_max_x) ? max_x : aabb_max_x;
        aabb_max_y = (max_y > aabb_max_y) ? max_y : aabb_max_y;
        aabb_max_z = (max_z > aabb_max_z) ? max_z : aabb_max_z;
    }
    
    b->nodes[index].aabb_min[0] = aabb_min_x;
    b->nodes[index].aabb_min[1] = aabb_min_y;
    b->nodes[index].aabb_min[2] = aabb_min_z;
    
    b->nodes[index].aabb_max[0] = aabb_max_x;
    b->nodes[index].aabb_max[1] = aabb_max_y;
    b->nodes[index].aabb_max[2] = aabb_max_z;
}

void bvh_node_subdivide(uint32_t index, sphere_data* spheres, bvh* b)
{
    if(b->nodes[index].count <= 2) return;
    
    dm_vec3 extents;
    dm_vec3_sub_vec3(b->nodes[index].aabb_max, b->nodes[index].aabb_min, extents);
    int axis = 0;
    if(extents[1] > extents[0]) axis = 1;
    if(extents[2] > extents[axis]) axis = 2;
    
    float split_pos = b->nodes[index].aabb_min[axis] + extents[axis] * 0.5f;
    
    int i = b->nodes[index].start;
    int j = i + b->nodes[index].count - 1;
    
    while(i <= j)
    {
        bool swap = false;
        switch(axis)
        {
            case 0:
            if(spheres->x[b->indices[i]] < split_pos) i++; 
            else swap = true;
            break;
            
            case 1:
            if(spheres->y[b->indices[i]] < split_pos) i++; 
            else swap = true;
            break;
            
            case 2:
            if(spheres->z[b->indices[i]] < split_pos) i++; 
            else swap = true;
            break;
            
            default:
            break;
        }
        
        if(!swap) continue;
        
        uint32_t temp = b->indices[i];
        b->indices[i] = b->indices[j];
        b->indices[j--] = temp;
    }
    
    int left_count = i - b->nodes[index].start;
    if(left_count==0 || left_count==b->nodes[index].count) return;
    
    b->nodes[b->nodes[index].children[0]].start = b->nodes[index].start;
    b->nodes[b->nodes[index].children[0]].count = left_count;
    
    b->nodes[b->nodes[index].children[1]].start = i;
    b->nodes[b->nodes[index].children[1]].count = b->nodes[index].count - left_count;
    
    b->nodes[index].count = 0;
    
    bvh_update_node_bounds(b->nodes[index].children[0], spheres, b);
    bvh_update_node_bounds(b->nodes[index].children[1], spheres, b);
    
    bvh_node_subdivide(b->nodes[index].children[0], spheres, b);
    bvh_node_subdivide(b->nodes[index].children[1], spheres, b);
}

void bvh_populate(sphere_data* spheres, bvh* b)
{
    b->nodes[0].start = 0;
    b->nodes[0].count = spheres->count;
    
    bvh_update_node_bounds(0, spheres, b);
    bvh_node_subdivide(0, spheres, b);
}

/*******
HELPERS
*********/
void make_sphere(const float x, const float y, const float z, const float radius, application_data* app_data, dm_context* context)
{
    sphere_data* spheres = &app_data->spheres;
    const uint32_t index = spheres->count;
    
    spheres->x[index] = x;
    spheres->y[index] = y;
    spheres->z[index] = z;
    
    spheres->radius[index]   = radius;
    spheres->radius_2[index] = radius * radius;
    
    spheres->material_id[index] = dm_random_uint32_range(0,MAX_MATERIAL_COUNT, context);
    
    spheres->aabb_min_x[index] = x - radius;
    spheres->aabb_min_y[index] = y - radius;
    spheres->aabb_min_z[index] = z - radius;
    
    spheres->aabb_max_x[index] = x + radius;
    spheres->aabb_max_y[index] = y + radius;
    spheres->aabb_max_z[index] = z + radius;
    
    spheres->count++;
}

void make_random_sphere(application_data* app_data, dm_context* context)
{
    sphere_data* spheres = &app_data->spheres;
    const uint32_t index = spheres->count;
    
    const float x = dm_random_float_range(-10,10,context);
    const float y = dm_random_float_range(-10,10,context);
    const float z = dm_random_float_range(-10,10,context);
    
    spheres->x[index] = x;
    spheres->y[index] = y;
    spheres->z[index] = z;
    
    const float radius = dm_random_float_range(0.1f,2.0f, context);
    
    spheres->radius[index]   = radius;
    spheres->radius_2[index] = radius * radius;
    
    const float r = dm_random_float(context);
    const float g = dm_random_float(context);
    const float b = dm_random_float(context);
    
    spheres->material_id[index] = (uint32_t)dm_random_float_range(0,MAX_MATERIAL_COUNT, context);
    
    spheres->aabb_min_x[index] = x - radius;
    spheres->aabb_min_y[index] = y - radius;
    spheres->aabb_min_z[index] = z - radius;
    
    spheres->aabb_max_x[index] = x + radius;
    spheres->aabb_max_y[index] = y + radius;
    spheres->aabb_max_z[index] = z + radius;
    
    spheres->count++;
}

void reset_frame_index(application_data* app_data)
{
    app_data->image.frame_index = 1;
}

uint32_t vec4_to_uint32(const float vec[4])
{
    const uint8_t r = (uint8_t)(255.0f * vec[0]);
    const uint8_t g = (uint8_t)(255.0f * vec[1]);
    const uint8_t b = (uint8_t)(255.0f * vec[2]);
    const uint8_t a = (uint8_t)(255.0f * vec[3]);
    
    uint32_t result = (a << 24) | (b << 16) | (g << 8) | r;
    return result;
}

uint32_t pcg_hash(uint32_t input)
{
    uint32_t state = input * 747796405u + 2891336453u;
    uint32_t word  = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

float random_float(uint32_t* seed)
{
    *seed = pcg_hash(*seed);
    
    return (float)*seed / (float)UINT_MAX;
}

hit_payload closest_hit(const ray r, float hit_distance, int obj_index, dm_vec4 color, sphere_data* spheres)
{
    hit_payload payload = {
        .hit_distance=hit_distance,
        .obj_index=obj_index
    };
    
    dm_vec3 origin;
    origin[0] = r.origin[0] - spheres->x[obj_index];
    origin[1] = r.origin[1] - spheres->y[obj_index];
    origin[2] = r.origin[2] - spheres->z[obj_index];
    
    dm_vec3 hit_point;
    dm_vec3_scale(r.direction, hit_distance, hit_point);
    dm_vec3_add_vec3(origin, hit_point, payload.world_position);
    dm_vec3_norm(payload.world_position, payload.world_normal);
    
    payload.world_position[0] += spheres->x[obj_index];
    payload.world_position[1] += spheres->y[obj_index];
    payload.world_position[2] += spheres->z[obj_index];
    
    return payload;
}

hit_payload miss(const ray r)
{
    hit_payload payload = {
        .hit_distance=-1
    };
    
    return payload;
}

void trace_ray2(ray* r, float color[4], sphere_data* spheres)
{
    dm_mm_float origin_x, origin_y, origin_z;
    dm_mm_float sphere_r2;
    dm_mm_float b, c, d, d_sqrt, b2;
    dm_mm_float d_mask, hit_mask, final_mask, close_mask;
    dm_mm_float closest_t;
    
    const dm_mm_float ray_origin_x = dm_mm_set1_ps(r->origin[0]);
    const dm_mm_float ray_origin_y = dm_mm_set1_ps(r->origin[1]);
    const dm_mm_float ray_origin_z = dm_mm_set1_ps(r->origin[2]);
    
    const dm_mm_float ray_dir_x = dm_mm_set1_ps(r->direction[0]);
    const dm_mm_float ray_dir_y = dm_mm_set1_ps(r->direction[1]);
    const dm_mm_float ray_dir_z = dm_mm_set1_ps(r->direction[2]);
    
    static int starting_indices[] = { 0,1,2,3 };
    
    dm_mm_int indices = dm_mm_load_i(starting_indices);
    
    const dm_mm_float zeros = dm_mm_set1_ps(0);
    const dm_mm_float halfs = dm_mm_set1_ps(0.5f);
    const dm_mm_float twos  = dm_mm_set1_ps(2.0f);
    
    const dm_mm_int fours_i = dm_mm_set1_i(4);
    
    dm_mm_int   hit_index = dm_mm_set1_i(-1);
    dm_mm_float hit_t     = dm_mm_set1_ps(FLT_MAX);
    
    for(uint32_t i=0; i<spheres->count; i+=4)
    {
        origin_x  = dm_mm_load_ps(spheres->x + i);
        origin_y  = dm_mm_load_ps(spheres->y + i);
        origin_z  = dm_mm_load_ps(spheres->z + i);
        
        // o = ray.o - sphere.o
        origin_x = dm_mm_sub_ps(origin_x, ray_origin_x);
        origin_y = dm_mm_sub_ps(origin_y, ray_origin_y);
        origin_z = dm_mm_sub_ps(origin_z, ray_origin_z);
        
        // c = dot(o,o) - sphere.r * sphere.r
        c = dm_mm_mul_ps(origin_x,origin_x);
        c = dm_mm_fmadd_ps(origin_y,origin_y, c);
        c = dm_mm_fmadd_ps(origin_z,origin_z, c);
        c = dm_mm_sub_ps(c, dm_mm_load_ps(spheres->radius_2 + i));
        
        // b = 2 * dot(o, r.dir)
        b  = dm_mm_mul_ps(origin_x, ray_dir_x);
        b  = dm_mm_fmadd_ps(origin_y,ray_dir_y, b);
        b  = dm_mm_fmadd_ps(origin_z,ray_dir_z, b);
        b  = dm_mm_mul_ps(b, twos);
        
        // d = b * b - 4 * c
        d = dm_mm_mul_ps(twos, c);
        d = dm_mm_mul_ps(twos, d);
        d = dm_mm_sub_ps(dm_mm_mul_ps(b,b), d);
        
        // d > 0
        d_mask = dm_mm_gt_ps(d, zeros);
        
        // closest hit point
        d_sqrt = dm_mm_sqrt_ps(d);
        
        // (-b - sqrt(d)) / 2
        closest_t = dm_mm_sub_ps(b, d_sqrt);
        closest_t = dm_mm_mul_ps(closest_t, halfs);
        
        // closest_t > 0
        close_mask = dm_mm_lt_ps(closest_t, hit_t);
        hit_mask   = dm_mm_gt_ps(closest_t, zeros);
        hit_mask   = dm_mm_and_ps(hit_mask, close_mask);
        
        final_mask = dm_mm_and_ps(d_mask, hit_mask);
        
        hit_t     = dm_mm_blendv_ps(hit_t, closest_t, final_mask);
#ifdef DM_SIMD_X86
        hit_index = dm_mm_blendv_i(hit_index, indices, dm_mm_cast_float_to_int(final_mask));
#else
        hit_index = dm_mm_blendv_i(hit_index, indices, final_mask);
#endif
        // iterate
        indices = dm_mm_add_i(indices, fours_i);
    }
    
    int     hit_inds[4];
    dm_vec4 hits;
    
    dm_mm_store_ps(hits, hit_t);
    dm_mm_store_i(hit_inds, hit_index);
    
    float nearest_hit = FLT_MAX;
    int   nearest_index = -1;
    
    for(uint32_t i=0; i<4; i++)
    {
        if(hits[i] >= nearest_hit) continue;
        
        nearest_hit   = hits[i];
        nearest_index = hit_inds[i];
    }
    
    if(nearest_index < 0) return;
    
    r->hit_distance = nearest_hit;
    r->hit_index    = nearest_index;
}

void trace_ray3(ray* r, sphere_data* spheres, bvh* b)
{
    r->hit_distance = FLT_MAX;
    r->hit_index    = UINT_MAX;
    
    ray_intersect_bvh(r, 0, spheres, b);
}

hit_payload trace_ray(const ray r, float color[4], sphere_data* spheres)
{
    dm_vec3 origin;
    
    float closest_t;
    float hit_distance = FLT_MAX;
    int   nearest_sphere_index = -1;
    
    float b, c, dis;
    
    for(uint32_t i=0; i<spheres->count; i++)
    {
        origin[0] = r.origin[0] - spheres->x[i];
        origin[1] = r.origin[1] - spheres->y[i];
        origin[2] = r.origin[2] - spheres->z[i];
        
        b = 2.0f * dm_vec3_dot(origin, r.direction);
        c = dm_vec3_dot(origin, origin) - spheres->radius_2[i];
        
        dis = b * b - 4.0f * c;
        
        // don't hit this sphere
        if(dis < 0) continue;
        
        // are we the closest sphere?
        closest_t = (-b - dm_sqrtf(dis)) * 0.5f;
        if(closest_t > hit_distance || closest_t < 0) continue;
        
        hit_distance = closest_t;
        nearest_sphere_index = i;
    }
    
    if(nearest_sphere_index<0) return miss(r);
    
    return closest_hit(r, hit_distance, nearest_sphere_index, color, spheres);
}

void per_pixel(uint32_t x, uint32_t y, dm_vec4 color, dm_vec4 sky_color, dm_vec3 pos, dm_vec3 dir, uint32_t seed, uint32_t* ray_count, sphere_data* spheres, material_data* materials, bvh* b)
{
    ray r;
    
    r.origin[0] = pos[0];
    r.origin[1] = pos[1];
    r.origin[2] = pos[2];
    
    r.direction[0] = dir[0];
    r.direction[1] = dir[1];
    r.direction[2] = dir[2];
    
    uint32_t bounces      = 5;
    dm_vec3  contribution = { 1,1,1 };
    dm_vec3  light        = { 0 };
    
    float light_r = 0;
    float light_g = 0;
    float light_b = 0;
    
    dm_vec3 sampling;
    
    for(uint32_t i=0; i<bounces; i++)
    {
        seed += i;
        *ray_count = *ray_count + 1;
        
        //payload = trace_ray(r, color, spheres);
        //trace_ray2(&r, color, spheres);
        trace_ray3(&r, spheres, b);
        if(r.hit_index==UINT_MAX) break;
        
        dm_vec3 origin;
        origin[0] = r.origin[0] - spheres->x[r.hit_index];
        origin[1] = r.origin[1] - spheres->y[r.hit_index];
        origin[2] = r.origin[2] - spheres->z[r.hit_index];
        
        dm_vec3 hit_point, world_pos, world_norm;
        dm_vec3_scale(r.direction, r.hit_distance, hit_point);
        dm_vec3_add_vec3(origin, hit_point, world_pos);
        dm_vec3_norm(world_pos, world_norm);
        
        world_pos[0] += spheres->x[r.hit_index];
        world_pos[1] += spheres->y[r.hit_index];
        world_pos[2] += spheres->z[r.hit_index];
        
        // determine color 
        const uint32_t mat_id = spheres->material_id[r.hit_index];
        float emission_power = materials->emission_power[mat_id];
        
        light_r += materials->emission_r[mat_id] * emission_power;
        light_g += materials->emission_g[mat_id] * emission_power;
        light_b += materials->emission_b[mat_id] * emission_power;
        
        dm_vec3 offset_pos;
        dm_vec3 scaled_normal;
        dm_vec3_scale(world_norm, 0.0001f, scaled_normal);
        dm_vec3_add_vec3(world_pos, scaled_normal, offset_pos);
        
        r.origin[0] = offset_pos[0];
        r.origin[1] = offset_pos[1];
        r.origin[2] = offset_pos[2];
        
        // scattering
        sampling[0] = random_float(&seed) * 2.0f - 1.0f; 
        sampling[1] = random_float(&seed) * 2.0f - 1.0f; 
        sampling[2] = random_float(&seed) * 2.0f - 1.0f; 
        dm_vec3_norm(sampling, sampling);
        
        // biased towards normal
        dm_vec3_add_vec3(sampling, world_norm, sampling);
        //dm_vec3_norm(sampling, sampling);
        
        dm_vec3_add_vec3(r.direction, sampling, r.direction);
        dm_vec3_norm(r.direction, r.direction);
        
        r.hit_distance = FLT_MAX;
        r.hit_index    = UINT_MAX;
    }
    
    color[0] = light_r;
    color[1] = light_g;
    color[2] = light_b;
    color[3] = 1;
}

void recreate_rays(application_data* app_data)
{
    if(app_data->ray_dirs) 
    {
        app_data->ray_dirs = dm_realloc(app_data->ray_dirs, DM_VEC4_SIZE * app_data->image.w * app_data->image.h);
    }
    else
    {
        app_data->ray_dirs = dm_alloc(DM_VEC4_SIZE * app_data->image.w * app_data->image.h);
    }
    
    const float height_f_inv = 1.0f / (float)app_data->image.h;
    const float width_f_inv  = 1.0f / (float)app_data->image.w;
    
    uint32_t index;
    
    dm_mm_float w_mm, target_mm, row1,row2,row3,row4;
    dm_mm_float target_x_mm, target_y_mm, target_z_mm;
    dm_mm_float coords_mm, coords_x_mm, coords_y_mm, coords_z_mm, coords_w_mm;
    
    //////////// camera matrices are constant, so do them outside
    dm_mm_float proj_row1, proj_row2, proj_row3, proj_row4;
    dm_mm_float view_row1, view_row2, view_row3, view_row4;
    
#ifdef DM_SIMD_ARM
    dm_mat4 temp;
    dm_mat4_transpose(app_data->camera.inv_proj, temp);
    
    proj_row1 = dm_mm_load_ps(temp[0]);
    proj_row2 = dm_mm_load_ps(temp[1]);
    proj_row3 = dm_mm_load_ps(temp[2]);
    proj_row4 = dm_mm_load_ps(temp[3]);
    
    dm_mat4_transpose(app_data->camera.inv_view, temp);
    
    view_row1 = dm_mm_load_ps(temp[0]);
    view_row2 = dm_mm_load_ps(temp[1]);
    view_row3 = dm_mm_load_ps(temp[2]);
    view_row4 = dm_mm_load_ps(temp[3]);
    
#elif defined(DM_SIMD_X86)
    proj_row1 = dm_mm_load_ps(app_data->camera.inv_proj[0]);
    proj_row2 = dm_mm_load_ps(app_data->camera.inv_proj[1]);
    proj_row3 = dm_mm_load_ps(app_data->camera.inv_proj[2]);
    proj_row4 = dm_mm_load_ps(app_data->camera.inv_proj[3]);
    
    dm_mm_transpose_mat4(&proj_row1, &proj_row2, &proj_row3, &proj_row4);
    
    view_row1 = dm_mm_load_ps(app_data->camera.inv_view[0]);
    view_row2 = dm_mm_load_ps(app_data->camera.inv_view[1]);
    view_row3 = dm_mm_load_ps(app_data->camera.inv_view[2]);
    view_row4 = dm_mm_load_ps(app_data->camera.inv_view[3]);
    
    dm_mm_transpose_mat4(&view_row1, &view_row2, &view_row3, &view_row4);
#endif
    
    //////////////////////
    coords_z_mm = dm_mm_set1_ps(1);
    coords_w_mm = dm_mm_set1_ps(1);
    
    const dm_mm_float ones     = dm_mm_set1_ps(1.f);
    const dm_mm_float neg_ones = dm_mm_set1_ps(-1.0f);
    const dm_mm_float twos     = dm_mm_set1_ps(2.0f);
    
    dm_mm_float mag;
    
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        index = y * app_data->image.w;
        
        coords_y_mm = dm_mm_set1_ps((float)y);
        coords_y_mm = dm_mm_mul_ps(coords_y_mm, dm_mm_set1_ps(height_f_inv));
        coords_y_mm = dm_mm_mul_ps(coords_y_mm, twos);
        coords_y_mm = dm_mm_add_ps(coords_y_mm, neg_ones);
        
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            /////////////////
            coords_x_mm = dm_mm_set1_ps((float)x);
            coords_x_mm = dm_mm_mul_ps(coords_x_mm, dm_mm_set1_ps(width_f_inv));
            coords_x_mm = dm_mm_mul_ps(coords_x_mm, twos);
            coords_x_mm = dm_mm_add_ps(coords_x_mm, neg_ones);
            
            target_mm = dm_mm_mul_ps(coords_x_mm, proj_row1);
            target_mm = dm_mm_fmadd_ps(coords_y_mm,proj_row2, target_mm);
            target_mm = dm_mm_add_ps(target_mm, proj_row3);
            target_mm = dm_mm_add_ps(target_mm, proj_row4);
            
            w_mm = dm_mm_broadcast_w_ps(target_mm);
            target_mm = dm_mm_div_ps(target_mm, w_mm);
            //target_mm = dm_mm_normalize_ps(target_mm);
            
            target_x_mm = dm_mm_broadcast_x_ps(target_mm);
            target_y_mm = dm_mm_broadcast_y_ps(target_mm);
            target_z_mm = dm_mm_broadcast_z_ps(target_mm);
            
            mag = dm_mm_mul_ps(target_x_mm, target_x_mm);
            mag = dm_mm_fmadd_ps(target_y_mm,target_y_mm, mag);
            mag = dm_mm_fmadd_ps(target_z_mm,target_z_mm, mag);
            mag = dm_mm_sqrt_ps(mag);
            mag = dm_mm_div_ps(ones, mag);
            
            target_x_mm = dm_mm_mul_ps(target_x_mm, mag);
            target_y_mm = dm_mm_mul_ps(target_y_mm, mag);
            target_z_mm = dm_mm_mul_ps(target_z_mm, mag);
            
            target_mm = dm_mm_mul_ps(target_x_mm, view_row1);
            target_mm = dm_mm_fmadd_ps(target_y_mm,view_row2, target_mm);
            target_mm = dm_mm_fmadd_ps(target_z_mm,view_row3, target_mm);
            
            dm_mm_store_ps(app_data->ray_dirs[index++], target_mm);
            /////
        }
    }
}

#define SAMPLES 1
static const float sample_inv = 1.0f / SAMPLES;

void create_image(application_data* app_data, dm_context* context)
{
    float color[N4] = { 1,1,1,1 };
    dm_vec3 dir;
    uint32_t seed;
    
    if(app_data->image.frame_index==1)
    {
        dm_memzero(app_data->image.accumulated_data, sizeof(dm_vec4) * app_data->image.w * app_data->image.h);
    }
    
    const float inv_frame_index = 1.0f / (float)app_data->image.frame_index;
    
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            seed = app_data->image.random_numbers[x + y * app_data->image.w];
            
            color[0] = 0;
            color[1] = 0;
            color[2] = 0;
            color[3] = 1;
            
            for(uint32_t i=0; i<SAMPLES; i++)
            {
                seed *= app_data->image.frame_index;
                
                dir[0] = app_data->ray_dirs[x + y * app_data->image.w][0];
                dir[1] = app_data->ray_dirs[x + y * app_data->image.w][1];
                dir[2] = app_data->ray_dirs[x + y * app_data->image.w][2];
                per_pixel(x,y, color, app_data->sky_color, app_data->camera.pos, dir, seed, &app_data->rays_processed, &app_data->spheres, &app_data->materials, &app_data->b);
            }
            color[0] *= sample_inv;
            color[1] *= sample_inv;
            color[2] *= sample_inv;
            
            app_data->image.accumulated_data[x + y * app_data->image.w][0] += color[0];
            app_data->image.accumulated_data[x + y * app_data->image.w][1] += color[1];
            app_data->image.accumulated_data[x + y * app_data->image.w][2] += color[2];
            app_data->image.accumulated_data[x + y * app_data->image.w][3] += 1;
            
            color[0] = app_data->image.accumulated_data[x + y * app_data->image.w][0];
            color[1] = app_data->image.accumulated_data[x + y * app_data->image.w][1];
            color[2] = app_data->image.accumulated_data[x + y * app_data->image.w][2];
            color[3] = app_data->image.accumulated_data[x + y * app_data->image.w][3];
            
            color[0] *= inv_frame_index;
            color[1] *= inv_frame_index;
            color[2] *= inv_frame_index;
            color[3] *= inv_frame_index;
            
            color[0] = dm_clamp(color[0], 0, 1);
            color[1] = dm_clamp(color[1], 0, 1);
            color[2] = dm_clamp(color[2], 0, 1);
            
            app_data->image.data[x + y * app_data->image.w] = vec4_to_uint32(color);
        }
    }
}

void* image_mt_func(void* func_data)
{
    mt_data* data = func_data;
    
    dm_vec4 color = { 1,1,1,1 };
    dm_vec3 dir;
    
    uint32_t seed;
    
    const float inv_frame_index = 1.0f / (float)data->frame_index;
    
    dm_vec4 sample_color;
    
    for(uint32_t y=0; y<data->y_incr; y++)
    {
        for(uint32_t x=0; x<data->width; x++)
        {
            seed = data->random_numbers[x + y * data->width];
            
            sample_color[0] = 0;
            sample_color[1] = 0;
            sample_color[2] = 0;
            sample_color[3] = 1;
            
            for(uint32_t i=0; i<SAMPLES; i++)
            {
                seed++;
                
                dir[0] = data->ray_dirs[x + y * data->width][0];
                dir[1] = data->ray_dirs[x + y * data->width][1];
                dir[2] = data->ray_dirs[x + y * data->width][2];
                per_pixel(x,y, sample_color, data->sky_color, data->ray_pos, dir, seed, &data->rays_processed, data->spheres, data->materials, &data->b);
            }
            
            sample_color[0] *= sample_inv;
            sample_color[1] *= sample_inv;
            sample_color[2] *= sample_inv;
            
            data->accumulated_data[x + y * data->width][0] += sample_color[0];
            data->accumulated_data[x + y * data->width][1] += sample_color[1];
            data->accumulated_data[x + y * data->width][2] += sample_color[2];
            data->accumulated_data[x + y * data->width][3] += 1;
            
            color[0] = data->accumulated_data[x + y * data->width][0];
            color[1] = data->accumulated_data[x + y * data->width][1];
            color[2] = data->accumulated_data[x + y * data->width][2];
            color[3] = data->accumulated_data[x + y * data->width][3];
            
            color[0] *= inv_frame_index;
            color[1] *= inv_frame_index;
            color[2] *= inv_frame_index;
            color[3] *= inv_frame_index;
            
            // clamp to 255
            color[0] = dm_clamp(color[0], 0, 1);
            color[1] = dm_clamp(color[1], 0, 1);
            color[2] = dm_clamp(color[2], 0, 1);
            color[3] = dm_clamp(color[3], 0, 1);
            
            data->image_data[x + y * data->width] = vec4_to_uint32(color);
        }
    }
    
    return NULL;
}

void create_image_mt(application_data* app_data, dm_context* context)
{
    const uint32_t y_incr    = app_data->image.h / NUM_TASKS;
    const size_t   data_size = sizeof(uint32_t) * y_incr * app_data->image.w;
    const size_t   acc_size  = sizeof(dm_vec4) * y_incr * app_data->image.w;
    const size_t   ray_size  = sizeof(dm_vec4) * y_incr * app_data->image.w;
    
    uint32_t offset;
    
    if(app_data->image.frame_index==1)
    {
        dm_memzero(app_data->image.accumulated_data, sizeof(dm_vec4) * app_data->image.w * app_data->image.h);
    }
    
    for(uint32_t i=0; i<NUM_TASKS; i++)
    {
        app_data->thread_data[i].y_incr      = y_incr;
        app_data->thread_data[i].width       = app_data->image.w;
        app_data->thread_data[i].spheres     = &app_data->spheres;
        app_data->thread_data[i].materials   = &app_data->materials;
        app_data->thread_data[i].frame_index = app_data->image.frame_index;
        app_data->thread_data[i].b           = app_data->b;
        
        app_data->thread_data[i].sky_color[0] = app_data->sky_color[0];
        app_data->thread_data[i].sky_color[1] = app_data->sky_color[1];
        app_data->thread_data[i].sky_color[2] = app_data->sky_color[2];
        app_data->thread_data[i].sky_color[3] = app_data->sky_color[3];
        
        app_data->thread_data[i].ray_pos[0] = app_data->camera.pos[0];
        app_data->thread_data[i].ray_pos[1] = app_data->camera.pos[1];
        app_data->thread_data[i].ray_pos[2] = app_data->camera.pos[2];
        
        app_data->thread_data[i].rays_processed = 0;
        
        offset = y_incr * i * app_data->image.w;
        
        // generate "image" arrays
        if(!app_data->thread_data[i].image_data)
        {
            app_data->thread_data[i].image_data       = dm_alloc(data_size);
            app_data->thread_data[i].accumulated_data = dm_alloc(acc_size);
            app_data->thread_data[i].random_numbers   = dm_alloc(data_size);
        }
        else
        {
            app_data->thread_data[i].image_data       = dm_realloc(app_data->thread_data[i].image_data, data_size);
            app_data->thread_data[i].accumulated_data = dm_realloc(app_data->thread_data[i].accumulated_data, acc_size);
            app_data->thread_data[i].random_numbers   = dm_realloc(app_data->thread_data[i].random_numbers, data_size);
        }
        
        // copy over data
        void* src = app_data->image.data + offset;
        dm_memcpy(app_data->thread_data[i].image_data, src, data_size);
        
        src = app_data->image.accumulated_data + offset;
        dm_memcpy(app_data->thread_data[i].accumulated_data, src, acc_size);
        
        src = app_data->image.random_numbers + offset;
        dm_memcpy(app_data->thread_data[i].random_numbers, src, data_size);
        
        // ray directions
        if(!app_data->thread_data[i].ray_dirs)
        {
            app_data->thread_data[i].ray_dirs = dm_alloc(ray_size);
        }
        else
        {
            app_data->thread_data[i].ray_dirs = dm_realloc(app_data->thread_data[i].ray_dirs, ray_size);
        }
        dm_memcpy(app_data->thread_data[i].ray_dirs, app_data->ray_dirs + offset, ray_size);
        
        ///////////////////////////
        app_data->tasks[i].func = image_mt_func;
        app_data->tasks[i].args = &app_data->thread_data[i];
    }
    
    // task submission
    for(uint32_t i=0; i<NUM_TASKS; i++)
    {
        dm_threadpool_submit_task(&app_data->tasks[i], &app_data->threadpool);
    }
    
    dm_threadpool_wait_for_completion(&app_data->threadpool);
    
    // copy new image data back over
    for(uint32_t i=0; i<NUM_TASKS; i++)
    {
        offset = y_incr * i * app_data->image.w;
        
        dm_memcpy(app_data->image.data + offset, app_data->thread_data[i].image_data, data_size);
        dm_memcpy(app_data->image.accumulated_data + offset, app_data->thread_data[i].accumulated_data, acc_size);
        
        app_data->rays_processed += app_data->thread_data[i].rays_processed;
    }
}

/*******************
FRAMEWORK INTERFACE
*********************/
void dm_application_setup(dm_context_init_packet* init_packet)
{
    init_packet->window_width  = 1280;
    init_packet->window_height = 720;
}

bool dm_application_init(dm_context* context)
{
    context->app_data = dm_alloc(sizeof(application_data));
    application_data* app_data = context->app_data;
    
    // vb is just a quad
    typedef struct vertex_t
    {
        float pos[2];
        float uv[2];
    } vertex;
    
    vertex vertices[] = {
        // triangle 1
        { { -1,-1 }, { 0,0 } },
        { {  1,-1 }, { 1,0 } },
        { {  1, 1 }, { 1,1 } },
        
        // triangle 2
        { {  1, 1 }, { 1,1 } },
        { { -1, 1 }, { 0,1 } },
        { { -1,-1 }, { 0,0 } },
    };
    
    if(!dm_renderer_create_static_vertex_buffer(vertices, sizeof(vertices), sizeof(vertex), &app_data->handles.vb, context)) return false;
    
    // pipeline and shader
    dm_pipeline_desc p_desc = dm_renderer_default_pipeline();
    
    dm_shader_desc s_desc = {
#ifdef DM_DIRECTX
        .vertex="assets/shaders/quad_vertex.fxc",
        .pixel="assets/shaders/quad_pixel.fxc",
#elif defined(DM_OPENGL)
        .vertex="assets/shaders/quad_vertex.glsl",
        .pixel="assets/shaders/quad_pixel.glsl",
#elif defined(DM_METAL)
        .master="assets/shaders/quad.metallib",
#endif
        .vb={ app_data->handles.vb },
        .vb_count=1
    };
    
#ifdef DM_METAL
    strcpy(s_desc.vertex, "vertex_main");
    strcpy(s_desc.pixel,  "fragment_main");
#endif
    
    dm_vertex_attrib_desc attrib_descs[] = {
        { .name="POSITION", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(vertex), .offset=offsetof(vertex, pos), .count=2, .index=0, .normalized=false },
        { .name="TEX_COORDS", .data_t=DM_VERTEX_DATA_T_FLOAT, .attrib_class=DM_VERTEX_ATTRIB_CLASS_VERTEX, .stride=sizeof(vertex), .offset=offsetof(vertex, uv), .count=2, .index=0, .normalized=false },
    };
    
    if(!dm_renderer_create_shader_and_pipeline(s_desc, p_desc, attrib_descs, 2, &app_data->handles.shader, &app_data->handles.pipeline, context)) return false;
    
    // texture we write to
    app_data->image.w = context->platform_data.window_data.width;
    app_data->image.h = context->platform_data.window_data.height;
    
    app_data->image.data_size = sizeof(uint32_t) * app_data->image.w * app_data->image.h;
    
    app_data->image.data = dm_alloc(app_data->image.data_size);
    app_data->image.accumulated_data = dm_alloc(sizeof(dm_vec4) * app_data->image.w * app_data->image.h);
    app_data->image.frame_index = 1;
    
    app_data->image.random_numbers = dm_alloc(sizeof(uint32_t) * app_data->image.w * app_data->image.h);
    
    if(!dm_renderer_create_dynamic_texture(app_data->image.w, app_data->image.h, 4, app_data->image.data, "image_texture", &app_data->handles.texture, context)) return false;
    
    // camera
    float camera_p[] = { 0,0,5 };
    float camera_f[] = { 0,0,-1 };
    camera_init(camera_p, camera_f, 0.01f, 100.0f, 75.0f, app_data->image.w, app_data->image.h, 5.0f, 0.1f, &app_data->camera);
    
    recreate_rays(app_data);
    
    for(uint32_t i=0; i<MAX_SPHERE_COUNT; i++)
    {
        app_data->spheres.x[i] = FLT_MAX;
        app_data->spheres.y[i] = FLT_MAX;
        app_data->spheres.z[i] = FLT_MAX;
        
        app_data->spheres.radius[i] = FLT_MAX;
        app_data->spheres.radius_2[i] = FLT_MAX;
    }
    
    // threadool
    if(!dm_threadpool_create("ray_tracer", 4, &app_data->threadpool)) return false;
    
#if 1
    // materials
    for(uint32_t i=0; i<MAX_MATERIAL_COUNT; i++)
    {
        app_data->materials.albedo_r[i] = dm_random_float(context);
        app_data->materials.albedo_g[i] = dm_random_float(context);
        app_data->materials.albedo_b[i] = dm_random_float(context);
        
        app_data->materials.roughness[i] = dm_random_float(context);
        app_data->materials.metallic[i]  = dm_random_float(context);
        
        if(dm_random_float(context) < 0.8f) continue;
        
        app_data->materials.emission_r[i] = app_data->materials.albedo_r[i];
        app_data->materials.emission_g[i] = app_data->materials.albedo_g[i];
        app_data->materials.emission_b[i] = app_data->materials.albedo_b[i];
        
        app_data->materials.emission_power[i] = dm_random_float_range(0.5f,10, context);;
    }
    
#define NUM_SPHERES 32
    // spheres
    for(uint32_t i=0; i<NUM_SPHERES; i++)
    {
        make_random_sphere(app_data, context);
    }
#else
    {
        app_data->materials.albedo_r[0] = 1;
        app_data->materials.albedo_b[0] = 1;
    }
    
    {
        app_data->materials.albedo_b[1] = 1;
    }
    
    {
        app_data->materials.albedo_r[2] = 0.7f;
        app_data->materials.albedo_g[2] = 0.2f;
        app_data->materials.albedo_b[2] = 0.3f;
        
        app_data->materials.emission_r[2] = 0.7f;
        app_data->materials.emission_g[2] = 0.2f;
        app_data->materials.emission_b[2] = 0.3f;
        
        app_data->materials.emission_power[2] = 20.0f;
    }
    
    {
        app_data->materials.albedo_g[3] = 1;
    }
    
    app_data->materials.count = 4;
    
    // pink sphere
    {
        app_data->spheres.x[0] = 0;
        app_data->spheres.y[0] = 0;
        app_data->spheres.z[0] = 0;
        
        app_data->spheres.radius[0] = 1;
        app_data->spheres.radius_2[0] = 1;
        
        app_data->spheres.material_id[0] = 0;
    }
    
    // blue sphere
    {
        app_data->spheres.x[1] = 0;
        app_data->spheres.y[1] = -11;
        app_data->spheres.z[1] = 0;
        
        app_data->spheres.radius[1] = 10;
        app_data->spheres.radius_2[1] = 100;
        
        app_data->spheres.material_id[1] = 1;
    }
    
    // green sphere
    {
        app_data->spheres.x[3] = 2;
        app_data->spheres.y[3] = 2;
        app_data->spheres.z[3] = 2;
        
        app_data->spheres.radius[3] = 2;
        app_data->spheres.radius_2[3] = 4;
        
        app_data->spheres.material_id[3] = 3;
    }
    
    // emitter
    {
        app_data->spheres.x[2] = -20;
        app_data->spheres.y[2] = 5;
        app_data->spheres.z[2] = -20;
        
        app_data->spheres.radius[2] = 10;
        app_data->spheres.radius_2[2] = 100;
        
        app_data->spheres.material_id[2] = 2;
    }
    
    app_data->spheres.count = 4;
#endif
    
    // bvh
    bvh_init(app_data->spheres.count, &app_data->b);
    bvh_populate(&app_data->spheres, &app_data->b);
    
    // misc
    app_data->sky_color[0] = 0.6f;
    app_data->sky_color[1] = 0.7f;
    app_data->sky_color[2] = 0.9f;
    app_data->sky_color[3] = 1;
    
    return true;
}

void dm_application_shutdown(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    for(uint32_t i=0; i<NUM_TASKS; i++)
    {
        dm_free(app_data->thread_data[i].ray_dirs);
        dm_free(app_data->thread_data[i].image_data);
        dm_free(app_data->thread_data[i].accumulated_data);
        dm_free(app_data->thread_data[i].random_numbers);
    }
    
    dm_threadpool_destroy(&app_data->threadpool);
    
    dm_free(app_data->ray_dirs);
    dm_free(app_data->image.accumulated_data);
    dm_free(app_data->image.data);
    dm_free(app_data->image.random_numbers);
    dm_free(context->app_data);
}

bool dm_application_update(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    // check image width and height
    const bool width_changed  = context->platform_data.window_data.width != app_data->image.w;
    const bool height_changed = context->platform_data.window_data.height != app_data->image.h;
    
    if(width_changed || height_changed) 
    {
        app_data->image.w = context->platform_data.window_data.width;
        app_data->image.h = context->platform_data.window_data.height;
        
        app_data->image.data_size = sizeof(uint32_t) * app_data->image.w * app_data->image.h;
        app_data->image.data = dm_realloc(app_data->image.data, app_data->image.data_size);
        
        app_data->image.accumulated_data = dm_realloc(app_data->image.accumulated_data, sizeof(dm_vec4) * app_data->image.w * app_data->image.h);
        
        app_data->image.random_numbers = dm_realloc(app_data->image.random_numbers, app_data->image.data_size);
        
        camera_resize(app_data->image.w, app_data->image.h, &app_data->camera, context);
    }
    
    // generate random numbers
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            app_data->image.random_numbers[x + y * app_data->image.w] = dm_random_uint32(context);
        }
    }
    
    dm_timer_start(&app_data->timer, context);
    const bool camera_updated = camera_update(&app_data->camera, context);
    if(width_changed || height_changed || camera_updated) recreate_rays(app_data);
    app_data->ray_creation_t = dm_timer_elapsed_ms(&app_data->timer, context);
    
    if(camera_updated) reset_frame_index(app_data);
    
    // update image
    dm_timer_start(&app_data->timer, context);
    
    app_data->rays_processed = 0;
    //create_image(app_data, context);
    create_image_mt(app_data, context);
    
    if(app_data->accumulate) app_data->image.frame_index++;
    else                     app_data->image.frame_index = 1;
    
    app_data->image_creation_t = dm_timer_elapsed_ms(&app_data->timer, context);
    
    /* If this generates a random image each frame, updating texture works
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            app_data->image.data[x + y * app_data->image.w] = dm_random_uint32(context);
            app_data->image.data[x + y * app_data->image.w] |= 0xFF000000;
        }
    }
     */
    
    dm_render_command_update_texture(app_data->handles.texture, app_data->image.w, app_data->image.h, app_data->image.data, app_data->image.data_size, context);
    
    return true;
}

bool dm_application_render(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    dm_render_command_set_default_viewport(context);
    dm_render_command_clear(0,0,0,1, context);
    
    dm_render_command_bind_shader(app_data->handles.shader, context);
    dm_render_command_bind_pipeline(app_data->handles.pipeline, context);
    dm_render_command_bind_buffer(app_data->handles.vb, 0, context);
    dm_render_command_bind_texture(app_data->handles.texture, 0, context);
    dm_render_command_draw_arrays(0, 6, context);
    
    
    ///////////////////// IMGUI
    dm_imgui_nuklear_context* imgui_nk_ctx = &context->imgui_context.internal_context;
    struct nk_context* ctx = &imgui_nk_ctx->ctx;
    
    if(nk_begin(ctx, "Ray Trace App", nk_rect(950,50, 300,450), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE | 
                NK_WINDOW_MINIMIZABLE | NK_WINDOW_TITLE))
    {
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_uint(ctx, "Sphere count", app_data->spheres.count);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Ray creation (ms)", app_data->ray_creation_t);
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_value_float(ctx, "Image creation (ms)", app_data->image_creation_t);
        
        nk_layout_row_dynamic(ctx, 30, 1);
        const float num_rays = (float)app_data->rays_processed / app_data->image_creation_t / 1e3f;
        nk_value_float(ctx, "Rays processed (millions per second)", num_rays);
        
        nk_layout_row_dynamic(ctx, 30, 2);
        nk_bool acc = app_data->accumulate;
        nk_checkbox_label(ctx, "Accumulate", &acc);
        if(nk_button_label(ctx, "Reset")) reset_frame_index(app_data);
        
        app_data->accumulate = acc;
        
        // emitter
        struct nk_colorf c;
        c.r = app_data->materials.albedo_r[2];
        c.g = app_data->materials.albedo_g[2];
        c.b = app_data->materials.albedo_b[2];
        
        nk_layout_row_dynamic(ctx, 120, 1);
        c = nk_color_picker(ctx, c, NK_RGB);
        
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Emission Power", 0, &app_data->materials.emission_power[2], 30, 0.1f, 0.1f);
        
        app_data->materials.albedo_r[2] = c.r;
        app_data->materials.albedo_g[2] = c.g;
        app_data->materials.albedo_b[2] = c.b;
        
        app_data->materials.emission_r[2] = c.r;
        app_data->materials.emission_g[2] = c.g;
        app_data->materials.emission_b[2] = c.b;
        
        nk_layout_row_dynamic(ctx, 20, 3);
        nk_property_float(ctx, "X", -30, &app_data->spheres.x[2], 30, 0.1f, 0.1f);
        nk_property_float(ctx, "Y", -30, &app_data->spheres.y[2], 30, 0.1f, 0.1f);
        nk_property_float(ctx, "Z", -30, &app_data->spheres.z[2], 30, 0.1f, 0.1f);
        
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Radius", 10, &app_data->spheres.radius[2], 50, 0.1f, 0.1f);
        
        app_data->spheres.radius_2[2] = app_data->spheres.radius[2] * app_data->spheres.radius[2];
    }
    nk_end(ctx);
    /////////////////////////////////
    
    return true;
}
