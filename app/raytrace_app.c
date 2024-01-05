#include "dm.h"
#include "camera.h"

#include <float.h>
#include <assert.h>

/************
 GLOBALS
 **************/
#define NUM_TASKS 16

#define DIMENSION 20

#define SAMPLES           1
#define SAMPLES_PER_PIXEL 4
#define BOUNCES           50

#define RAY_MIN 0.001f

static const float sample_inv = 1.0f / SAMPLES;

#define MT_IMAGE
//#define RANDOM_SCENE

#ifdef RANDOM_SCENE
    #define MATERIAL_COUNT 100
    #define OBJECT_COUNT   32
#else
    #define MATERIAL_COUNT 4
    #define OBJECT_COUNT   4
#endif

#define BVH_NODE_COUNT    (OBJECT_COUNT * 2 - 1)
#define BVH_INVALID_INDEX BVH_NODE_COUNT

/****************************************************/

typedef enum object_shape_t
{
    OBJECT_SHAPE_SPHERE,
    OBJECT_SHAPE_BOX,
    OBJECT_SHAPE_TRIANGLE,
    OBJECT_SHAPE_UNKNOWN
} object_shape;

typedef enum material_type_t
{
    MATERIAL_TYPE_LAMBERT,
    MATERIAL_TYPE_METAL,
    MATERIAL_TYPE_DIALECTIC,
    MATERIAL_TYPE_EMISSION,
    MATERIAL_TYPE_UNKNOWN
} material_type;

typedef struct ray_t
{
    dm_vec3 origin, direction;
    
    // hit data
    dm_vec3  hit_pos, hit_normal;
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
    bool     resized;
    
    uint32_t* data;
    dm_vec4*  accumulated_data;
    uint32_t* random_numbers;
} app_image;

typedef struct material_data_t
{
    union
    {
        DM_ALIGN(16) float albedo_r[MATERIAL_COUNT];
        DM_ALIGN(16) float emission_r[MATERIAL_COUNT];
    };
    
    union
    {
        DM_ALIGN(16) float albedo_g[MATERIAL_COUNT];
        DM_ALIGN(16) float emission_g[MATERIAL_COUNT];
    };
    
    union
    {
        DM_ALIGN(16) float albedo_b[MATERIAL_COUNT];
        DM_ALIGN(16) float emission_b[MATERIAL_COUNT];
    };
    
    // type parameters
    union
    {
        DM_ALIGN(16) float roughness[MATERIAL_COUNT];
        DM_ALIGN(16) float metallic[MATERIAL_COUNT];
        DM_ALIGN(16) float index_refraction[MATERIAL_COUNT];
        DM_ALIGN(16) float emission_power[MATERIAL_COUNT];
    };
    
    DM_ALIGN(16) material_type type[MATERIAL_COUNT];
} material_data;

typedef struct object_data_t
{
    DM_ALIGN(16) float x[OBJECT_COUNT];
    DM_ALIGN(16) float y[OBJECT_COUNT];
    DM_ALIGN(16) float z[OBJECT_COUNT];
    
    union
    {
        DM_ALIGN(16) float radius[OBJECT_COUNT];
        DM_ALIGN(16) float extent_x[OBJECT_COUNT];
    };
    
    union
    {
        DM_ALIGN(16) float radius_2[OBJECT_COUNT];
        DM_ALIGN(16) float extent_y[OBJECT_COUNT];
    };
    
    union
    {
        DM_ALIGN(16) float extent_z[OBJECT_COUNT];
    };
    
    DM_ALIGN(16) float aabb_min_x[OBJECT_COUNT];
    DM_ALIGN(16) float aabb_min_y[OBJECT_COUNT];
    DM_ALIGN(16) float aabb_min_z[OBJECT_COUNT];
    
    DM_ALIGN(16) float aabb_max_x[OBJECT_COUNT];
    DM_ALIGN(16) float aabb_max_y[OBJECT_COUNT];
    DM_ALIGN(16) float aabb_max_z[OBJECT_COUNT];
    
    DM_ALIGN(16) uint32_t material_id[OBJECT_COUNT];
    DM_ALIGN(16) object_shape shape[OBJECT_COUNT];
} object_data;

typedef struct bvh_node_t
{
    dm_vec3 aabb_min;
    dm_vec3 aabb_max;
    
    uint32_t start, count;
    
    uint32_t left_child;
} bvh_node;

typedef struct bvh_t
{
    bvh_node nodes[BVH_NODE_COUNT];
    uint32_t indices[OBJECT_COUNT];
} bvh;

typedef struct mt_data_t
{
    uint32_t y_incr, width;
    dm_vec4  sky_color;
    dm_vec3  ray_pos;
    uint32_t frame_index;
    
    uint32_t rays_processed;
    
    bool samp;
    
    bvh      b;
    
    object_data*   objects;
    material_data* materials;
    
    uint32_t*    image_data;
    dm_vec4*     accumulated_data;
    uint32_t*    random_numbers;
    
    dm_vec4*     ray_dirs;
} mt_data;

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
    bool sample;
    
    // ray directions
    dm_vec4* ray_dirs;
    
    // objects
    object_data objects;
    
    // materials
    material_data materials;
    
    // bvh
    bvh b;
} application_data;

/*************
INTERSECTIONS
***************/
DM_INLINE
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

DM_INLINE
void ray_intersects_sphere(ray* r, const uint32_t index, const float sphere_x, const float sphere_y, const float sphere_z, const float radius, const float sphere_r2)
{
    const dm_vec3 origin = {
        r->origin[0] - sphere_x,
        r->origin[1] - sphere_y,
        r->origin[2] - sphere_z
    };
    
    const float half_b = dm_vec3_dot(origin, r->direction);
    const float c      = dm_vec3_dot(origin, origin) - sphere_r2;
    
    const float dis       =   half_b * half_b -  c;
    const float closest_t = -(half_b + dm_sqrtf(dis));
    
    if (dis < 0 || closest_t > r->hit_distance || closest_t < 0 || closest_t < RAY_MIN) return;
    
    const float f = 1.0f / radius;
    
    r->hit_distance = closest_t;
    r->hit_index    = index;
    
    dm_vec3_scale(r->direction, r->hit_distance, r->hit_pos);
    dm_vec3_add_vec3(r->hit_pos, r->origin, r->hit_pos);
    
    r->hit_normal[0] = r->hit_pos[0] - sphere_x;
    r->hit_normal[1] = r->hit_pos[1] - sphere_y;
    r->hit_normal[2] = r->hit_pos[2] - sphere_z;
    dm_vec3_scale(r->hit_normal, f, r->hit_normal);
    dm_vec3_norm(r->hit_normal, r->hit_normal);
}

void ray_intersect_bvh(ray* r, uint32_t index, object_data* objects, bvh* b)
{
    if(!ray_intersects_aabb(r, b->nodes[index].aabb_min, b->nodes[index].aabb_max)) return;
    
    if(b->nodes[index].count > 0)
    {
        uint32_t o_index;
        for(uint32_t i=0; i<b->nodes[index].count; i++)
        {
            o_index = b->indices[b->nodes[index].start + i];
            
            switch(objects->shape[o_index])
            {
                case OBJECT_SHAPE_SPHERE:
                    ray_intersects_sphere(r, o_index, objects->x[o_index], objects->y[o_index], objects->z[o_index], objects->radius[o_index], objects->radius_2[o_index]);
                    break;
                    
                default:
                    DM_LOG_ERROR("Unknown object shape!");
                    return;
            }
        }
    }
    else
    {
        ray_intersect_bvh(r, b->nodes[index].left_child, objects, b);
        ray_intersect_bvh(r, b->nodes[index].left_child + 1, objects, b);
    }
}

/***
BVH
*****/
void bvh_update_node_bounds(uint32_t index, object_data* objects, bvh* b)
{
    float aabb_min_x, aabb_min_y, aabb_min_z;
    float aabb_max_x, aabb_max_y, aabb_max_z;
    
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    
    aabb_min_x = aabb_min_y = aabb_min_z =  FLT_MAX;
    aabb_max_x = aabb_max_y = aabb_max_z = -FLT_MAX;
    
    uint32_t s_index;
    for(uint32_t i=0; i<b->nodes[index].count; i++)
    {
        s_index = b->indices[i + b->nodes[index].start];
        
        min_x = objects->aabb_min_x[s_index] + objects->x[s_index];
        min_y = objects->aabb_min_y[s_index] + objects->y[s_index];
        min_z = objects->aabb_min_z[s_index] + objects->z[s_index];
        
        max_x = objects->aabb_max_x[s_index] + objects->x[s_index];
        max_y = objects->aabb_max_y[s_index] + objects->y[s_index];
        max_z = objects->aabb_max_z[s_index] + objects->z[s_index];
        
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

void bvh_node_subdivide(uint32_t current_index, uint32_t* global_index, object_data* objects, bvh* b)
{
    bvh_node* current_node = &b->nodes[current_index];
    
    if(b->nodes[current_index].count <= 2) return;
    
    dm_vec3 extents;
    dm_vec3_sub_vec3(current_node->aabb_max, current_node->aabb_min, extents);
    int axis = 0;
    if(extents[1] > extents[0]) axis = 1;
    if(extents[2] > extents[axis]) axis = 2;
    
    float split_pos = current_node->aabb_min[axis] + extents[axis] * 0.5f;
    
    int i = current_node->start;
    int j = i + current_node->count - 1;
    
    while(i <= j)
    {
        bool swap = false;
        switch(axis)
        {
            case 0:
            if(objects->x[b->indices[i]] < split_pos) i++;
            else swap = true;
            break;
            
            case 1:
            if(objects->y[b->indices[i]] < split_pos) i++;
            else swap = true;
            break;
            
            case 2:
            if(objects->z[b->indices[i]] < split_pos) i++;
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
    
    int left_count = i - current_node->start;
    if(left_count==0 || left_count==current_node->count) return;
    
    const uint32_t left_child  = ++(*global_index);
    const uint32_t right_child = ++(*global_index);
    
    current_node->left_child = left_child;
    
    b->nodes[left_child].start = current_node->start;
    b->nodes[left_child].count = left_count;
    
    b->nodes[right_child].start = i;
    b->nodes[right_child].count = current_node->count - left_count;
    
    current_node->count = 0;
    
    bvh_update_node_bounds(left_child, objects, b);
    bvh_node_subdivide(left_child, global_index, objects, b);
    
    bvh_update_node_bounds(right_child, objects, b);
    bvh_node_subdivide(right_child, global_index, objects, b);
}

void bvh_populate(object_data* objects, bvh* b)
{
    for(uint32_t i=0; i<OBJECT_COUNT; i++)
    {
        b->indices[i] = i;
    }
    
    for(uint32_t i=0; i<BVH_NODE_COUNT; i++)
    {
        b->nodes[i].left_child = BVH_INVALID_INDEX;
    }
    
    b->nodes[0].start = 0;
    b->nodes[0].count = OBJECT_COUNT;
    
    uint32_t global_index = 0;
    
    bvh_update_node_bounds(0, objects, b);
    bvh_node_subdivide(0, &global_index, objects, b);
}

/*******
HELPERS
*********/
void make_random_sphere(application_data* app_data, uint32_t index, dm_context* context)
{
    object_data* objects = &app_data->objects;
    
    const float x = dm_random_float_range(-DIMENSION,DIMENSION,context);
    const float y = dm_random_float_range(-DIMENSION,DIMENSION,context);
    const float z = dm_random_float_range(-DIMENSION,DIMENSION,context);
    
    objects->x[index] = x;
    objects->y[index] = y;
    objects->z[index] = z;
    
    const float radius = dm_random_float_range(0.1f,3.0f, context);
    
    objects->radius[index]   = radius;
    objects->radius_2[index] = radius * radius;
    
    const float r = dm_random_float(context);
    const float g = dm_random_float(context);
    const float b = dm_random_float(context);
    
    objects->material_id[index] = (uint32_t)dm_random_float_range(0,MATERIAL_COUNT, context);
    
    objects->aabb_min_x[index] = x - radius;
    objects->aabb_min_y[index] = y - radius;
    objects->aabb_min_z[index] = z - radius;
    
    objects->aabb_max_x[index] = x + radius;
    objects->aabb_max_y[index] = y + radius;
    objects->aabb_max_z[index] = z + radius;
}

DM_INLINE
void reset_frame_index(application_data* app_data)
{
    app_data->image.frame_index = 1;
}

DM_INLINE
uint32_t vec4_to_uint32(const float vec[4])
{
    const uint8_t r = (uint8_t)(255.0f * vec[0]);
    const uint8_t g = (uint8_t)(255.0f * vec[1]);
    const uint8_t b = (uint8_t)(255.0f * vec[2]);
    const uint8_t a = (uint8_t)(255.0f * vec[3]);
    
    uint32_t result = (a << 24) | (b << 16) | (g << 8) | r;
    return result;
}

DM_INLINE
uint32_t pcg_hash(uint32_t input)
{
    uint32_t state = input * 747796405u + 2891336453u;
    uint32_t word  = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

DM_INLINE
float random_float(uint32_t* seed)
{
    *seed = pcg_hash(*seed);
    
    return (float)*seed / (float)UINT_MAX;
}

void trace_ray(ray* r, object_data* objects, bvh* b)
{
    r->hit_distance = FLT_MAX;
    r->hit_index    = UINT_MAX;
    
    ray_intersect_bvh(r, 0, objects, b);
}

void per_pixel(uint32_t x, uint32_t y, dm_vec4 color, dm_vec4 sky_color, dm_vec3 pos, dm_vec3 dir, uint32_t seed, bool sample, uint32_t* ray_count, object_data* objects, material_data* materials, bvh* b)
{
    *ray_count = *ray_count + 1;
    
    ray r;
    
    r.origin[0] = pos[0];
    r.origin[1] = pos[1];
    r.origin[2] = pos[2];
    
    r.direction[0] = dir[0];
    r.direction[1] = dir[1];
    r.direction[2] = dir[2];
    
    float light_r = 1;
    float light_g = 1;
    float light_b = 1;
    
    uint32_t mat_id, o_index;
    dm_vec3 offset;
    
    dm_vec3 attenuation = { 1,1,1 };
    
    for(uint32_t i=0; i<BOUNCES; i++)
    {
        seed += i;
        
        trace_ray(&r, objects, b);
        if(r.hit_index==UINT_MAX)
        {
            dm_vec3_mul_vec3(attenuation, sky_color, attenuation);
            
            light_r *= attenuation[0];
            light_g *= attenuation[1];
            light_b *= attenuation[2];
            
            break;
        }
        
        o_index = r.hit_index;
        mat_id  = objects->material_id[o_index];
    
        bool back_face = dm_vec3_same_direction(r.hit_normal, r.direction);
        if(back_face)    dm_vec3_negate(r.hit_normal, r.hit_normal);
        
        if(sample)
        {
            switch(materials->type[mat_id])
            {
                case MATERIAL_TYPE_LAMBERT:
                {
                    r.direction[0] = random_float(&seed) * 2.f - 1.f;
                    r.direction[1] = random_float(&seed) * 2.f - 1.f;
                    r.direction[2] = random_float(&seed) * 2.f - 1.f;
                    dm_vec3_norm(r.direction, r.direction);
                    dm_vec3_scale(r.direction, materials->roughness[mat_id], r.direction);
                    dm_vec3_add_vec3(r.hit_normal, r.direction, r.direction);
                    dm_vec3_norm(r.direction, r.direction);
                } break;
                    
                case MATERIAL_TYPE_METAL:
                {
                    dm_vec3_reflect(r.direction, r.hit_normal, r.direction);
                    dm_vec3 fuzziness;
                    fuzziness[0] = random_float(&seed) * 2.0f - 1.f;
                    fuzziness[1] = random_float(&seed) * 2.0f - 1.f;
                    fuzziness[2] = random_float(&seed) * 2.0f - 1.f;
                    dm_vec3_norm(fuzziness, fuzziness);
                    dm_vec3_scale(fuzziness, materials->metallic[mat_id], fuzziness);
                    dm_vec3_add_vec3(r.direction, fuzziness, r.direction);
                    dm_vec3_norm(r.direction, r.direction);
                } break;
                    
                case MATERIAL_TYPE_DIALECTIC:
                {
                    float ir = materials->index_refraction[mat_id];
                    if(!back_face) ir = 1.0f / ir;
                    
                    float cos_theta, sin_theta;
                    dm_vec3 neg_d;
                    
                    dm_vec3_negate(r.direction, neg_d);
                    cos_theta = dm_vec3_dot(neg_d, r.hit_normal);
                    cos_theta = DM_MIN(cos_theta, 1);
                    
                    sin_theta = cos_theta * cos_theta;
                    sin_theta = 1 - sin_theta;
                    sin_theta = dm_sqrtf(sin_theta);
                    
                    const bool cannot = ir * sin_theta > 0;
                    
                    // schlick approx
                    float r0 = (1-ir) / (1+ir);
                    r0 *= r0;
                    r0 += (1-r0)*dm_powf(1-cos_theta, 5);
                    
                    if(!cannot || (r0 > random_float(&seed))) dm_vec3_reflect(r.direction, r.hit_normal, r.direction);
                    else                                      dm_vec3_refract(r.direction, r.hit_normal, ir, r.direction);
                    
                    dm_vec3_norm(r.direction, r.direction);
                } break;
                    
                default:
                    DM_LOG_ERROR("Unknown material type!");
                    return;
            }
        }
        else
        {
            r.direction[0] = r.hit_normal[0];
            r.direction[1] = r.hit_normal[1];
            r.direction[2] = r.hit_normal[2];
        }
        
        switch(materials->type[mat_id])
        {
            case MATERIAL_TYPE_LAMBERT:
            case MATERIAL_TYPE_METAL:
            default:
                attenuation[0] *= materials->albedo_r[mat_id];
                attenuation[1] *= materials->albedo_g[mat_id];
                attenuation[2] *= materials->albedo_b[mat_id];
                break;
                
            case MATERIAL_TYPE_DIALECTIC:
                attenuation[0] = attenuation[1] = attenuation[2] = 1;
                break;
        }
        
        light_r *= attenuation[0];
        light_g *= attenuation[1];
        light_b *= attenuation[2];
        
        r.origin[0] = r.hit_pos[0]; r.origin[1] = r.hit_pos[1]; r.origin[2] = r.hit_pos[2];
    }
    
    color[0] = light_r;
    color[1] = light_g;
    color[2] = light_b;
    color[3] = 1;
}

void recreate_rays(application_data* app_data)
{
    if(app_data->image.resized && app_data->ray_dirs)
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
    
    dm_simd_float w_mm, target_mm, row1,row2,row3,row4;
    dm_simd_float target_x_mm, target_y_mm, target_z_mm;
    dm_simd_float coords_mm, coords_x_mm, coords_y_mm, coords_z_mm, coords_w_mm;
    
    //////////// camera matrices are constant, so do them outside
    dm_simd_float proj_row1, proj_row2, proj_row3, proj_row4;
    dm_simd_float view_row1, view_row2, view_row3, view_row4;
    
#ifdef DM_SIMD_ARM
    dm_mat4 temp;
    dm_mat4_transpose(app_data->camera.inv_proj, temp);
    
    proj_row1 = dm_simd_load_float(temp[0]);
    proj_row2 = dm_simd_load_float(temp[1]);
    proj_row3 = dm_simd_load_float(temp[2]);
    proj_row4 = dm_simd_load_float(temp[3]);
    
    dm_mat4_transpose(app_data->camera.inv_view, temp);
    
    view_row1 = dm_simd_load_float(temp[0]);
    view_row2 = dm_simd_load_float(temp[1]);
    view_row3 = dm_simd_load_float(temp[2]);
    view_row4 = dm_simd_load_float(temp[3]);
    
#elif defined(DM_SIMD_X86)
    proj_row1 = dm_simd_load_float(app_data->camera.inv_proj[0]);
    proj_row2 = dm_simd_load_float(app_data->camera.inv_proj[1]);
    proj_row3 = dm_simd_load_float(app_data->camera.inv_proj[2]);
    proj_row4 = dm_simd_load_float(app_data->camera.inv_proj[3]);
    
    dm_simd_transpose_mat4(&proj_row1, &proj_row2, &proj_row3, &proj_row4);
    
    view_row1 = dm_simd_load_float(app_data->camera.inv_view[0]);
    view_row2 = dm_simd_load_float(app_data->camera.inv_view[1]);
    view_row3 = dm_simd_load_float(app_data->camera.inv_view[2]);
    view_row4 = dm_simd_load_float(app_data->camera.inv_view[3]);
    
    dm_simd_transpose_mat4(&view_row1, &view_row2, &view_row3, &view_row4);
#endif
    
    //////////////////////
    coords_z_mm = dm_simd_set1_float(1);
    coords_w_mm = dm_simd_set1_float(1);
    
    const dm_simd_float ones     = dm_simd_set1_float(1.f);
    const dm_simd_float neg_ones = dm_simd_set1_float(-1.0f);
    const dm_simd_float twos     = dm_simd_set1_float(2.0f);
    
    dm_simd_float mag;
    
    for(uint32_t y=0; y<app_data->image.h; y++)
    {
        index = y * app_data->image.w;
        
        coords_y_mm = dm_simd_set1_float((float)y);
        coords_y_mm = dm_simd_mul_float(coords_y_mm, dm_simd_set1_float(height_f_inv));
        coords_y_mm = dm_simd_mul_float(coords_y_mm, twos);
        coords_y_mm = dm_simd_add_float(coords_y_mm, neg_ones);
        
        for(uint32_t x=0; x<app_data->image.w; x++)
        {
            /////////////////
            coords_x_mm = dm_simd_set1_float((float)x);
            coords_x_mm = dm_simd_mul_float(coords_x_mm, dm_simd_set1_float(width_f_inv));
            coords_x_mm = dm_simd_mul_float(coords_x_mm, twos);
            coords_x_mm = dm_simd_add_float(coords_x_mm, neg_ones);
            
            target_mm = dm_simd_mul_float(coords_x_mm, proj_row1);
            target_mm = dm_simd_fmadd_float(coords_y_mm,proj_row2, target_mm);
            target_mm = dm_simd_add_float(target_mm, proj_row3);
            target_mm = dm_simd_add_float(target_mm, proj_row4);
            
            w_mm = dm_simd_broadcast_w_float(target_mm);
            target_mm = dm_simd_div_float(target_mm, w_mm);
            
            target_x_mm = dm_simd_broadcast_x_float(target_mm);
            target_y_mm = dm_simd_broadcast_y_float(target_mm);
            target_z_mm = dm_simd_broadcast_z_float(target_mm);
            
            mag = dm_simd_mul_float(target_x_mm, target_x_mm);
            mag = dm_simd_fmadd_float(target_y_mm,target_y_mm, mag);
            mag = dm_simd_fmadd_float(target_z_mm,target_z_mm, mag);
            mag = dm_simd_sqrt_float(mag);
            mag = dm_simd_div_float(ones, mag);
            
            target_x_mm = dm_simd_mul_float(target_x_mm, mag);
            target_y_mm = dm_simd_mul_float(target_y_mm, mag);
            target_z_mm = dm_simd_mul_float(target_z_mm, mag);
            
            target_mm = dm_simd_mul_float(target_x_mm, view_row1);
            target_mm = dm_simd_fmadd_float(target_y_mm,view_row2, target_mm);
            target_mm = dm_simd_fmadd_float(target_z_mm,view_row3, target_mm);
            
            dm_simd_store_float(app_data->ray_dirs[index++], target_mm);
            /////
        }
    }
}

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
                per_pixel(x,y, color, app_data->sky_color, app_data->camera.pos, dir, seed, app_data->sample, &app_data->rays_processed, &app_data->objects, &app_data->materials, &app_data->b);
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
                per_pixel(x,y, sample_color, data->sky_color, data->ray_pos, dir, seed, data->samp, &data->rays_processed, data->objects, data->materials, &data->b);
            }
            
            sample_color[0] *= sample_inv;
            sample_color[1] *= sample_inv;
            sample_color[2] *= sample_inv;
            
            data->accumulated_data[x + y * data->width][0] += dm_sqrtf(sample_color[0]);
            data->accumulated_data[x + y * data->width][1] += dm_sqrtf(sample_color[1]);
            data->accumulated_data[x + y * data->width][2] += dm_sqrtf(sample_color[2]);
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
    
    mt_data* thread_data = NULL;
    for(uint32_t i=0; i<NUM_TASKS; i++)
    {
        thread_data = &app_data->thread_data[i];
        
        thread_data->y_incr      = y_incr;
        thread_data->width       = app_data->image.w;
        
        thread_data->frame_index  = app_data->image.frame_index;
        thread_data->b            = app_data->b;
        
        thread_data->samp = app_data->sample;
        
        thread_data->sky_color[0] = app_data->sky_color[0];
        thread_data->sky_color[1] = app_data->sky_color[1];
        thread_data->sky_color[2] = app_data->sky_color[2];
        thread_data->sky_color[3] = app_data->sky_color[3];
        
        thread_data->ray_pos[0] = app_data->camera.pos[0];
        thread_data->ray_pos[1] = app_data->camera.pos[1];
        thread_data->ray_pos[2] = app_data->camera.pos[2];
        
        thread_data->rays_processed = 0;
        
        offset = y_incr * i * app_data->image.w;
        
        // generate "image" arrays
        if(!thread_data->image_data)
        {
            thread_data->image_data       = dm_alloc(data_size);
            thread_data->accumulated_data = dm_alloc(acc_size);
            thread_data->random_numbers   = dm_alloc(data_size);
        }
        else
        {
            thread_data->image_data       = dm_realloc(thread_data->image_data, data_size);
            thread_data->accumulated_data = dm_realloc(thread_data->accumulated_data, acc_size);
            thread_data->random_numbers   = dm_realloc(thread_data->random_numbers, data_size);
        }
        
        // copy over data
        void* src = app_data->image.data + offset;
        dm_memcpy(thread_data->image_data, src, data_size);
        
        src = app_data->image.accumulated_data + offset;
        dm_memcpy(thread_data->accumulated_data, src, acc_size);
        
        src = app_data->image.random_numbers + offset;
        dm_memcpy(thread_data->random_numbers, src, data_size);
        
        // ray directions
        if(!thread_data->ray_dirs)
        {
            thread_data->ray_dirs = dm_alloc(ray_size);
        }
        else
        {
            thread_data->ray_dirs = dm_realloc(thread_data->ray_dirs, ray_size);
        }
        dm_memcpy(thread_data->ray_dirs, app_data->ray_dirs + offset, ray_size);
        
        // objects and materials
        dm_memcpy(thread_data->objects, &app_data->objects, sizeof(object_data));
        dm_memcpy(thread_data->materials, &app_data->materials, sizeof(material_data));
        
        ///////////////////////////
        app_data->tasks[i].func = image_mt_func;
        app_data->tasks[i].args = thread_data;
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
    float camera_p[] = { 0,0,2 };
    float camera_f[] = { 0,0,-1 };
    camera_init(camera_p, camera_f, 0.01f, 100.0f, 75.0f, app_data->image.w, app_data->image.h, 5.0f, 0.1f, &app_data->camera);
    
    recreate_rays(app_data);
    
    for(uint32_t i=0; i<OBJECT_COUNT; i++)
    {
        app_data->objects.x[i] = FLT_MAX;
        app_data->objects.y[i] = FLT_MAX;
        app_data->objects.z[i] = FLT_MAX;
        
        app_data->objects.radius[i] = FLT_MAX;
        app_data->objects.radius_2[i] = FLT_MAX;
    }
    
    // threadool
    if(!dm_threadpool_create("ray_tracer", dm_get_available_processor_count(context), &app_data->threadpool)) return false;
    
#ifdef RANDOM_SCENE
    // materials
    for(uint32_t i=0; i<MATERIAL_COUNT; i++)
    {
        app_data->materials.albedo_r[i] = dm_random_float(context);
        app_data->materials.albedo_g[i] = dm_random_float(context);
        app_data->materials.albedo_b[i] = dm_random_float(context);
        
        material_type type = (int)dm_random_float_range(0, MATERIAL_TYPE_UNKNOWN, context);
        
        switch(type)
        {
            case MATERIAL_TYPE_LAMBERT:
                app_data->materials.roughness[i] = dm_random_float(context);
                break;
                
            case MATERIAL_TYPE_METAL:
                app_data->materials.metallic[i]  = dm_random_float(context);
                break;
                
            case MATERIAL_TYPE_DIALECTIC:
                app_data->materials.index_refraction[i] = dm_random_float_range(1, 2, context);
                break;
        
            default:
                DM_LOG_FATAL("Unknown material type! Shouldn't be here...");
                assert(false);
        };
        
        app_data->materials.type[i] = type;
        
        /*
        if(dm_random_float(context) < 0.8f) continue;
        
        app_data->materials.emission_r[i] = app_data->materials.albedo_r[i];
        app_data->materials.emission_g[i] = app_data->materials.albedo_g[i];
        app_data->materials.emission_b[i] = app_data->materials.albedo_b[i];
        
        app_data->materials.emission_power[i] = dm_random_float_range(0.5f,10, context);
         */
    }
    
    // spheres
    for(uint32_t i=0; i<OBJECT_COUNT; i++)
    {
        make_random_sphere(app_data, i, context);
    }
#else
    app_data->materials.albedo_r[0]  = 0.8f;
    app_data->materials.albedo_g[0]  = 0.8f;
    app_data->materials.albedo_b[0]  = 0.0f;
    app_data->materials.type[0]      = MATERIAL_TYPE_LAMBERT;
    app_data->materials.roughness[0] = 0.5f;
    
    app_data->materials.albedo_r[1]  = 0.1f;
    app_data->materials.albedo_g[1]  = 0.2f;
    app_data->materials.albedo_b[1]  = 0.5f;
    app_data->materials.type[1]      = MATERIAL_TYPE_LAMBERT;
    app_data->materials.roughness[1] = 0.5f;
    
    app_data->materials.albedo_r[2]         = 1.0f;
    app_data->materials.albedo_g[2]         = 1.0f;
    app_data->materials.albedo_b[2]         = 1.0f;
    app_data->materials.type[2]             = MATERIAL_TYPE_DIALECTIC;
    app_data->materials.index_refraction[2] = 1.5f;
    
    app_data->materials.albedo_r[3] = 0.8f;
    app_data->materials.albedo_g[3] = 0.6f;
    app_data->materials.albedo_b[3] = 0.2f;
    app_data->materials.type[3]     = MATERIAL_TYPE_METAL;
    app_data->materials.metallic[3] = 0;
    
    app_data->objects.x[0] = 0.0f;
    app_data->objects.y[0] = -100.5f;
    app_data->objects.z[0] = -1.0f;
    app_data->objects.shape[0] = OBJECT_SHAPE_SPHERE;
    app_data->objects.radius[0] = 100.0f;
    app_data->objects.radius_2[0] = 100.0f * 100.0f;
    app_data->objects.aabb_min_x[0] = -100.0f; app_data->objects.aabb_min_y[0] = -100.0f; app_data->objects.aabb_min_z[0] = -100.0f;
    app_data->objects.aabb_max_x[0] =  100.0f; app_data->objects.aabb_max_y[0] =  100.0f; app_data->objects.aabb_max_z[0] =  100.0f;
    app_data->objects.material_id[0] = 0;
    
    app_data->objects.x[1] = 0.0f;
    app_data->objects.y[1] = 0.0;
    app_data->objects.z[1] = -1.0f;
    app_data->objects.shape[1] = OBJECT_SHAPE_SPHERE;
    app_data->objects.radius[1] = 0.5f;
    app_data->objects.radius_2[1] = 0.5f * 0.5f;
    app_data->objects.aabb_min_x[1] = -0.5f; app_data->objects.aabb_min_y[1] = -0.5f; app_data->objects.aabb_min_z[1] = -0.5f;
    app_data->objects.aabb_max_x[1] =  0.5f; app_data->objects.aabb_max_y[1] =  0.5f; app_data->objects.aabb_max_z[1] =  0.5f;
    app_data->objects.material_id[1] = 1;
    
    app_data->objects.x[2] = -1.0f;
    app_data->objects.y[2] = 0.0;
    app_data->objects.z[2] = -1.0f;
    app_data->objects.shape[2] = OBJECT_SHAPE_SPHERE;
    app_data->objects.radius[2] = 0.5f;
    app_data->objects.radius_2[2] = 0.5f * 0.5f;
    app_data->objects.aabb_min_x[2] = -0.5f; app_data->objects.aabb_min_y[2] = -0.5f; app_data->objects.aabb_min_z[2] = -0.5f;
    app_data->objects.aabb_max_x[2] =  0.5f; app_data->objects.aabb_max_y[2] =  0.5f; app_data->objects.aabb_max_z[2] =  0.5f;
    app_data->objects.material_id[2] = 2;
    
    app_data->objects.x[3] = 1.0f;
    app_data->objects.y[3] = 0.0;
    app_data->objects.z[3] = -1.0f;
    app_data->objects.shape[3] = OBJECT_SHAPE_SPHERE;
    app_data->objects.radius[3] = 0.5f;
    app_data->objects.radius_2[3] = 0.5f * 0.5f;
    app_data->objects.aabb_min_x[3] = -0.5f; app_data->objects.aabb_min_y[3] = -0.5f; app_data->objects.aabb_min_z[3] = -0.5f;
    app_data->objects.aabb_max_x[3] =  0.5f; app_data->objects.aabb_max_y[3] =  0.5f; app_data->objects.aabb_max_z[3] =  0.5f;
    app_data->objects.material_id[3] = 3;
#endif
    // bvh
    bvh_populate(&app_data->objects, &app_data->b);
    
    // misc
    app_data->sky_color[0] = 0.6f;
    app_data->sky_color[1] = 0.7f;
    app_data->sky_color[2] = 0.9f;
    app_data->sky_color[3] = 1;
    
    app_data->sample = true;
    
    // copy over object data
    for(uint32_t i=0; i<NUM_TASKS; i++)
    {
        app_data->thread_data[i].objects = dm_alloc(sizeof(object_data));
        app_data->thread_data[i].materials = dm_alloc(sizeof(material_data));
    }
    
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
        dm_free(app_data->thread_data[i].objects);
        dm_free(app_data->thread_data[i].materials);
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
        
        app_data->image.resized = true;
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
#ifdef MT_IMAGE
    create_image_mt(app_data, context);
#else
    create_image(app_data, context);
#endif
    
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
    
    app_data->image.resized = false;
    
    return true;
}

bool dm_application_render(dm_context* context)
{
    application_data* app_data = context->app_data;
    
    dm_render_command_set_default_viewport(context);
    dm_render_command_clear(0,0,0,1, context);
    
    // all of this is just drawing a quad to fill the screem
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
        nk_value_uint(ctx, "Object count", OBJECT_COUNT);
        
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
        
        nk_layout_row_dynamic(ctx, 30, 1);
        nk_bool samp = app_data->sample;
        nk_checkbox_label(ctx, "Sample", &samp);
        
        app_data->sample = samp;
        
        struct nk_colorf bg;
        bg.r = app_data->sky_color[0];
        bg.g = app_data->sky_color[1];
        bg.b = app_data->sky_color[2];
        bg.a = app_data->sky_color[3];
        nk_layout_row_dynamic(ctx, 100, 1);
        bg = nk_color_picker(ctx, bg, NK_RGBA);
        
        app_data->sky_color[0] = bg.r;
        app_data->sky_color[1] = bg.g;
        app_data->sky_color[2] = bg.b;
        app_data->sky_color[3] = bg.a;
    }
    nk_end(ctx);
    /////////////////////////////////
    
    return true;
}
