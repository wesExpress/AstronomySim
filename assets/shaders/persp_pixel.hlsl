struct PS_INPUT
{
	float4 position     : SV_Position;
	float3 normal       : NORMAL1;
	float2 tex_coords   : TEXCOORD1;
	float3 frag_pos     : FRAG_POS;
	float4 obj_diffuse  : OBJ_DIFFUSE1;
	float4 obj_specular : OBJ_SPECULAR1;
	float  depth        : DEPTH;
};

struct PS_OUTPUT
{
	float4 color : SV_Target;
	float  depth : SV_Depth;
};

#define MAX_LIGHTS 10
struct point_light
{
	float4 position;

	float4 ambient;
	float4 diffuse;
	float4 specular;

	float4 params;
};

struct blackbody
{
	float4 position;
	float4 color;
	float  brightness;
	float3 padding;
};

// constant buffers
cbuffer scene_cb : register(b0)
{
	matrix view_proj;
	float3 view_pos;
	float  fcoef_inv;
};

cbuffer lights_cb : register(b1)
{
	point_light point_lights[MAX_LIGHTS];
	blackbody   blackbodies[MAX_LIGHTS];

	uint        num_point_lights;
	uint        num_blackbodies;
};

SamplerState sample_state;
Texture2D obj_texture : register(t0);

#define INV_4PI 0.0795774715459f

float3 calc_point_light(point_light light, float3 normal, float3 frag_pos, float3 view_dir, float3 diffuse_color, float3 specular_color)
{
	float3 light_pos   = light.position.xyz;
	float3 light_dir   = normalize(light_pos - frag_pos);
	float3 reflect_dir = reflect(-light_dir, normal);

	float diff = max(dot(normal, light_dir), 0.0f);
	float spec = pow(max(dot(view_dir, reflect_dir), 0.0f), 4);

	float distance    = length(light_pos - frag_pos);
	float attenuation = 1.0f / (light.params.x + light.params.y * distance + light.params.z * distance * distance);

	float3 ambient  = light.ambient.xyz  * diffuse_color;
	float3 diffuse  = light.diffuse.xyz  * diff * diffuse_color;
	float3 specular = light.specular.xyz * spec * specular_color;

	return (ambient + diffuse + specular) * attenuation;
}

float3 calc_blackbody_light(blackbody bb, float3 normal, float3 frag_pos, float3 view_dir, float3 diffuse_color, float3 specular_color)
{
	float3 light_pos   = bb.position.xyz;
	float3 light_dir   = normalize(light_pos - frag_pos);
	float3 reflect_dir = reflect(-light_dir, normal);

	float diff = max(dot(normal, light_dir), 0.0f);
	float spec = pow(max(dot(view_dir, reflect_dir), 0.0f), 4);

	float3 diffuse  = bb.color.xyz * diff * diffuse_color;
	float3 specular = bb.color.xyz * spec * specular_color;

	return (diffuse + specular) * bb.brightness / 2048.0f;
}

PS_OUTPUT p_main(PS_INPUT input)
{
	PS_OUTPUT output = (PS_OUTPUT)0;

	float3 norm_normal = normalize(input.normal);
	float3 view_dir    = normalize(view_pos - input.frag_pos);

	output.color = 0;
	for(uint i=0; i<num_point_lights; i++)
	{
		float3 color = calc_point_light(point_lights[i], norm_normal, input.frag_pos, view_dir, input.obj_diffuse.xyz, input.obj_specular.xyz);
		output.color += float4(color, 1);
	}

	for(uint j=0; j<num_blackbodies; j++)
	{
		float3 color = calc_blackbody_light(blackbodies[j], norm_normal, input.frag_pos, view_dir, input.obj_diffuse.xyz, input.obj_specular.xyz);
		output.color += float4(color, 1);
	}
	
	output.color *= obj_texture.Sample(sample_state, input.tex_coords);
	
	output.depth = log2(input.depth) * fcoef_inv;

	return output;
}