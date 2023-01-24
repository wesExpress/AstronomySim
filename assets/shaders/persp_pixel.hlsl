struct PS_INPUT
{
	float4 position     : SV_Position;
	float3 normal       : NORMAL1;
	float2 tex_coords   : TEXCOORD1;
	float3 frag_pos     : FRAG_POS;
	float4 obj_diffuse  : OBJ_DIFFUSE1;
	float4 obj_specular : OBJ_SPECULAR1;
};

cbuffer scene_cb : register(b0)
{
	matrix view_proj;
	float4 light_color;
	float4 ambient_color;
	float3 light_pos;
	float  padding;
	float3 view_pos;
};

SamplerState sample_state;
Texture2D obj_texture : register(t0);

float4 p_main(PS_INPUT input) : SV_Target
{
	float3 norm_normal = normalize(input.normal);
	float3 light_dir = normalize(light_pos - input.frag_pos);
	float3 view_dir = normalize(view_pos - input.frag_pos);
	float3 reflect_dir = reflect(-light_dir, norm_normal);
		
	float diff = max(dot(norm_normal, light_dir), 0.0f);
	float spec = pow(max(dot(view_dir, reflect_dir), 0.0f), 4);

	float4 texture_color = obj_texture.Sample(sample_state, input.tex_coords);

	float4 diffuse  = diff * light_color * input.obj_diffuse * texture_color;
	float4 specular = spec * light_color * input.obj_specular * texture_color; 

	return (ambient_color * input.obj_diffuse * texture_color + diffuse + specular);
}