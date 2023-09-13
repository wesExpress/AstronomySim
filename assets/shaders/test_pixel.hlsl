struct PS_INPUT
{
	float4 position   : SV_Position;
	float2 tex_coords : TEXCOORDS1;
	float3 normal     : NORMAL1;
	float4 color      : COLOR1;
	float3 frag_pos   : FRAG_POS0;
};

cbuffer uni : register(b0)
{
	matrix view_proj;
	float3 view_pos;
};

SamplerState sample_state;
Texture2D tex : register(t0);

static const float3 light_pos = { 0,0,0 };
static const float4 light_ambient = { 0.33f,0.33f,0.33f,1.0f };
static const float4 light_diffuse = { 1,1,1,1 };
static const float4 light_specular = { 1,1,1,1 };

float4 p_main(PS_INPUT input) : SV_Target
{
	const float3 norm_normal = normalize(input.normal);
	const float3 view_dir = normalize(view_pos - input.frag_pos);
	const float3 light_dir = normalize(light_pos - input.frag_pos);
	const float3 reflect_dir = reflect(-light_dir, norm_normal);

	const float diff = max(dot(input.normal, light_dir), 0.0f);
	const float spec = pow(max(dot(view_dir, reflect_dir), 0.0f), 25);

	float4 obj_color = input.color * tex.Sample(sample_state, input.tex_coords);

	const float distance = length(light_pos - input.frag_pos);
	const float atten = 1.0f / (1.0f + 0.01f * distance + 0.0001f * distance * distance);

	float4 ambient = float4(input.color.xyz * light_ambient.xyz * atten, 1);
	float4 diffuse = float4(input.color.xyz * diff * light_diffuse.xyz * atten, 1);
	float4 specular = float4(input.color.xyz * spec * light_specular.xyz * atten, 1);

	return (ambient + diffuse + specular) * obj_color;
}