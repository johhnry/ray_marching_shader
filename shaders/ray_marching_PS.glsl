#version 330

uniform vec2 uResolution;
uniform float uTime;	

out vec4 outColor;

// Ray marching parameters
int MAX_MARCHING_STEPS = 10000;
float EPSILON = 0.0001;
float MIN_DEPTH = 0;
float MAX_DEPTH = 10000;


// 3d simplex noise function taken from : 
// https://github.com/ashima/webgl-noise/blob/master/src/noise3D.glsl
vec3 mod289(vec3 x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 mod289(vec4 x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 permute(vec4 x) {
     return mod289(((x*34.0)+1.0)*x);
}

vec4 taylorInvSqrt(vec4 r)
{
  return 1.79284291400159 - 0.85373472095314 * r;
}

float snoise(vec3 v)
{ 
  const vec2  C = vec2(1.0/6.0, 1.0/3.0) ;
  const vec4  D = vec4(0.0, 0.5, 1.0, 2.0);

// First corner
  vec3 i  = floor(v + dot(v, C.yyy) );
  vec3 x0 =   v - i + dot(i, C.xxx) ;

// Other corners
  vec3 g = step(x0.yzx, x0.xyz);
  vec3 l = 1.0 - g;
  vec3 i1 = min( g.xyz, l.zxy );
  vec3 i2 = max( g.xyz, l.zxy );

  //   x0 = x0 - 0.0 + 0.0 * C.xxx;
  //   x1 = x0 - i1  + 1.0 * C.xxx;
  //   x2 = x0 - i2  + 2.0 * C.xxx;
  //   x3 = x0 - 1.0 + 3.0 * C.xxx;
  vec3 x1 = x0 - i1 + C.xxx;
  vec3 x2 = x0 - i2 + C.yyy; // 2.0*C.x = 1/3 = C.y
  vec3 x3 = x0 - D.yyy;      // -1.0+3.0*C.x = -0.5 = -D.y

// Permutations
  i = mod289(i); 
  vec4 p = permute( permute( permute( 
             i.z + vec4(0.0, i1.z, i2.z, 1.0 ))
           + i.y + vec4(0.0, i1.y, i2.y, 1.0 )) 
           + i.x + vec4(0.0, i1.x, i2.x, 1.0 ));

// Gradients: 7x7 points over a square, mapped onto an octahedron.
// The ring size 17*17 = 289 is close to a multiple of 49 (49*6 = 294)
  float n_ = 0.142857142857; // 1.0/7.0
  vec3  ns = n_ * D.wyz - D.xzx;

  vec4 j = p - 49.0 * floor(p * ns.z * ns.z);  //  mod(p,7*7)

  vec4 x_ = floor(j * ns.z);
  vec4 y_ = floor(j - 7.0 * x_ );    // mod(j,N)

  vec4 x = x_ *ns.x + ns.yyyy;
  vec4 y = y_ *ns.x + ns.yyyy;
  vec4 h = 1.0 - abs(x) - abs(y);

  vec4 b0 = vec4( x.xy, y.xy );
  vec4 b1 = vec4( x.zw, y.zw );

  //vec4 s0 = vec4(lessThan(b0,0.0))*2.0 - 1.0;
  //vec4 s1 = vec4(lessThan(b1,0.0))*2.0 - 1.0;
  vec4 s0 = floor(b0)*2.0 + 1.0;
  vec4 s1 = floor(b1)*2.0 + 1.0;
  vec4 sh = -step(h, vec4(0.0));

  vec4 a0 = b0.xzyw + s0.xzyw*sh.xxyy ;
  vec4 a1 = b1.xzyw + s1.xzyw*sh.zzww ;

  vec3 p0 = vec3(a0.xy,h.x);
  vec3 p1 = vec3(a0.zw,h.y);
  vec3 p2 = vec3(a1.xy,h.z);
  vec3 p3 = vec3(a1.zw,h.w);

//Normalise gradients
  vec4 norm = taylorInvSqrt(vec4(dot(p0,p0), dot(p1,p1), dot(p2, p2), dot(p3,p3)));
  p0 *= norm.x;
  p1 *= norm.y;
  p2 *= norm.z;
  p3 *= norm.w;

// Mix final noise value
  vec4 m = max(0.5 - vec4(dot(x0,x0), dot(x1,x1), dot(x2,x2), dot(x3,x3)), 0.0);
  m = m * m;
  return 105.0 * dot( m*m, vec4( dot(p0,x0), dot(p1,x1), 
                                dot(p2,x2), dot(p3,x3) ) );
	}

// Computing the screen space ray direction
vec3 rayDir(float fov, vec2 size, vec2 fragCoord)
{
	vec2 xy = fragCoord.xy - (size / 2);
    float z = size.y / tan(radians(fov) / 2);
    return normalize(vec3(xy, -z));
}

// Primitives shapes SDF taken from : 
// https://iquilezles.org/www/articles/distfunctions/distfunctions.htm

float sphereSDF(vec3 pos, vec3 origin, float radius)
{
	return length(pos + origin) - radius;
}

float orbitSphereSDF(vec3 pos, float radius, float speed)
{
	return sphereSDF(pos, vec3(cos(uTime * speed) * radius,
					sin(uTime * speed) * radius, 0), 0.2);
}

float rectangleSDF(vec3 pos, vec3 center, vec3 b)
{
	vec3 q = abs(pos + center) - b;
  	return length(max(q, 0)) + min(max(q.x,max(q.y,q.z)),0.0);
}

float roundRectangleSDF( vec3 p, vec3 b, float r )
{
  vec3 q = abs(p) - b;
  return length(max(q,0.0)) + min(max(q.x,max(q.y,q.z)),0.0) - r;
}

// Infinite spheres SDF
float infiniteSpheresSDF(vec3 pos)
{
	return length(vec3(cos(pos.x), cos(pos.y + uTime), sin(pos.z))) - 
					(abs(cos(pos.z / 30 + uTime * 2) / 2));
}

float infinitePlaneSDF(vec3 pos, float height)
{
	return dot(pos, vec3(0, 1.0, 0));
}

float distortedSphereSDF(vec3 pos, vec3 origin, float radius)
{
	return length(pos + origin) - (radius + snoise(pos + uTime/4));
}

// Intersection between two SDFs functions
float intersectSDF(float distA, float distB) 
{
    return max(distA, distB);
}

// Union between two SDFs functions
float unionSDF(float distA, float distB) 
{
    return min(distA, distB);
}

// Difference between two SDFs functions
float differenceSDF(float distA, float distB) 
{
    return max(distA, -distB);
}

//  Return the global scene SDF
float sceneSDF(vec3 pos)
{
	return unionSDF(
			differenceSDF(
				sphereSDF(pos, vec3(0), 1),
				distortedSphereSDF(pos, vec3(0), 1)),
			orbitSphereSDF(pos, 1.5, 1));
}

// Compute the shortest distance to the scene
float shortestDistance(vec3 eye, vec3 ray, float start, float end)
{
	float depth = start;
	
	// Ray marching loop
	for (int i = 0; i < MAX_MARCHING_STEPS; i++)
	{
		// Compute the distance
		float dist = sceneSDF(eye + depth * ray);
		
		// If we are close enough, we stop
		if (dist < EPSILON)
		{
			return depth;
		}
		
		// Move along the ray
		depth += dist;
		
		// if we reach the limit, return
		if (depth > end)
		{
			return end;
		}
	}
	
	return end;	
}

// Estimate surface normal by making an approximation
// with the gradient of the surface curvature
vec3 estimateNormal(vec3 p) 
{
    return normalize(vec3(
        sceneSDF(vec3(p.x + EPSILON, p.y, p.z)) - sceneSDF(vec3(p.x - EPSILON, p.y, p.z)),
        sceneSDF(vec3(p.x, p.y + EPSILON, p.z)) - sceneSDF(vec3(p.x, p.y - EPSILON, p.z)),
        sceneSDF(vec3(p.x, p.y, p.z  + EPSILON)) - sceneSDF(vec3(p.x, p.y, p.z - EPSILON))
    ));
}

// Compute phong lighting at point
vec3 phongLighting(vec3 p, vec3 normal, vec3 ray, vec3 diffuseColor)
{
	// Light setup
    vec3 lightPos = vec3(cos(uTime) * 100, sin(uTime) * 100, sin(uTime) * 5);
    vec3 lightDir = lightPos - p;
    float lightDistance = length(lightDir);
    lightDir = normalize(lightDir);
    
    // Ambient
    vec3 ambient = vec3(0.01, 0.01, 0.02);
    
    // Diffuse
    vec3 diffuse = max(dot(lightDir, normal), 0) * vec3(1, 1, 1);
    
    // Specular	
    vec3 reflectDir = reflect(lightDir, normal);
    float specAngle = max(dot(reflectDir, ray), 0.0);
    float specular = pow(specAngle, 128 / 4.0);
    
    return ambient + diffuse * diffuseColor + specular;
}

// Compute the view matrix similar to lookAt
mat4 viewMatrix(vec3 eye, vec3 center, vec3 up) {
	vec3 f = normalize(center - eye);
	vec3 s = normalize(cross(f, up));
	vec3 u = cross(s, f);
	return mat4(
		vec4(s, 0.0),
		vec4(u, 0.0),
		vec4(-f, 0.0),
		vec4(0.0, 0.0, 0.0, 1)
	);
}

void main()
{
	// Eye position
    vec3 eye = vec3(5, 5, 5);
    
    
    // Ray direction from camera
    vec3 viewDir = rayDir(50, uResolution, gl_FragCoord.xy);
    
    // Camera matrix look at
    mat4 viewToWorld = viewMatrix(eye, vec3(0.0, 0.0, 0.0), vec3(0, 1, 0));
    
    // World ray direction
    vec3 worldDir = (viewToWorld * vec4(viewDir, 0.0)).xyz;
    
    // Distance from scene
    float dist = shortestDistance(eye, worldDir, MIN_DEPTH, MAX_DEPTH);
    
    // Stop if ray don't touch
    if (dist > MAX_DEPTH - EPSILON) {
    	outColor = vec4(0.0, 0.0, 0.0, 0.0);
        return;
    }
    
    // Compute point on surface
    vec3 p = eye + dist * worldDir;
    
    // Compute normal
    vec3 normal = estimateNormal(p);
    
    // Compute phong lighting
    vec3 colorLinear = phongLighting(p, normal, worldDir, normal);
    
    // Gamma correction
    // https://en.wikipedia.org/wiki/Blinn%E2%80%93Phong_reflection_model
    vec3 colorGammaCorrected = pow(colorLinear, vec3(1.0 / 2.2));
    
    // Return the color
    outColor = vec4(colorGammaCorrected, 1);
}