"""GLSL shader 源码：多 pass HDR + Bloom 渲染管线。

Pipeline:
  Pass 1: 点云 → HDR FBO (RGBA16F / R11F_G11F_B10F)
          - CLOUD_VS/FS: VBO(pos, color) + uniform(MVP, light, fog, camera)
          - 点光源 + 雾 + 自发光 + 可选圆形 splat
  Pass 2: 亮度阈值 → Bright FBO
          - BRIGHT_FS: smoothstep threshold 提取高光给 bloom
  Pass 3: 高斯模糊 → Blur FBO (ping-pong)
  Pass 4: 合成 → 屏幕
          - HDR + blur(bloom) → Reinhard tonemap → gamma
"""

CLOUD_VS = """\
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uMVP;
uniform mat4 uModel;     // submap_local → root
uniform vec3 uCameraPos; // root 系
uniform float uPointSize;
uniform float uPointSizeFalloff;   // 0=不随距离变, 1=按距离衰减
out vec3 vWorldPos;
out vec3 vBaseColor;
out float vDistToCamera;
void main() {
    vec4 wp = uModel * vec4(aPos, 1.0);
    vWorldPos = wp.xyz;
    vBaseColor = aColor;
    vDistToCamera = length(uCameraPos - wp.xyz);
    gl_Position = uMVP * vec4(aPos, 1.0);
    // 按 1/dist 调整 point size（远点更小）
    float sz = uPointSize / max(1.0, vDistToCamera * uPointSizeFalloff);
    gl_PointSize = clamp(sz, 1.0, 64.0);
}
"""

CLOUD_FS = """\
#version 330 core
in vec3 vWorldPos;
in vec3 vBaseColor;
in float vDistToCamera;
uniform vec3 uLightPos;      // 点光源 root 系
uniform vec3 uLightColor;    // 光颜色
uniform float uLightRadius;  // exp 衰减常数
uniform vec3 uAmbient;       // 环境光 base 颜色（如 0.15 蓝）
uniform float uAmbientStrength;
uniform vec3 uFogColor;
uniform float uFogStart;
uniform float uFogEnd;
uniform bool uCircular;      // 圆形 splat
uniform float uExposure;     // HDR 曝光
uniform float uEmissive;     // 自发光占比 (0..1)
out vec4 fragColor;
void main() {
    // 圆形 splat：用 gl_PointCoord 裁掉方点边角
    if (uCircular) {
        vec2 cc = gl_PointCoord - vec2(0.5);
        float r2 = dot(cc, cc);
        if (r2 > 0.25) discard;
    }
    // 1) 点光：exp(-d/R)
    float d = distance(vWorldPos, uLightPos);
    float intensity = exp(-d / uLightRadius);
    // 2) "Phong-lite"：distance-attenuated diffuse
    //    （点云没有法线，无法做真 highlight）
    vec3 baseLit = vBaseColor * uLightColor * intensity;
    // 3) 自发光：baseColor 直接出
    vec3 emissive = vBaseColor * uEmissive;
    // 4) 环境光
    vec3 ambient = uAmbient * uAmbientStrength * vBaseColor;
    vec3 color = ambient + baseLit + emissive;
    // 5) 距离 spec：靠近光源时额外加亮一点
    float specT = smoothstep(0.0, uLightRadius * 0.4, uLightRadius - d);
    color += uLightColor * specT * 0.5;
    // 6) HDR 曝光
    color *= uExposure;
    // 7) 雾 (camera-space distance)
    float fogT = clamp((vDistToCamera - uFogStart) / max(0.001, uFogEnd - uFogStart),
                       0.0, 1.0);
    color = mix(color, uFogColor, fogT);
    fragColor = vec4(color, 1.0);
}
"""

FULLSCREEN_VS = """\
#version 330 core
out vec2 vUV;
void main() {
    // 3 顶点全屏 trick: gl_VertexID -> [(0,0),(2,0),(0,2)]
    vUV = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    gl_Position = vec4(vUV * 2.0 - 1.0, 0.0, 1.0);
}
"""

BRIGHT_FS = """\
#version 330 core
in vec2 vUV;
uniform sampler2D uScene;
uniform float uThreshold;     // 亮度阈值
uniform float uKnee;          // 软过渡宽度
out vec4 fragColor;
void main() {
    vec3 c = texture(uScene, vUV).rgb;
    float lum = max(c.r, max(c.g, c.b));
    float w = smoothstep(uThreshold, uThreshold + uKnee, lum);
    fragColor = vec4(c * w, 1.0);
}
"""

BLUR_FS = """\
#version 330 core
in vec2 vUV;
uniform sampler2D uTex;
uniform vec2 uDir;             // (1/W, 0) 水平 / (0, 1/H) 垂直
out vec4 fragColor;
const float w[5] = float[5](0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216);
void main() {
    vec3 c = texture(uTex, vUV).rgb * w[0];
    for (int i = 1; i < 5; i++) {
        c += texture(uTex, vUV + uDir * float(i)).rgb * w[i];
        c += texture(uTex, vUV - uDir * float(i)).rgb * w[i];
    }
    fragColor = vec4(c, 1.0);
}
"""

COMPOSITE_FS = """\
#version 330 core
in vec2 vUV;
uniform sampler2D uScene;
uniform sampler2D uBloom;
uniform float uBloomStrength;
uniform float uGamma;
out vec4 fragColor;
void main() {
    vec3 hdr = texture(uScene, vUV).rgb + texture(uBloom, vUV).rgb * uBloomStrength;
    // Reinhard tonemap
    vec3 ldr = hdr / (1.0 + hdr);
    // Gamma 校正
    ldr = pow(ldr, vec3(1.0 / uGamma));
    fragColor = vec4(ldr, 1.0);
}
"""

SIMPLE_VS = """\
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uMVP;
out vec3 vColor;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vColor = aColor;
}
"""

SIMPLE_FS = """\
#version 330 core
in vec3 vColor;
uniform float uAlpha;
out vec4 fragColor;
void main() {
    fragColor = vec4(vColor, uAlpha);
}
"""
