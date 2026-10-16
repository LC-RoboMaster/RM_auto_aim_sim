#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

#include "rm_auto_aim_sim/hdr_pipeline.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace rm_auto_aim_sim {
namespace {
constexpr char kFullscreen[] = R"GLSL(#version 330 core
out vec2 uv;
void main() {
  vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
  uv = p;
  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
})GLSL";

// Geometry and textures come from MuJoCo's single raster pass. This shader
// reconstructs position from depth and adds radiance from the visible LEDs.
// GGX/Fresnel/Smith follow Filament's documented microfacet model. The native
// image supplies the environment term, preserving textured number stickers.
constexpr char kRelight[] = R"GLSL(#version 330 core
in vec2 uv;
out vec4 outColor;
uniform sampler2D baseColor;
uniform sampler2D sceneDepth;
uniform vec4 K;                    // fx fy cx cy in OpenCV pixel coordinates
uniform vec2 resolution;
uniform vec2 clipPlanes;
uniform vec3 cameraPos, cameraRight, cameraUp, cameraForward;
uniform int lightCount;
uniform vec3 lightPos[16], lightColor[16];
uniform float lampIntensity;

vec3 worldPosition(float depth) {
  // MuJoCo stores reversed Z: 1 at the near plane, 0 at the far plane.
  float distance = clipPlanes.x * clipPlanes.y /
                   (clipPlanes.x + depth * (clipPlanes.y - clipPlanes.x));
  vec2 pixel = uv * resolution - vec2(.5);
  vec2 xy = vec2((pixel.x - K.z) / K.x,
                 (pixel.y - (resolution.y - 1.0 - K.w)) / K.y);
  return cameraPos + distance * (cameraForward + xy.x * cameraRight + xy.y * cameraUp);
}
float distributionGGX(float nDotH, float roughness) {
  float a = roughness * roughness;
  float a2 = a * a;
  float d = nDotH * nDotH * (a2 - 1.0) + 1.0;
  return a2 / max(3.14159265 * d * d, 1e-5);
}
float smithG(float nDotX, float roughness) {
  float k = pow(roughness + 1.0, 2.0) / 8.0;
  return nDotX / (nDotX * (1.0 - k) + k);
}
void main() {
  vec3 encoded = texture(baseColor, uv).rgb;
  float depth = texture(sceneDepth, uv).r;
  if (depth <= 0.0) { outColor = vec4(pow(encoded, vec3(2.2)), 1.0); return; }
  vec3 p = worldPosition(depth);
  vec3 dx = dFdx(p), dy = dFdy(p);
  vec3 n = normalize(cross(dy, dx));
  vec3 v = normalize(cameraPos - p);
  if (dot(n, v) < 0.0) n = -n;
  vec3 albedo = pow(encoded, vec3(2.2));
  vec3 radiance = albedo;
  const float roughness = .58;
  vec3 f0 = vec3(.04);
  for (int i = 0; i < lightCount; ++i) {
    vec3 delta = lightPos[i] - p;
    float d2 = max(dot(delta, delta), .025);
    vec3 l = normalize(delta);
    vec3 h = normalize(v + l);
    float nl = max(dot(n, l), 0.0), nv = max(dot(n, v), 0.0);
    float vh = max(dot(v, h), 0.0);
    vec3 f = f0 + (1.0 - f0) * pow(1.0 - vh, 5.0);
    float spec = distributionGGX(max(dot(n, h), 0.0), roughness) *
                 smithG(nl, roughness) * smithG(nv, roughness) /
                 max(4.0 * nl * nv, 1e-4);
    vec3 irradiance = lampIntensity * lightColor[i] /
                      (1.0 + 5.0 * d2 + 3.0 * d2 * d2);
    radiance += (albedo * (1.0 - f) / 3.14159265 + f * spec) * irradiance * nl;
    // The source pixels are identified using both depth and chroma, so an
    // occluded bar cannot shine through a foreground object.
    float nearSource = 1.0 - smoothstep(.04, .27, sqrt(d2));
    float chroma = max(max(encoded.r, encoded.g), encoded.b) -
                   min(min(encoded.r, encoded.g), encoded.b);
    radiance += lightColor[i] * lampIntensity * 1.4 * nearSource *
                smoothstep(.05, .20, chroma);
  }
  outColor = vec4(radiance, 1.0);
})GLSL";

constexpr char kDownsample[] = R"GLSL(#version 330 core
in vec2 uv;
out vec4 outColor;
uniform sampler2D source;
void main() {
  vec2 t = 1.0 / vec2(textureSize(source, 0));
  vec3 c = texture(source, uv).rgb * .25;
  c += texture(source, uv + vec2(t.x, 0)).rgb * .125;
  c += texture(source, uv - vec2(t.x, 0)).rgb * .125;
  c += texture(source, uv + vec2(0, t.y)).rgb * .125;
  c += texture(source, uv - vec2(0, t.y)).rgb * .125;
  outColor = vec4(max(c - vec3(1.0), 0.0), 1.0);
})GLSL";

constexpr char kBlur[] = R"GLSL(#version 330 core
in vec2 uv;
out vec4 outColor;
uniform sampler2D source;
uniform vec2 direction;
void main() {
  vec3 color = texture(source, uv).rgb * .227027;
  color += texture(source, uv + direction * 1.384615).rgb * .316216;
  color += texture(source, uv - direction * 1.384615).rgb * .316216;
  color += texture(source, uv + direction * 3.230769).rgb * .070270;
  color += texture(source, uv - direction * 3.230769).rgb * .070270;
  outColor = vec4(color, 1.0);
})GLSL";

constexpr char kCamera[] = R"GLSL(#version 330 core
in vec2 uv;
out vec4 outColor;
uniform sampler2D hdrColor, bloomColor, nativeColor;
uniform float exposure, bloomStrength;
uniform int nativeMode;
void main() {
  // GL reads the bottom framebuffer row first; write the camera's top row
  // there to avoid a full CPU vertical flip of every image.
  vec2 cameraUv = vec2(uv.x, 1.0 - uv.y);
  if (nativeMode != 0) {
    outColor = vec4(texture(nativeColor, cameraUv).rgb, 1.0);
    return;
  }
  vec3 hdr = (texture(hdrColor, cameraUv).rgb +
              bloomStrength * texture(bloomColor, cameraUv).rgb) * exposure;
  // ACES fitted response, then display encoding. Exposure is fixed for
  // repeatable detector tests; no per-frame auto-exposure feedback.
  vec3 mapped = clamp((hdr * (2.51 * hdr + .03)) /
                      (hdr * (2.43 * hdr + .59) + .14), 0.0, 1.0);
  outColor = vec4(pow(mapped, vec3(1.0 / 2.2)), 1.0);
})GLSL";

GLuint Compile(GLenum type, const char* source) {
  GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  GLint ok = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[2048]{};
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    glDeleteShader(shader);
    throw std::runtime_error(std::string("Camera shader compile failed: ") + log);
  }
  return shader;
}
GLuint Program(const char* fragment) {
  GLuint vertex = Compile(GL_VERTEX_SHADER, kFullscreen);
  GLuint pixel = Compile(GL_FRAGMENT_SHADER, fragment);
  GLuint program = glCreateProgram();
  glAttachShader(program, vertex);
  glAttachShader(program, pixel);
  glLinkProgram(program);
  glDeleteShader(vertex);
  glDeleteShader(pixel);
  GLint ok = GL_FALSE;
  glGetProgramiv(program, GL_LINK_STATUS, &ok);
  if (!ok) {
    char log[2048]{};
    glGetProgramInfoLog(program, sizeof(log), nullptr, log);
    glDeleteProgram(program);
    throw std::runtime_error(std::string("Camera shader link failed: ") + log);
  }
  return program;
}
GLuint Texture(int width, int height, GLenum format, GLenum pixelFormat, GLenum pixelType) {
  GLuint id;
  glGenTextures(1, &id);
  glBindTexture(GL_TEXTURE_2D, id);
  glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, pixelFormat, pixelType, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  return id;
}
GLuint Framebuffer(GLuint color, GLuint depth = 0) {
  GLuint id;
  glGenFramebuffers(1, &id);
  glBindFramebuffer(GL_FRAMEBUFFER, id);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color, 0);
  if (depth)
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, depth, 0);
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    throw std::runtime_error("Camera shader framebuffer is incomplete");
  return id;
}
void BindTexture(GLuint program, const char* name, GLuint texture, int unit) {
  glActiveTexture(GL_TEXTURE0 + unit);
  glBindTexture(GL_TEXTURE_2D, texture);
  glUniform1i(glGetUniformLocation(program, name), unit);
}
void Draw(GLuint fbo, int width, int height) {
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  glViewport(0, 0, width, height);
  glDrawArrays(GL_TRIANGLES, 0, 3);
}
void Check(const char* pass) {
  const GLenum error = glGetError();
  if (error != GL_NO_ERROR)
    throw std::runtime_error(std::string("HDR ") + pass + " GL error " +
                             std::to_string(static_cast<unsigned>(error)));
}
}  // namespace

struct HdrPipeline::Impl {
  int width, height, bloomWidth, bloomHeight;
  GLuint vao = 0, base = 0, depth = 0, hdr = 0, blur[2]{}, final = 0;
  GLuint baseFbo = 0, hdrFbo = 0, blurFbo[2]{}, finalFbo = 0;
  GLuint relight = 0, downsample = 0, blurProgram = 0, camera = 0;
  explicit Impl(int w, int h, const mjrContext& context)
      : width(w), height(h), bloomWidth(std::max(1, w / 4)),
        bloomHeight(std::max(1, h / 4)) {
    if (!glGenVertexArrays || !glBlitFramebuffer || !glCreateShader)
      throw std::runtime_error("Camera shader requires OpenGL 3.3");
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    relight = Program(kRelight);
    downsample = Program(kDownsample);
    blurProgram = Program(kBlur);
    camera = Program(kCamera);
    base = Texture(w, h, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    GLint sourceDepthFormat = 0;
    glBindRenderbuffer(GL_RENDERBUFFER, context.offDepthStencil);
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_INTERNAL_FORMAT,
                                 &sourceDepthFormat);
    if (sourceDepthFormat != GL_DEPTH24_STENCIL8 &&
        sourceDepthFormat != GL_DEPTH32F_STENCIL8)
      throw std::runtime_error("Unsupported MuJoCo depth-stencil buffer format");
    const GLenum depthPixelType = sourceDepthFormat == GL_DEPTH32F_STENCIL8
        ? GL_FLOAT_32_UNSIGNED_INT_24_8_REV : GL_UNSIGNED_INT_24_8;
    depth = Texture(w, h, sourceDepthFormat, GL_DEPTH_STENCIL, depthPixelType);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    hdr = Texture(w, h, GL_RGBA16F, GL_RGBA, GL_HALF_FLOAT);
    blur[0] = Texture(bloomWidth, bloomHeight, GL_RGBA16F, GL_RGBA, GL_HALF_FLOAT);
    blur[1] = Texture(bloomWidth, bloomHeight, GL_RGBA16F, GL_RGBA, GL_HALF_FLOAT);
    final = Texture(w, h, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    baseFbo = Framebuffer(base, depth);
    hdrFbo = Framebuffer(hdr);
    blurFbo[0] = Framebuffer(blur[0]);
    blurFbo[1] = Framebuffer(blur[1]);
    finalFbo = Framebuffer(final);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }
  ~Impl() {
    GLuint fbos[] = {baseFbo, hdrFbo, blurFbo[0], blurFbo[1], finalFbo};
    glDeleteFramebuffers(5, fbos);
    GLuint textures[] = {base, depth, hdr, blur[0], blur[1], final};
    glDeleteTextures(6, textures);
    GLuint programs[] = {relight, downsample, blurProgram, camera};
    for (GLuint program : programs) glDeleteProgram(program);
    glDeleteVertexArrays(1, &vao);
  }
};

HdrPipeline::HdrPipeline(int width, int height, const mjrContext& context)
    : impl_(std::make_unique<Impl>(width, height, context)) {}
HdrPipeline::~HdrPipeline() = default;

void HdrPipeline::Process(const mjvScene& scene, const mjrContext& context,
                          double fx, double fy, double cx, double cy,
                          double exposure, double bloom, double lamp_intensity,
                          bool shader_enabled) {
  auto& p = *impl_;
  glDisable(GL_BLEND);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_FRAMEBUFFER_SRGB);
  glBindVertexArray(p.vao);
  glBindFramebuffer(GL_READ_FRAMEBUFFER, context.offFBO);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, p.baseFbo);
  glBlitFramebuffer(0, 0, p.width, p.height, 0, 0, p.width, p.height,
                    GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, GL_NEAREST);
  Check("source blit");
  if (shader_enabled) {
    const auto& cam = scene.camera[0];
    glUseProgram(p.relight);
    BindTexture(p.relight, "baseColor", p.base, 0);
    BindTexture(p.relight, "sceneDepth", p.depth, 1);
    glUniform4f(glGetUniformLocation(p.relight, "K"), fx, fy, cx, cy);
    glUniform2f(glGetUniformLocation(p.relight, "resolution"), p.width, p.height);
    glUniform2f(glGetUniformLocation(p.relight, "clipPlanes"), cam.frustum_near, cam.frustum_far);
    glUniform3fv(glGetUniformLocation(p.relight, "cameraPos"), 1, cam.pos);
    glUniform3fv(glGetUniformLocation(p.relight, "cameraUp"), 1, cam.up);
    glUniform3fv(glGetUniformLocation(p.relight, "cameraForward"), 1, cam.forward);
    const float right[3] = {cam.forward[1] * cam.up[2] - cam.forward[2] * cam.up[1],
                            cam.forward[2] * cam.up[0] - cam.forward[0] * cam.up[2],
                            cam.forward[0] * cam.up[1] - cam.forward[1] * cam.up[0]};
    glUniform3fv(glGetUniformLocation(p.relight, "cameraRight"), 1, right);
    std::array<float, 16 * 3> positions{}, colors{};
    int count = 0;
    for (int i = 0; i < scene.ngeom && count < 16; ++i) {
      const auto& geom = scene.geoms[i];
      if (geom.emission < .5f || geom.rgba[3] < .5f) continue;
      for (int c = 0; c < 3; ++c) {
        positions[3 * count + c] = geom.pos[c];
        colors[3 * count + c] = geom.rgba[c] * geom.emission;
      }
      ++count;
    }
    glUniform1i(glGetUniformLocation(p.relight, "lightCount"), count);
    glUniform3fv(glGetUniformLocation(p.relight, "lightPos[0]"), count, positions.data());
    glUniform3fv(glGetUniformLocation(p.relight, "lightColor[0]"), count, colors.data());
    glUniform1f(glGetUniformLocation(p.relight, "lampIntensity"), lamp_intensity);
    Draw(p.hdrFbo, p.width, p.height);
    Check("relight");

    glUseProgram(p.downsample);
    BindTexture(p.downsample, "source", p.hdr, 0);
    Draw(p.blurFbo[0], p.bloomWidth, p.bloomHeight);
    Check("bloom");
    glUseProgram(p.blurProgram);
    BindTexture(p.blurProgram, "source", p.blur[0], 0);
    glUniform2f(glGetUniformLocation(p.blurProgram, "direction"),
                1.0f / p.bloomWidth, 0);
    Draw(p.blurFbo[1], p.bloomWidth, p.bloomHeight);
    BindTexture(p.blurProgram, "source", p.blur[1], 0);
    glUniform2f(glGetUniformLocation(p.blurProgram, "direction"),
                0, 1.0f / p.bloomHeight);
    Draw(p.blurFbo[0], p.bloomWidth, p.bloomHeight);
  }
  glUseProgram(p.camera);
  BindTexture(p.camera, "nativeColor", p.base, 0);
  BindTexture(p.camera, "hdrColor", p.hdr, 1);
  BindTexture(p.camera, "bloomColor", p.blur[0], 2);
  glUniform1i(glGetUniformLocation(p.camera, "nativeMode"), shader_enabled ? 0 : 1);
  glUniform1f(glGetUniformLocation(p.camera, "exposure"), exposure);
  glUniform1f(glGetUniformLocation(p.camera, "bloomStrength"), bloom);
  Draw(p.finalFbo, p.width, p.height);
  Check("camera response");
  // MuJoCo's next fixed-function scene draw shares this context.
  glUseProgram(0);
  glBindVertexArray(0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, 0);
  glEnable(GL_DEPTH_TEST);
}

void HdrPipeline::ReadPixelsToPbo(unsigned int pbo) const {
  const auto& p = *impl_;
  glBindFramebuffer(GL_READ_FRAMEBUFFER, p.finalFbo);
  glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, p.width, p.height, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
  glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
}

double HdrPipeline::EstimatedMiB() const {
  const auto& p = *impl_;
  const double bytes = double(p.width) * p.height * (4 + 4 + 8 + 4 + 8) +
                       double(p.bloomWidth) * p.bloomHeight * 16;
  return bytes / (1024.0 * 1024.0);
}
}  // namespace rm_auto_aim_sim
