#include "pch.h"
#include "RenderBackground.h"

#include "../FluidRenderer/SSFluidRenderer.h"

#include "../../CGLib/GltfRenderer/Gltf/GltfReader.h"
#include "../../CGLib/GltfRenderer/Gltf/ObjToGltfConverter.h"
#include "../../CGLib/GltfRenderer/Gltf/StlToGltfConverter.h"
#include "../../CGLib/File/File/OBJFileReader.h"
#include "../../CGLib/File/File/STLFileReader.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cctype>
#include <cstdio>
#include <filesystem>
#include <optional>

namespace Phantom {

namespace {

std::string lowerExtension(const std::string& path) {
    std::string ext = std::filesystem::path(path).extension().string();
    for (auto& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return ext;
}

// .obj/.stl are converted to a GltfDocument on the fly (shared converters, same
// as Universe's Rendering/GltfRenderer::loadAsGltfDocument()); anything else goes
// through GltfReader::load() as a real .gltf/.glb file.
std::optional<Phantom::Gltf::GltfDocument> loadAsGltfDocument(const std::string& path) {
    const std::string ext = lowerExtension(path);
    if (ext == ".obj") {
        Phantom::File::OBJFileReader reader;
        if (!reader.read(path)) return std::nullopt;
        Phantom::Gltf::GltfDocument doc = Phantom::Gltf::ObjToGltfConverter::convert(reader.getOBJ());
        if (doc.meshes.empty()) return std::nullopt;
        return doc;
    }
    if (ext == ".stl") {
        Phantom::File::STLFileReader reader;
        const bool ok = Phantom::File::STLFileReader::isBinary(path)
            ? reader.readBinary(path)
            : reader.readAscii(path);
        if (!ok) return std::nullopt;
        Phantom::Gltf::GltfDocument doc = Phantom::Gltf::StlToGltfConverter::convert(reader.getSTL());
        if (doc.meshes.empty()) return std::nullopt;
        return doc;
    }
    return Phantom::Gltf::GltfReader::load(path);
}

std::string jsonEscape(const std::string& in) {
    std::string out;
    out.reserve(in.size() + 4);
    for (char c : in) {
        if (c == '"' || c == '\\') { out.push_back('\\'); out.push_back(c); }
        else if (c == '\n') out += "\\n";
        else out.push_back(c);
    }
    return out;
}

} // namespace

std::array<std::string, 6> RenderBackground::cubeFacePaths(const std::string& dir) {
    // VulkanCubeMap / SSFluidRenderer face order: right(+X), left(-X), top(+Y),
    // bottom(-Y), front(+Z), back(-Z).
    const std::string d = dir.empty() ? "." : dir;
    return { d + "/right.png", d + "/left.png", d + "/top.png",
             d + "/bottom.png", d + "/front.png", d + "/back.png" };
}

void RenderBackground::setInitialDocument(Phantom::Gltf::GltfDocument doc) {
    doc_ = std::move(doc);
    bgPath_.clear();
    if (gltf_) gltf_->loadDocument(doc_);
}

void RenderBackground::applyLight() {
    // GltfSceneRenderer: pos.w == 0 -> directional; color.w == intensity.
    if (gltf_)
        gltf_->setLight(glm::vec4(glm::normalize(lightDir_), 0.0f),
                        glm::vec4(lightColor_, lightIntensity_));
    // SetLight's direction is the direction the light travels; the SSFR
    // reflection model wants the direction from the surface toward the emitter.
    if (ssfr_)
        ssfr_->setLight(-glm::normalize(lightDir_), lightColor_, lightIntensity_);
}

void RenderBackground::applyTransform() {
    if (!gltf_) return;
    glm::mat4 m(1.f);
    m = glm::translate(m, xfPos_);
    m = glm::rotate(m, glm::radians(xfRotDeg_.z), glm::vec3(0.f, 0.f, 1.f));
    m = glm::rotate(m, glm::radians(xfRotDeg_.y), glm::vec3(0.f, 1.f, 0.f));
    m = glm::rotate(m, glm::radians(xfRotDeg_.x), glm::vec3(1.f, 0.f, 0.f));
    m = glm::scale(m, glm::vec3(xfScale_));
    gltf_->setModelMatrix(m);
}

bool RenderBackground::loadBackground(const std::string& path) {
    if (path.empty()) return false;
    if (!gltf_) { bgPath_ = path; return true; } // headless no-op success

    auto loaded = loadAsGltfDocument(path);
    if (!loaded) {
        std::fprintf(stderr, "[RenderBackground] failed to load '%s'\n", path.c_str());
        return false;
    }
    doc_ = std::move(*loaded);
    bgPath_ = path;
    gltf_->loadDocument(doc_); // does its own vkDeviceWaitIdle
    applyTransform();
    return true;
}

void RenderBackground::clearBackground() {
    bgPath_.clear();
    doc_ = Phantom::Gltf::GltfDocument{};
    if (gltf_) gltf_->loadDocument(emptyDoc_); // frees GPU resources, 0 primitives
}

void RenderBackground::setTransform(const glm::vec3& posMeters, const glm::vec3& rotDegXYZ, float scale) {
    xfPos_    = posMeters;
    xfRotDeg_ = rotDegXYZ;
    xfScale_  = (scale > 0.f) ? scale : 1.f;
    applyTransform();
}

bool RenderBackground::setEnvironment(const std::string& faceDir) {
    if (faceDir.empty()) return false;
    const auto faces = cubeFacePaths(faceDir);
    for (const auto& f : faces) {
        if (!std::filesystem::exists(f)) {
            std::fprintf(stderr, "[RenderBackground] env face missing: %s\n", f.c_str());
            return false;
        }
    }
    envDir_ = faceDir;
    hasEnv_ = true;
    if (ssfr_) ssfr_->loadEnvMap(faces); // SSFR reflections + SSFR-mode skybox
    return true;
}

void RenderBackground::clearEnvironment() {
    if (!defaultEnvDir_.empty() && ssfr_) {
        const auto faces = cubeFacePaths(defaultEnvDir_);
        bool ok = true;
        for (const auto& f : faces) if (!std::filesystem::exists(f)) { ok = false; break; }
        if (ok) ssfr_->loadEnvMap(faces);
    }
    envDir_.clear();
    hasEnv_ = false;
}

void RenderBackground::setLight(const glm::vec3& direction, const glm::vec3& color, float intensity) {
    lightDir_       = direction;
    lightColor_     = color;
    lightIntensity_ = glm::max(intensity, 0.f);
    applyLight();
}

void RenderBackground::setUseIBL(bool v) {
    useIBL_ = v;
    if (gltf_) gltf_->setUseIBL(v);
}

std::string RenderBackground::sceneStateJson() const {
    if (!gltf_) return "{}";
    const glm::vec3 d = glm::normalize(lightDir_);
    std::string s = "{";
    s += "\"background\":\"" + jsonEscape(bgPath_) + "\",";
    s += "\"primitives\":" + std::to_string(gltf_->primitiveCount()) + ",";
    s += "\"environment\":\"" + jsonEscape(envDir_) + "\",";
    s += "\"hasEnvironment\":" + std::string(hasEnv_ ? "true" : "false") + ",";
    s += "\"useIBL\":" + std::string(useIBL_ ? "true" : "false") + ",";
    s += "\"castShadows\":" + std::string(castShadows_ ? "true" : "false") + ",";
    s += "\"transform\":{\"pos\":[" + std::to_string(xfPos_.x) + "," + std::to_string(xfPos_.y) + "," + std::to_string(xfPos_.z) + "],";
    s += "\"rotDeg\":[" + std::to_string(xfRotDeg_.x) + "," + std::to_string(xfRotDeg_.y) + "," + std::to_string(xfRotDeg_.z) + "],";
    s += "\"scale\":" + std::to_string(xfScale_) + "},";
    s += "\"light\":{\"dir\":[" + std::to_string(d.x) + "," + std::to_string(d.y) + "," + std::to_string(d.z) + "],";
    s += "\"color\":[" + std::to_string(lightColor_.x) + "," + std::to_string(lightColor_.y) + "," + std::to_string(lightColor_.z) + "],";
    s += "\"intensity\":" + std::to_string(lightIntensity_) + "}";
    s += "}";
    return s;
}

} // namespace Phantom
