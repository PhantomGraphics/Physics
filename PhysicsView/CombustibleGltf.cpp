#include "CombustibleGltf.h"
#include "../../CGLib/GltfRenderer/Gltf/GltfAccessorBuilder.h"
#include <algorithm>
#include <cmath>

int Phantom::combustibleColorLevel(const Physics::CombustibleSample& s,int mode)
{
    const double value=mode==1?(s.temperature-300)/1200.0:s.fuel/s.initialFuel;
    return std::clamp(static_cast<int>(value*15),0,15);
}

static Phantom::Gltf::GltfDocument buildCombustibleGltf(const Phantom::Physics::CombustibleBody& body,int mode,bool scalarField)
{
    using namespace Phantom;
    using namespace Gltf;
    GltfDocument doc; GltfMesh mesh;
    struct Geometry { std::vector<glm::vec3> p,n; std::vector<glm::vec2> uv; std::vector<uint32_t> indices; };
    Geometry geometry[16];
    for(int i=0;i<(scalarField?1:16);++i) {
        const float t=i/15.0f; glm::vec3 color=glm::mix(glm::vec3(0.015f),glm::vec3(0.45f,0.22f,0.08f),t);
        if(mode==1) color=glm::mix(glm::vec3(0.02f,0.1f,0.6f),glm::vec3(1,0.12f,0.01f),t);
        if(mode==2) color=glm::mix(glm::vec3(0.1f,0.01f,0.01f),glm::vec3(0.1f,0.7f,0.1f),t);
        GltfMaterial material; material.pbrMetallicRoughness.baseColorFactor=glm::vec4(scalarField?glm::vec3(1):color,1);
        material.pbrMetallicRoughness.metallicFactor=0; material.pbrMetallicRoughness.roughnessFactor=0.8f;
        doc.materials.push_back(material);
    }
    const bool box=body.shape()==Physics::CombustibleBody::Shape::Box;
    const int n=static_cast<int>(std::lround(std::sqrt(body.samples().size()/(box?6.0:8.0))));
    for(size_t index=0;index<body.samples().size();++index) {
        const auto& s=body.samples()[index]; auto& g=geometry[scalarField?0:combustibleColorLevel(s,mode)];
        if(body.usesSolidParticles()) {
            // Opaque particle glyphs: the visible colors are the actual volume
            // particle temperatures/fuel, with the same depth in Normal/PBVR.
            constexpr int bands=6,slices=8;
            constexpr float pi=3.14159265359f;
            const auto base=static_cast<uint32_t>(g.p.size());
            for(int lat=0;lat<=bands;++lat) for(int lon=0;lon<=slices;++lon) {
                const float theta=pi*lat/bands,phi=2*pi*lon/slices;
                const glm::vec3 axis{std::sin(theta)*std::cos(phi),std::cos(theta),std::sin(theta)*std::sin(phi)};
                g.p.push_back(s.position+axis*s.glyphHalfExtent);
                g.n.push_back(glm::normalize(axis/s.glyphHalfExtent));
                if(scalarField) g.uv.emplace_back(static_cast<float>(index),0);
            }
            for(int lat=0;lat<bands;++lat) for(int lon=0;lon<slices;++lon) {
                const auto a=base+lat*(slices+1)+lon,b=a+1,c=a+slices+1,d=c+1;
                if(lat>0) g.indices.insert(g.indices.end(),{a,b,c});
                if(lat<bands-1) g.indices.insert(g.indices.end(),{b,d,c});
            }
            continue;
        }
        glm::vec3 p[4],normals[4];
        if(box) {
            int axis=0; if(std::abs(s.normal.y)>0.5f) axis=1; if(std::abs(s.normal.z)>0.5f) axis=2;
            const int u=(axis+1)%3,v=(axis+2)%3;
            const float du=body.halfExtent()[u]/n,dv=body.halfExtent()[v]/n;
            for(int k=0;k<4;++k) { p[k]=s.position; normals[k]=s.normal; }
            p[0][u]-=du; p[0][v]-=dv; p[1][u]+=du; p[1][v]-=dv;
            p[2][u]+=du; p[2][v]+=dv; p[3][u]-=du; p[3][v]+=dv;
            if(s.normal[axis]<0) std::swap(p[1],p[3]);
        } else {
            constexpr float pi=3.14159265359f;
            const int lat=static_cast<int>(index)/(4*n),lon=static_cast<int>(index)%(4*n);
            const int band[]={lat,lat,lat+1,lat+1},slice[]={lon,lon+1,lon+1,lon};
            for(int k=0;k<4;++k) {
                const float y=1-static_cast<float>(band[k])/n,phi=2*pi*slice[k]/(4*n);
                const float r=std::sqrt(std::max(0.0f,1-y*y)); normals[k]={r*std::cos(phi),y,r*std::sin(phi)};
                p[k]=normals[k]*body.halfExtent().x;
            }
        }
        const auto base=static_cast<uint32_t>(g.p.size());
        for(int k=0;k<4;++k) { g.p.push_back(p[k]); g.n.push_back(normals[k]); if(scalarField) g.uv.emplace_back(static_cast<float>(index),0); }
        g.indices.insert(g.indices.end(),{base,base+1,base+2,base,base+2,base+3});
    }
    for(int i=0;i<16;++i) if(!geometry[i].p.empty()) {
        GltfPrimitive primitive; auto& g=geometry[i];
        primitive.positionAccessor=appendAccessor(doc,g.p,GltfComponentType::Float,GltfAccessorType::Vec3);
        primitive.normalAccessor=appendAccessor(doc,g.n,GltfComponentType::Float,GltfAccessorType::Vec3);
        if(scalarField) primitive.texCoord0Accessor=appendAccessor(doc,g.uv,GltfComponentType::Float,GltfAccessorType::Vec2);
        primitive.indicesAccessor=appendAccessor(doc,g.indices,GltfComponentType::UnsignedInt,GltfAccessorType::Scalar);
        primitive.materialIndex=i; mesh.primitives.push_back(primitive);
    }
    doc.meshes.push_back(std::move(mesh)); GltfNode node; node.meshIndex=0; doc.nodes.push_back(node);
    GltfScene scene; scene.nodes.push_back(0); doc.scenes.push_back(scene); doc.defaultScene=0;
    return doc;
}

Phantom::Gltf::GltfDocument Phantom::makeCombustibleGltf(const Physics::CombustibleBody& body,int mode)
{ return buildCombustibleGltf(body,mode,false); }

Phantom::Gltf::GltfDocument Phantom::makeCombustibleScalarGltf(const Physics::CombustibleBody& body)
{ return buildCombustibleGltf(body,0,true); }
