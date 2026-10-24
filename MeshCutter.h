#pragma once
// CPU-only triangle-soup cutter. No renderer, input or DirectX dependency.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace meshcut {
struct Vec3 { float x, y, z; };
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x*s,a.y*s,a.z*s}; }
inline float Dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 Cross(Vec3 a, Vec3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
inline Vec3 Normalize(Vec3 v) { float l=std::sqrt(Dot(v,v)); return l>0 ? v*(1/l) : Vec3{0,0,0}; }
template<class P> Vec3 Read(const P& p) { return {p.x,p.y,p.z}; }
template<class P> void Write(P& p, Vec3 v) { p.x=v.x; p.y=v.y; p.z=v.z; }
struct Plane { Vec3 point, normal; };
struct Options {
    float epsilon=0.0001f; // Local-space length tolerance; tune to model scale.
    float separation=1.5f; // Offset of EACH half, matching the original demo.
    float uvScale=0.1f;
};
enum class Status { Unchanged, Cut, InvalidInput, OpenOrAmbiguousContour, UnsupportedHole, TriangulationFailed };
template<class V> struct Result {
    Status status=Status::Unchanged;
    std::vector<V> negative, positive;
};
template<class V> V Interpolate(const V& a, const V& b, float t) {
    V v=a; // Additional vertex attributes require explicit interpolation here.
    Write(v.Position, Read(a.Position)+(Read(b.Position)-Read(a.Position))*t);
    Write(v.Normal, Normalize(Read(a.Normal)+(Read(b.Normal)-Read(a.Normal))*t));
    v.TexCoord.x=a.TexCoord.x+(b.TexCoord.x-a.TexCoord.x)*t;
    v.TexCoord.y=a.TexCoord.y+(b.TexCoord.y-a.TexCoord.y)*t;
    v.Diffuse.x=a.Diffuse.x+(b.Diffuse.x-a.Diffuse.x)*t;
    v.Diffuse.y=a.Diffuse.y+(b.Diffuse.y-a.Diffuse.y)*t;
    v.Diffuse.z=a.Diffuse.z+(b.Diffuse.z-a.Diffuse.z)*t;
    v.Diffuse.w=a.Diffuse.w+(b.Diffuse.w-a.Diffuse.w)*t;
    return v;
}
template<class V> void Triangle(std::vector<V>& out, const V& a, const V& b, const V& c, float e) {
    Vec3 n=Cross(Read(b.Position)-Read(a.Position),Read(c.Position)-Read(a.Position));
    if (Dot(n,n)>e*e*e*e) { out.push_back(a); out.push_back(b); out.push_back(c); }
}
template<class V> void Clip(const V* tri, const float* d, float side, float e, std::vector<V>& out) {
    std::vector<V> polygon;
    for (int i=0;i<3;++i) {
        int j=(i+1)%3;
        bool a=d[i]*side>=0, b=d[j]*side>=0;
        if (a) polygon.push_back(tri[i]);
        if (a!=b) polygon.push_back(Interpolate(tri[i],tri[j],d[i]/(d[i]-d[j])));
    }
    for (std::size_t i=1;i+1<polygon.size();++i) Triangle(out,polygon[0],polygon[i],polygon[i+1],e);
}
struct Point2 { float x,y; };
inline float Orient(Point2 a,Point2 b,Point2 c) { return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x); }
inline bool Contains(Point2 p,const std::vector<Point2>& loop) {
    bool inside=false;
    for (std::size_t i=0,j=loop.size()-1;i<loop.size();j=i++) {
        auto a=loop[i], b=loop[j];
        if ((a.y>p.y)!=(b.y>p.y) && p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x) inside=!inside;
    }
    return inside;
}
// Vertex requires Position/Normal{x,y,z}, TexCoord{x,y}, Diffuse{x,y,z,w}.
// A subset must be a closed manifold triangle soup. Holes fail transactionally.
// accept(point) optionally restricts the cut to the user's screen stroke.
template<class V,class Accept> Result<V> Slice(const std::vector<V>& input, Plane plane, Options opt, Accept accept) {
    Result<V> result;
    const float e=opt.epsilon;
    float length=std::sqrt(Dot(plane.normal,plane.normal));
    if (!(e>0) || !std::isfinite(e) || !(length>e) || !std::isfinite(length) || input.size()%3 ||
        !std::isfinite(opt.separation) || !std::isfinite(opt.uvScale) ||
        !std::isfinite(plane.point.x)||!std::isfinite(plane.point.y)||!std::isfinite(plane.point.z)) {
        result.status=Status::InvalidInput; return result;
    }
    plane.normal=plane.normal*(1/length);
    std::vector<V> nodes;
    std::vector<std::vector<std::size_t>> adjacency;
    auto node=[&](const V& v) {
        for(std::size_t i=0;i<nodes.size();++i) {
            Vec3 delta=Read(nodes[i].Position)-Read(v.Position);
            if(Dot(delta,delta)<=e*e) return i;
        }
        nodes.push_back(v); adjacency.emplace_back(); return nodes.size()-1;
    };
    bool positive=false,negative=false;
    for(std::size_t t=0;t<input.size();t+=3) {
        const V* tri=&input[t]; float d[3];
        for(int i=0;i<3;++i) {
            Vec3 p=Read(tri[i].Position);
            if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) { result.status=Status::InvalidInput;return result; }
            d[i]=Dot(p-plane.point,plane.normal);
            if(std::fabs(d[i])<=e) d[i]=0;
            positive|=d[i]>0; negative|=d[i]<0;
        }
        Clip(tri,d,-1,e,result.negative); Clip(tri,d,1,e,result.positive);
        std::vector<V> hits;
        auto hit=[&](const V& v) {
            for(const auto& h:hits) { Vec3 delta=Read(v.Position)-Read(h.Position); if(Dot(delta,delta)<=e*e) return; }
            hits.push_back(v);
        };
        for(int i=0;i<3;++i) {
            int j=(i+1)%3;
            if(d[i]==0) hit(tri[i]);
            if((d[i]<0&&d[j]>0)||(d[i]>0&&d[j]<0)) hit(Interpolate(tri[i],tri[j],d[i]/(d[i]-d[j])));
        }
        // Fully coplanar triangles are ambiguous for closed-solid slicing.
        if(hits.size()==3) { result.status=Status::OpenOrAmbiguousContour;return result; }
        if(hits.size()==2) {
            if(!accept(Read(hits[0].Position))||!accept(Read(hits[1].Position))) return Result<V>{};
            std::size_t a=node(hits[0]),b=node(hits[1]);
            if(a!=b && std::find(adjacency[a].begin(),adjacency[a].end(),b)==adjacency[a].end()) {
                adjacency[a].push_back(b); adjacency[b].push_back(a);
            }
        }
    }
    if(!positive||!negative||nodes.empty()) return Result<V>{};
    for(const auto& edges:adjacency) if(edges.size()!=2) { result.status=Status::OpenOrAmbiguousContour;return result; }
    Vec3 ref=std::fabs(plane.normal.x)<0.9f?Vec3{1,0,0}:Vec3{0,1,0};
    Vec3 u=Normalize(Cross(ref,plane.normal)),v=Cross(plane.normal,u);
    std::vector<Point2> projected;
    for(const auto& n:nodes) { Vec3 p=Read(n.Position)-plane.point; projected.push_back({Dot(p,u),Dot(p,v)}); }
    std::vector<std::vector<std::size_t>> loops;
    std::vector<bool> visited(nodes.size(),false);
    for(std::size_t start=0;start<nodes.size();++start) if(!visited[start]) {
        std::vector<std::size_t> loop;
        std::size_t previous=nodes.size(),current=start;
        do {
            if(visited[current]) { result.status=Status::OpenOrAmbiguousContour;return result; }
            visited[current]=true;loop.push_back(current);
            const auto& next=adjacency[current];std::size_t chosen=next[0]==previous?next[1]:next[0];
            previous=current;current=chosen;
        } while(current!=start);
        if(loop.size()<3) { result.status=Status::OpenOrAmbiguousContour;return result; }
        loops.push_back(loop);
    }
    // Independent contours are supported; nested contours need hole bridging.
    for(std::size_t i=0;i<loops.size();++i) for(std::size_t j=0;j<loops.size();++j) if(i!=j) {
        std::vector<Point2> polygon;for(auto id:loops[j]) polygon.push_back(projected[id]);
        if(Contains(projected[loops[i][0]],polygon)) { result.status=Status::UnsupportedHole;return result; }
    }
    auto capVertex=[&](std::size_t id,float side) {
        V vertex=nodes[id];Write(vertex.Normal,plane.normal*side);
        vertex.TexCoord.x=projected[id].x*opt.uvScale;vertex.TexCoord.y=projected[id].y*opt.uvScale;
        return vertex;
    };
    auto cap=[&](std::size_t a,std::size_t b,std::size_t c) {
        Triangle(result.negative,capVertex(a,1),capVertex(b,1),capVertex(c,1),e);
        Triangle(result.positive,capVertex(c,-1),capVertex(b,-1),capVertex(a,-1),e);
    };
    for(auto loop:loops) {
        float area=0;for(std::size_t i=0;i<loop.size();++i) { auto a=projected[loop[i]],b=projected[loop[(i+1)%loop.size()]];area+=a.x*b.y-a.y*b.x; }
        if(area<0) std::reverse(loop.begin(),loop.end());
        while(loop.size()>3) {
            bool found=false;
            for(std::size_t i=0;i<loop.size();++i) {
                auto a=loop[(i+loop.size()-1)%loop.size()],b=loop[i],c=loop[(i+1)%loop.size()];
                if(Orient(projected[a],projected[b],projected[c])<=e*e) continue;
                bool occupied=false;
                for(auto p:loop) if(p!=a&&p!=b&&p!=c &&
                    Orient(projected[a],projected[b],projected[p])>=-e*e &&
                    Orient(projected[b],projected[c],projected[p])>=-e*e &&
                    Orient(projected[c],projected[a],projected[p])>=-e*e) { occupied=true;break; }
                if(occupied) continue;
                cap(a,b,c);loop.erase(loop.begin()+i);found=true;break;
            }
            if(!found) { result.status=Status::TriangulationFailed;return result; }
        }
        if(Orient(projected[loop[0]],projected[loop[1]],projected[loop[2]])<=e*e) { result.status=Status::TriangulationFailed;return result; }
        cap(loop[0],loop[1],loop[2]);
    }
    for(auto& p:result.negative) Write(p.Position,Read(p.Position)-plane.normal*opt.separation);
    for(auto& p:result.positive) Write(p.Position,Read(p.Position)+plane.normal*opt.separation);
    result.status=Status::Cut;return result;
}
template<class V> Result<V> Slice(const std::vector<V>& input,Plane plane,Options opt=Options{}) {
    return Slice(input,plane,opt,[](Vec3){return true;});
}
} // namespace meshcut
