#include "MeshCutter.h"

template<class PositionType>
MeshCutter::Vec3 MeshCutter::Read(const PositionType& position)
{
    return {
        position.x,
        position.y,
        position.z
    };
}

template<class VertexType>
bool MeshCutter::InitializePieceIds(const std::vector<std::vector<VertexType>>& inputSubsets, float epsilon, 
    std::vector<std::vector<std::uint64_t>>& pieceIds, std::uint64_t& nextPieceId, bool requireClosed)
{
    // -------------------------------------------------------------
    // 1. 初期モデルの幾何接続を調べる
    // -------------------------------------------------------------
    std::vector<TriangleReference> triangles;

    if (!CollectTriangleReferences(inputSubsets, triangles))
        return false;

    std::vector<Vec3> positions;
    std::vector<TriangleTopology> topology;

    if (!BuildTriangleTopology(inputSubsets, triangles, epsilon, positions, topology))
        return false;

    std::vector<ConnectedComponent> components;

    if (!BuildConnectedComponents(topology, components, requireClosed))
        return false;

    // -------------------------------------------------------------
    // 2. 入力と対応する一時的な番号配列を用意する
    // -------------------------------------------------------------
    std::vector<std::vector<std::uint64_t>> initialIds;
    initialIds.resize(inputSubsets.size());

    for (std::size_t subsetIndex = 0; subsetIndex < inputSubsets.size(); ++subsetIndex)
    {
        const std::size_t triangleCount = inputSubsets[subsetIndex].size() / 3;

        initialIds[subsetIndex].resize(triangleCount, 0);
    }

    // 0は未割り当てとして予約する。
    std::uint64_t availableId = 1;

    const std::uint64_t maximumId = (std::numeric_limits<std::uint64_t>::max)();

    // -------------------------------------------------------------
    // 3. 同じ連結成分の三角形に、共通の番号を設定する
    // -------------------------------------------------------------
    for (const ConnectedComponent& component : components)
    {
        // 割り当て後も、次の番号を保持できる範囲に限定する。
        if (availableId == maximumId)
            return false;

        const std::uint64_t pieceId = availableId;
        ++availableId;

        for (std::size_t triangleIndex : component.triangleIndices)
        {
            const TriangleReference& reference = triangles[triangleIndex];

            const std::size_t subsetTriangleIndex = reference.vertexStart / 3;

            initialIds[reference.subsetIndex][subsetTriangleIndex] = pieceId;
        }
    }

    // 全処理が成功した時点で、出力を確定する。
    pieceIds.swap(initialIds);
    nextPieceId = availableId;

    return true;
}

template<class VertexType>
bool MeshCutter::CollectTriangleReferences(
    const std::vector<std::vector<VertexType>>& inputSubsets,
    std::vector<TriangleReference>& triangles)
{
    triangles.clear();

    // 材質の区切りを維持した入力から、
    // モデル全体の三角形参照を収集する。
    for (std::size_t subsetIndex = 0; subsetIndex < inputSubsets.size(); ++subsetIndex)
    {
        const std::vector<VertexType>& vertices = inputSubsets[subsetIndex];

        // 展開済み三角形として扱えることを確認する。
        if (vertices.size() % 3 != 0)
            return false;

        for (std::size_t vertexStart = 0; vertexStart < vertices.size(); vertexStart += 3)
        {
            // 接続判定に使用する位置が有限であることを確認する。
            for (std::size_t corner = 0; corner < 3; ++corner)
            {
                const Vec3 position = Read(vertices[vertexStart + corner].Position);

                const bool validPosition =
                    std::isfinite(position.x)
                    && std::isfinite(position.y)
                    && std::isfinite(position.z);

                if (!validPosition)
                    return false;
            }

            triangles.push_back(TriangleReference{subsetIndex, vertexStart});
        }
    }

    return true;
}

template<class VertexType>
bool MeshCutter::BuildTriangleTopology(
    const std::vector<std::vector<VertexType>>& inputSubsets,
    const std::vector<TriangleReference>& triangles,
    float epsilon,
    std::vector<Vec3>& positions,
    std::vector<TriangleTopology>& topology)
{
    positions.clear();
    topology.clear();

    if (!std::isfinite(epsilon) || epsilon <= 0.0f)
    {
        return false;
    }

    if (triangles.empty())
    {
        return true;
    }

    // -------------------------------------------------------------
    // 1. 入力全体の最短辺を調べる
    // -------------------------------------------------------------
    double shortestEdgeSquared = (std::numeric_limits<double>::max)();

    // trianglesはCollectTriangleReferencesで検証済みとする。
    for (const TriangleReference& reference : triangles)
    {
        const std::vector<VertexType>& vertices = inputSubsets[reference.subsetIndex];

        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            const std::size_t nextCorner = (corner + 1) % 3;

            const Vec3 current = Read(vertices[reference.vertexStart + corner].Position);
            const Vec3 next = Read(vertices[reference.vertexStart + nextCorner].Position);

            // 短い辺でも距離を計算できるよう、doubleを使用する。
            const double dx = static_cast<double>(current.x) - next.x;
            const double dy = static_cast<double>(current.y) - next.y;
            const double dz = static_cast<double>(current.z) - next.z;

            const double edgeSquared = dx * dx + dy * dy + dz * dz;

            // 座標が完全に一致する辺は、実際に長さがない。
            // 許容誤差による統合とは区別して拒否する。
            if (!std::isfinite(edgeSquared) || edgeSquared <= 0.0)
                return false;

            shortestEdgeSquared = (std::min)(shortestEdgeSquared, edgeSquared);
        }
    }

    // -------------------------------------------------------------
    // 2. 有効な短辺を潰さないよう、接続判定の許容誤差を制限する
    // -------------------------------------------------------------
    const double shortestEdgeLength = std::sqrt(shortestEdgeSquared);

    const double safeTolerance = (std::min)(static_cast<double>(epsilon), shortestEdgeLength * 0.25);

    const float topologyEpsilon = static_cast<float>(safeTolerance);

    // floatで正の許容誤差を表現できない場合は処理しない。
    if (!std::isfinite(topologyEpsilon) || topologyEpsilon <= 0.0f)
        return false;

    // -------------------------------------------------------------
    // 3. 接続判定専用の位置番号を割り当てる
    // -------------------------------------------------------------
    TopologyGrid grid;

    topology.reserve(triangles.size());

    for (const TriangleReference& reference : triangles)
    {
        const std::vector<VertexType>& vertices = inputSubsets[reference.subsetIndex];

        TriangleTopology triangleTopology{};

        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            const VertexType& vertex = vertices[reference.vertexStart + corner];

            const Vec3 position = Read(vertex.Position);

            // 切断判定用epsilonではなく、接続専用の値を使用する。
            if (!FindOrAddTopologyVertex(
                position,
                topologyEpsilon,
                positions,
                grid,
                triangleTopology.vertexIndices[corner]))
            {
                return false;
            }
        }

        topology.push_back(triangleTopology);
    }

    return true;
}

template <class VertexType>
bool MeshCutter::TryGetFlatSurfacePlane(const std::vector<VertexType>& surface,
    const std::vector<VertexType>& reference, float epsilon,
    Plane& plane)
{
    if (surface.empty() || reference.empty())
    {
        return false;
    }
    plane.normal = Normalize(Read(reference.front().Normal));
    if (Dot(plane.normal, plane.normal) < 0.5f)
    {
        return false;
    }

    // 入力が持っていた向きを基準にする。
    // 描画法線だけを信じて、元から逆向きの面を勝手に反転しない。
    for (const VertexType& vertex : reference)
    {
        if (Dot(Normalize(Read(vertex.Normal)), plane.normal) < 0.99999f)
        {
            return false;
        }
    }
    if (!HasValidSurfaceWinding(reference, plane.normal))
    {
        return false;
    }

    plane.point = Read(surface.front().Position);
    for (const VertexType& vertex : surface)
    {
        if (Dot(Normalize(Read(vertex.Normal)), plane.normal) < 0.99999f)
        {
            return false;
        }
        const Vec3 position = Read(vertex.Position);
        const double distance = (static_cast<double>(position.x) - plane.point.x) * plane.normal.x +
            (static_cast<double>(position.y) - plane.point.y) * plane.normal.y +
            (static_cast<double>(position.z) - plane.point.z) * plane.normal.z;
        if (std::abs(distance) > epsilon)
        {
            return false;
        }
    }
    return true;
}

template <class VertexType>
bool MeshCutter::BuildFlatSurfaceData(const std::vector<VertexType>& surface, float epsilon,
    std::vector<VertexType>& nodes,
    std::vector<TriangleTopology>& topology)
{
    const std::vector<std::vector<VertexType>> input{ surface };
    std::vector<TriangleReference> references;
    std::vector<Vec3> positions;
    if (!CollectTriangleReferences(input, references) ||
        !BuildTriangleTopology(input, references, epsilon, positions, topology))
    {
        return false;
    }
    nodes.resize(positions.size());
    std::vector<bool> assigned(nodes.size(), false);
    for (std::size_t triangle = 0; triangle < topology.size(); ++triangle)
    {
        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            const std::size_t node = topology[triangle].vertexIndices[corner];
            const VertexType& vertex = surface[triangle * 3 + corner];
            if (!assigned[node])
            {
                nodes[node] = vertex;
                assigned[node] = true;
            }
        }
    }
    return true;
}

template <class VertexType>
bool MeshCutter::HasAffineSurfaceAttributes(const std::vector<VertexType>& nodes,
    const std::vector<TriangleTopology>& topology,
    const std::vector<Point2>& projected,
    const std::vector<VertexType>& drawingVertices,
    float epsilon)
{
    // 安定した係数を得るため、投影面積が最大の入力三角形を基準にする。
    std::size_t bestTriangle = 0;
    double maximumArea = 0.0;
    for (std::size_t index = 0; index < topology.size(); ++index)
    {
        const TriangleTopology& triangle = topology[index];
        const double area = std::abs(Orient(projected[triangle.vertexIndices[0]],
            projected[triangle.vertexIndices[1]],
            projected[triangle.vertexIndices[2]]));
        if (area > maximumArea)
        {
            maximumArea = area;
            bestTriangle = index;
        }
    }
    if (maximumArea == 0.0)
        return false;

    const TriangleTopology& triangle = topology[bestTriangle];
    const std::size_t indexA = triangle.vertexIndices[0];
    const std::size_t indexB = triangle.vertexIndices[1];
    const std::size_t indexC = triangle.vertexIndices[2];
    const Point2& a = projected[indexA];
    const Point2& b = projected[indexB];
    const Point2& c = projected[indexC];
    const double area = Orient(a, b, c);
    const double deltaUB = static_cast<double>(nodes[indexB].TexCoord.x) - nodes[indexA].TexCoord.x;
    const double deltaUC = static_cast<double>(nodes[indexC].TexCoord.x) - nodes[indexA].TexCoord.x;
    const double deltaVB = static_cast<double>(nodes[indexB].TexCoord.y) - nodes[indexA].TexCoord.y;
    const double deltaVC = static_cast<double>(nodes[indexC].TexCoord.y) - nodes[indexA].TexCoord.y;
    const double gradientU = std::hypot((deltaUB * (c.y - a.y) - deltaUC * (b.y - a.y)) / area,
        ((b.x - a.x) * deltaUC - (c.x - a.x) * deltaUB) / area);
    const double gradientV = std::hypot((deltaVB * (c.y - a.y) - deltaVC * (b.y - a.y)) / area,
        ((b.x - a.x) * deltaVC - (c.x - a.x) * deltaVB) / area);

    // 同一位置にある別の描画頂点も検査する。代表頂点だけではUVシームを見落とす。
    for (std::size_t index = 0; index < drawingVertices.size(); ++index)
    {
        const std::size_t node = topology[index / 3].vertexIndices[index % 3];
        const VertexType& drawingVertex = drawingVertices[index];
        const Point2& point = projected[node];
        const double weightB = ((point.x - a.x) * (c.y - a.y) - (point.y - a.y) * (c.x - a.x)) / area;
        const double weightC = ((b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x)) / area;
        const double u = nodes[indexA].TexCoord.x + weightB * deltaUB + weightC * deltaUC;
        const double v = nodes[indexA].TexCoord.y + weightB * deltaVB + weightC * deltaVC;

        // 位置の統合誤差をUV勾配で換算し、floatの丸め誤差も加える。
        const double rounding = 32.0 * std::numeric_limits<float>::epsilon();
        const double toleranceU = 4.0 * epsilon * gradientU + rounding * (std::max)(1.0, std::abs(u));
        const double toleranceV = 4.0 * epsilon * gradientV + rounding * (std::max)(1.0, std::abs(v));
        if (std::abs(u - drawingVertex.TexCoord.x) > toleranceU ||
            std::abs(v - drawingVertex.TexCoord.y) > toleranceV)
        {
            return false;
        }

        // 色が非線形に変化する面の再分割は行わない。
        const auto& color = drawingVertex.Diffuse;
        const auto& referenceColor = nodes[indexA].Diffuse;
        if (std::abs(color.x - referenceColor.x) > 1.0e-5f ||
            std::abs(color.y - referenceColor.y) > 1.0e-5f ||
            std::abs(color.z - referenceColor.z) > 1.0e-5f ||
            std::abs(color.w - referenceColor.w) > 1.0e-5f)
        {
            return false;
        }
    }
    return true;
}

template <class VertexType>
bool MeshCutter::RepairFlatSurface(std::vector<VertexType>& surface,
    const std::vector<VertexType>& reference, float epsilon)
{
    Plane plane;
    if (!TryGetFlatSurfacePlane(surface, reference, epsilon, plane) ||
        HasValidSurfaceWinding(surface, plane.normal))
    {
        return true;
    }

    // 再切断した既存封口など、属性を維持できる平面だけを修復する。
    std::vector<VertexType> nodes;
    std::vector<TriangleTopology> topology;
    if (!BuildFlatSurfaceData(surface, epsilon, nodes, topology))
    {
        return false;
    }
    const std::vector<Point2> projected = ProjectCutNodesToPlane(nodes, plane);
    if (!HasAffineSurfaceAttributes(nodes, topology, projected, surface, epsilon))
    {
        return false;
    }
    std::vector<std::vector<std::size_t>> loops;
    if (!BuildSurfaceBoundaryLoops(topology, nodes.size(), loops) || loops.empty() ||
        HasNestedContours(loops, projected))
    {
        return false;
    }
    std::vector<TriangleIndices> triangles;
    if (!TriangulateContours(loops, projected, epsilon, triangles))
    {
        return false;
    }
    std::vector<VertexType> rebuilt;
    rebuilt.reserve(triangles.size() * 3);
    for (const TriangleIndices& triangle : triangles)
    {
        rebuilt.push_back(nodes[triangle.indexA]);
        rebuilt.push_back(nodes[triangle.indexB]);
        rebuilt.push_back(nodes[triangle.indexC]);
    }
    if (!HasValidSurfaceWinding(rebuilt, plane.normal))
    {
        return false;
    }
    surface.swap(rebuilt);
    return true;
}

template <class VertexType>
bool MeshCutter::InitializeCutContext(const std::vector<std::vector<VertexType>>& inputSubsets,
    const Plane& plane, float epsilon,
    CutContext<VertexType>& context)
{
    context = CutContext<VertexType>{};
    std::vector<TriangleReference> references;
    std::vector<TriangleTopology> topology;
    if (!CollectTriangleReferences(inputSubsets, references) ||
        !BuildTriangleTopology(inputSubsets, references, epsilon, context.inputPositions, topology))
    {
        return false;
    }

    // 材質ごとの描画頂点に、材質をまたぐ共通位置番号を対応させる。
    context.inputVertexIds.resize(inputSubsets.size());
    for (std::size_t subset = 0; subset < inputSubsets.size(); ++subset)
    {
        context.inputVertexIds[subset].resize(inputSubsets[subset].size());
    }
    for (std::size_t index = 0; index < references.size(); ++index)
    {
        const TriangleReference& reference = references[index];
        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            context.inputVertexIds[reference.subsetIndex][reference.vertexStart + corner] =
                topology[index].vertexIndices[corner];
        }
    }

    context.inputDistances.resize(context.inputPositions.size());
    const double normalSquared = static_cast<double>(plane.normal.x) * plane.normal.x +
        static_cast<double>(plane.normal.y) * plane.normal.y +
        static_cast<double>(plane.normal.z) * plane.normal.z;

    // 距離と平面上の分類は、共通位置ごとに一度だけ計算する。
    for (std::size_t index = 0; index < context.inputPositions.size(); ++index)
    {
        Vec3& position = context.inputPositions[index];
        double distance = (static_cast<double>(position.x) - plane.point.x) * plane.normal.x +
            (static_cast<double>(position.y) - plane.point.y) * plane.normal.y +
            (static_cast<double>(position.z) - plane.point.z) * plane.normal.z;
        if (std::abs(distance) <= epsilon)
        {
            // 分類だけを0にせず、原表面と封口が共有する位置も平面へそろえる。
            position = { static_cast<float>(position.x - distance / normalSquared * plane.normal.x),
                        static_cast<float>(position.y - distance / normalSquared * plane.normal.y),
                        static_cast<float>(position.z - distance / normalSquared * plane.normal.z) };
            distance = 0.0;
        }
        context.inputDistances[index] = distance;
        context.hasPositive |= distance > 0.0;
        context.hasNegative |= distance < 0.0;
    }
    return true;
}

template <class VertexType>
std::size_t MeshCutter::FindOrAddSharedCutNode(IndexPair key, const VertexType& vertex,
    float epsilon, CutContext<VertexType>& context)
{
    const auto found = context.cutNodeLookup.find(key);
    if (found != context.cutNodeLookup.end())
    {
        return found->second;
    }
    const std::size_t node = FindOrAddCutNode(vertex, epsilon, context.nodes, context.adjacency);
    context.cutNodeLookup.emplace(key, node);
    return node;
}

template <class VertexType>
VertexType MeshCutter::InterpolateSharedEdge(const VertexType& a, const VertexType& b,
    std::size_t indexA, std::size_t indexB, float epsilon,
    CutContext<VertexType>& context,
    std::size_t& nodeIndex)
{
    const double distanceA = context.inputDistances[indexA];
    const double distanceB = context.inputDistances[indexB];
    const double t = distanceA / (distanceA - distanceB);

    // UV、法線、色は各描画頂点から補間する。材質境界の属性は統合しない。
    VertexType vertex = Interpolate(a, b, static_cast<float>(t));
    const IndexPair key = std::minmax(indexA, indexB);
    const auto found = context.cutNodeLookup.find(key);
    if (found == context.cutNodeLookup.end())
    {
        // 位置だけは常に同じ端点順とdoubleの係数で計算する。
        const Vec3& start = context.inputPositions[key.first];
        const Vec3& end = context.inputPositions[key.second];
        const double startDistance = context.inputDistances[key.first];
        const double endDistance = context.inputDistances[key.second];
        const double coefficient = startDistance / (startDistance - endDistance);
        const Vec3 position{
            static_cast<float>(start.x + (static_cast<double>(end.x) - start.x) * coefficient),
            static_cast<float>(start.y + (static_cast<double>(end.y) - start.y) * coefficient),
            static_cast<float>(start.z + (static_cast<double>(end.z) - start.z) * coefficient) };
        Write(vertex.Position, position);
        nodeIndex = FindOrAddSharedCutNode(key, vertex, epsilon, context);
    }
    else
    {
        nodeIndex = found->second;
    }

    // 近接交点が統合された場合も、原表面と封口に同じ代表位置を書き込む。
    Write(vertex.Position, Read(context.nodes[nodeIndex].Position));
    return vertex;
}

template <class VertexType>
void MeshCutter::AppendClippedTriangle(const VertexType* triangle, const double* distances,
    const VertexType* intersections, const bool* crossed,
    int side, std::vector<VertexType>& output)
{
    std::vector<VertexType> polygon;
    polygon.reserve(4);
    for (std::size_t corner = 0; corner < 3; ++corner)
    {
        if (distances[corner] * side >= 0.0)
        {
            polygon.push_back(triangle[corner]);
        }
        if (crossed[corner])
        {
            // 両側で同じ補間済み頂点を使用し、ここでは交点を再計算しない。
            polygon.push_back(intersections[corner]);
        }
    }
    for (std::size_t index = 1; index + 1 < polygon.size(); ++index)
    {
        Triangle(output, polygon[0], polygon[index], polygon[index + 1]);
    }
}

template <class VertexType>
bool MeshCutter::ClipSubset(const std::vector<VertexType>& input,
    const std::vector<std::size_t>& inputIds, float epsilon,
    SurfaceResult<VertexType>& surface, CutContext<VertexType>& context, Status& failureStatus)
{
    failureStatus = Status::Unchanged;
    if (input.size() % 3 != 0 || inputIds.size() != input.size())
    {
        failureStatus = Status::InvalidInput;
        return false;
    }
    surface.negative.clear();
    surface.positive.clear();

    for (std::size_t start = 0; start < input.size(); start += 3)
    {
        VertexType triangle[3] = { input[start], input[start + 1], input[start + 2] };
        VertexType intersections[3];
        double distances[3];
        std::size_t ids[3];
        bool crossed[3] = {};
        std::vector<std::size_t> hits;
        hits.reserve(3);

        // 1. 全材質で共通の位置と分類を読み込む。
        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            ids[corner] = inputIds[start + corner];
            distances[corner] = context.inputDistances[ids[corner]];
            Write(triangle[corner].Position, context.inputPositions[ids[corner]]);
        }

        // 2. 平面上の頂点と、厳密に異符号の辺の交点を登録する。
        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            const std::size_t next = (corner + 1) % 3;
            if (distances[corner] == 0.0)
            {
                const std::size_t node = FindOrAddSharedCutNode({ ids[corner], ids[corner] },
                    triangle[corner], epsilon, context);
                AppendUniqueIndex(hits, node);
            }
            crossed[corner] = (distances[corner] < 0.0 && distances[next] > 0.0) ||
                (distances[corner] > 0.0 && distances[next] < 0.0);
            if (crossed[corner])
            {
                std::size_t node;
                intersections[corner] =
                    InterpolateSharedEdge(triangle[corner], triangle[next], ids[corner], ids[next],
                        epsilon, context, node);
                AppendUniqueIndex(hits, node);
            }
        }

        // 平面上の元頂点も、交点と同じ代表位置へそろえる。
        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            if (distances[corner] == 0.0)
            {
                const std::size_t node = context.cutNodeLookup.at({ ids[corner], ids[corner] });
                Write(triangle[corner].Position, Read(context.nodes[node].Position));
            }
        }

        // 3. 共有する交点から、原表面の両側を作成する。
        AppendClippedTriangle(triangle, distances, intersections, crossed, -1, surface.negative);
        AppendClippedTriangle(triangle, distances, intersections, crossed, 1, surface.positive);
        if (hits.size() == 3)
        {
            // 共面三角形の一般的な処理は、この版では行わない。
            failureStatus = Status::OpenOrAmbiguousContour;
            return false;
        }
        if (hits.size() != 2)
        {
            continue;
        }

        // 4. 接触辺か、本当に両側を分ける辺かを、全隣接面を集めた後で判定する。
        unsigned sides = 0;
        for (double distance : distances)
        {
            if (distance > 0.0)
            {
                sides |= PositiveSideMask;
            }
            if (distance < 0.0)
            {
                sides |= NegativeSideMask;
            }
        }
        context.contourEdges[std::minmax(hits[0], hits[1])] |= sides;
    }
    return true;
}

template <class VertexType, class Accept>
bool MeshCutter::BuildContourAdjacency(CutContext<VertexType>& context, Accept& accept)
{
    for (const auto& edge : context.contourEdges)
    {
        if (edge.second != BothSidesMask)
        {
            // 両隣の面が同じ側にある辺は、平面との接触であり切断輪郭ではない。
            continue;
        }
        const std::size_t a = edge.first.first;
        const std::size_t b = edge.first.second;
        if (!accept(Read(context.nodes[a].Position)) || !accept(Read(context.nodes[b].Position)))
        {
            return false;
        }
        context.adjacency[a].push_back(b);
        context.adjacency[b].push_back(a);
    }
    return true;
}

template <class VertexType>
bool MeshCutter::HasValidSurfaceWinding(const std::vector<VertexType>& vertices, Vec3 normal)
{
    for (std::size_t index = 0; index < vertices.size(); index += 3)
    {
        const Vec3 a = Read(vertices[index].Position);
        const Vec3 b = Read(vertices[index + 1].Position);
        const Vec3 c = Read(vertices[index + 2].Position);
        const double abX = static_cast<double>(b.x) - a.x;
        const double abY = static_cast<double>(b.y) - a.y;
        const double abZ = static_cast<double>(b.z) - a.z;
        const double acX = static_cast<double>(c.x) - a.x;
        const double acY = static_cast<double>(c.y) - a.y;
        const double acZ = static_cast<double>(c.z) - a.z;
        const double facing = (abY * acZ - abZ * acY) * normal.x +
            (abZ * acX - abX * acZ) * normal.y +
            (abX * acY - abY * acX) * normal.z;
        if (!(facing > 0.0))
        {
            return false;
        }
    }
    return true;
}

template <class VertexType>
bool MeshCutter::RepairMovedCaps(SurfaceResult<VertexType>& caps,
    const std::vector<VertexType>& nodes,
    const std::vector<std::vector<std::size_t>>& loops,
    const std::vector<Point2>& originalProjected, const Plane& plane,
    const Options& options)
{
    for (bool positive : {false, true})
    {
        std::vector<VertexType>& cap = positive ? caps.positive : caps.negative;
        const Vec3 outward = plane.normal * (positive ? -1.0f : 1.0f);
        if (HasValidSurfaceWinding(cap, outward))
            continue;

        // floatで移動した後の座標を使い、この側だけ再分割する。
        std::vector<VertexType> moved = nodes;
        const Vec3 displacement =
            plane.normal * (positive ? options.separation : -options.separation);
        for (VertexType& vertex : moved)
        {
            Write(vertex.Position, Read(vertex.Position) + displacement);
        }
        Plane movedPlane = plane;
        movedPlane.point = movedPlane.point + displacement;
        const std::vector<Point2> projected = ProjectCutNodesToPlane(moved, movedPlane);
        std::vector<TriangleIndices> triangles;
        if (!TriangulateContours(loops, projected, options.epsilon, triangles))
        {
            return false;
        }

        std::vector<VertexType> rebuilt;
        rebuilt.reserve(triangles.size() * 3);
        for (const TriangleIndices& triangle : triangles)
        {
            const std::size_t order[3] = { triangle.indexA,
                                          positive ? triangle.indexC : triangle.indexB,
                                          positive ? triangle.indexB : triangle.indexC };
            for (std::size_t index : order)
            {
                // UVの原点は、移動前の切断平面に維持する。
                rebuilt.push_back(CreateCapVertex(moved[index], originalProjected[index], outward,
                    options.uvScale));
            }
        }
        if (!HasValidSurfaceWinding(rebuilt, outward))
        {
            return false;
        }
        cap.swap(rebuilt);
    }
    return true;
}

template <class VertexType>
double MeshCutter::MinimumRefinementEpsilon(const std::vector<std::vector<VertexType>>& input,
    const Plane& plane, float separation)
{
    double magnitude = 0.0;
    for (const auto& subset : input)
    {
        for (const VertexType& vertex : subset)
        {
            const Vec3 position = Read(vertex.Position);
            magnitude = (std::max)({ magnitude, std::abs(static_cast<double>(position.x)),
                                    std::abs(static_cast<double>(position.y)),
                                    std::abs(static_cast<double>(position.z)) });
        }
    }
    const double inputMagnitude = magnitude;
    magnitude += std::abs(static_cast<double>(separation));
    if (magnitude >= (std::numeric_limits<float>::max)())
    {
        return std::numeric_limits<double>::infinity();
    }

    // floatで保存される頂点と、分離後の座標の間隔を基準にする。
    // 平面上の基準点を遠くへずらしただけで、この間隔を大きくしない。
    const float value = static_cast<float>(magnitude);
    const float next = std::nextafter(value, std::numeric_limits<float>::infinity());
    const double positionFloor = 4.0 * (static_cast<double>(next) - value);

    // 距離のdouble演算については、各項の大きさから別の下限を見積もる。
    const double distanceScale =
        (inputMagnitude + std::abs(static_cast<double>(plane.point.x))) * std::abs(plane.normal.x) +
        (inputMagnitude + std::abs(static_cast<double>(plane.point.y))) * std::abs(plane.normal.y) +
        (inputMagnitude + std::abs(static_cast<double>(plane.point.z))) * std::abs(plane.normal.z);
    const double arithmeticFloor = 16.0 * std::numeric_limits<double>::epsilon() * distanceScale;
    return (std::max)(positionFloor, arithmeticFloor);
}

template <class VertexType, class Accept>
MeshCutter::ModelResult<VertexType> MeshCutter::SliceComponent(
    const std::vector<std::vector<VertexType>>& input, const Plane& plane, const Options& options,
    Accept& accept, std::uint64_t& nextPieceId)
{
    const double refinementFloor = MinimumRefinementEpsilon(input, plane, options.separation);
    Options effective = options;
    ModelResult<VertexType> result;
    constexpr unsigned maximumRefinements = 2;

    for (unsigned attempt = 0; attempt <= maximumRefinements; ++attempt)
    {
        // 失敗した試行で、呼び出し元の破片番号を消費しない。
        std::uint64_t stagedNext = nextPieceId;
        result = SliceComponentAttempt(input, plane, effective, accept, stagedNext);
        if (result.status == Status::Cut)
        {
            nextPieceId = stagedNext;
            return result;
        }
        if (result.status != Status::OpenOrAmbiguousContour &&
            result.status != Status::TriangulationFailed)
        {
            return result;
        }

        const float smaller = effective.epsilon * 0.5f;
        if (attempt == maximumRefinements || smaller <= 0.0f || smaller == effective.epsilon ||
            smaller < refinementFloor)
        {
            return result;
        }
        effective.epsilon = smaller;
        // 再試行の新しい一時配列を作る前に、失敗結果のメモリを解放する。
        result = ModelResult<VertexType>{};
    }
    return result;
}

template <class VertexType, class Accept>
MeshCutter::ModelResult<VertexType> MeshCutter::SliceComponentAttempt(
    const std::vector<std::vector<VertexType>>& inputSubsets, const Plane& plane,
    const Options& options, Accept& accept, std::uint64_t& nextPieceId)
{
    ModelResult<VertexType> result;
    result.subsets.resize(inputSubsets.size());
    const float epsilon = options.epsilon;

    // 1. 原表面と封口で共有する位置、距離、交点キャッシュを準備する。
    CutContext<VertexType> context;
    if (!InitializeCutContext(inputSubsets, plane, epsilon, context))
    {
        result.status = Status::InvalidInput;
        return result;
    }
    if (!context.hasPositive || !context.hasNegative)
    {
        return ModelResult<VertexType>{};
    }

    // 2. 材質単位で原表面を切り分け、モデル共通の交線を収集する。
    for (std::size_t index = 0; index < inputSubsets.size(); ++index)
    {
        Status failure = Status::Unchanged;
        if (!ClipSubset(inputSubsets[index], context.inputVertexIds[index], epsilon,
            result.subsets[index], context, failure))
        {
            ModelResult<VertexType> stopped;
            stopped.status = failure;
            return stopped;
        }
    }
    if (!BuildContourAdjacency(context, accept))
    {
        return ModelResult<VertexType>{};
    }

    // 3. 実際に両側を分ける辺だけから閉輪郭を作る。
    std::vector<std::vector<std::size_t>> loops;
    if (!BuildClosedContours(context.adjacency, loops))
    {
        result.status = Status::OpenOrAmbiguousContour;
        return result;
    }
    if (loops.empty())
    {
        return ModelResult<VertexType>{};
    }
    const std::vector<Point2> projected = ProjectCutNodesToPlane(context.nodes, plane);
    if (HasNestedContours(loops, projected))
    {
        result.status = Status::UnsupportedHole;
        return result;
    }

    // 4. 境界分割を維持した三角形群から、両側の封口を作る。
    std::vector<TriangleIndices> capTriangles;
    if (!TriangulateContours(loops, projected, epsilon, capTriangles))
    {
        result.status = Status::TriangulationFailed;
        return result;
    }
    AppendCapTriangles(result.caps, context.nodes, projected, capTriangles, plane, options);

    // 5. 実際のfloat座標へ分離移動を適用してから、面の向きを再確認する。
    for (auto& surface : result.subsets)
    {
        ApplySeparation(surface, plane, options.separation);
    }
    ApplySeparation(result.caps, plane, options.separation);
    if (!RepairMovedCaps(result.caps, context.nodes, loops, projected, plane, options))
    {
        result.status = Status::TriangulationFailed;
        return result;
    }
    for (std::size_t index = 0; index < result.subsets.size(); ++index)
    {
        if (!RepairFlatSurface(result.subsets[index].negative, inputSubsets[index], epsilon) ||
            !RepairFlatSurface(result.subsets[index].positive, inputSubsets[index], epsilon))
        {
            ModelResult<VertexType> stopped;
            stopped.status = Status::TriangulationFailed;
            return stopped;
        }
    }

    // 6. 最終座標の閉合を確認した後で、新しい破片番号を確定する。
    if (!AssignNewPieceIds(result, epsilon, nextPieceId))
    {
        ModelResult<VertexType> stopped;
        stopped.status = Status::OpenOrAmbiguousContour;
        return stopped;
    }
    result.status = Status::Cut;
    return result;
}

template <class VertexType, class Accept>
MeshCutter::ModelResult<VertexType> MeshCutter::SliceModel(
    const std::vector<std::vector<VertexType>>& inputSubsets, Plane plane, Options options,
    Accept accept, const std::vector<std::vector<std::uint64_t>>& inputPieceIds,
    std::uint64_t nextPieceId)
{
    ModelResult<VertexType> result;

    // -----------------------------------------------------------------
    // 1. 全成分で共通の平面と設定を確認する
    // -----------------------------------------------------------------
    const float epsilon = options.epsilon;

    const float normalLength = std::sqrt(Dot(plane.normal, plane.normal));

    const bool validEpsilon = std::isfinite(epsilon) && epsilon > 0.0f;

    const bool validNormal = std::isfinite(normalLength) && normalLength > epsilon;

    const bool validOptions = std::isfinite(options.separation) && std::isfinite(options.uvScale);

    const bool validPlanePoint = std::isfinite(plane.point.x) && std::isfinite(plane.point.y) &&
        std::isfinite(plane.point.z);

    if (!validEpsilon || !validNormal || !validOptions || !validPlanePoint)
    {
        result.status = Status::InvalidInput;
        return result;
    }

    // 正規化は、モデル全体で一度だけ行う。
    plane.normal = plane.normal * (1.0f / normalLength);

    // -----------------------------------------------------------------
    // 2. 三角形の参照情報を準備する
    // -----------------------------------------------------------------
    std::vector<TriangleReference> triangles;

    if (!CollectTriangleReferences(inputSubsets, triangles))
    {
        result.status = Status::InvalidInput;
        return result;
    }

    // この関数内で進める番号。
    // 元モデルのカウンターは、更新成功時まで変更しない。
    std::uint64_t availableId = nextPieceId;

    std::vector<std::vector<std::uint64_t>> initializedIds;

    const std::vector<std::vector<std::uint64_t>>* activeIds = &inputPieceIds;

    // -----------------------------------------------------------------
    // 3. 新規モデルだけ、幾何接続から初期番号を生成する
    // -----------------------------------------------------------------
    if (inputPieceIds.empty())
    {
        if (!InitializePieceIds(inputSubsets, epsilon, initializedIds, availableId))
        {
            result.status = Status::OpenOrAmbiguousContour;
            return result;
        }

        activeIds = &initializedIds;
    }

    // 番号配列の外側は、入力サブセットと対応する。
    if (activeIds->size() != inputSubsets.size() || availableId == 0)
    {
        result.status = Status::InvalidInput;
        return result;
    }

    for (std::size_t subsetIndex = 0; subsetIndex < inputSubsets.size(); ++subsetIndex)
    {
        const std::vector<std::uint64_t>& subsetIds = (*activeIds)[subsetIndex];

        const std::size_t triangleCount = inputSubsets[subsetIndex].size() / 3;

        if (subsetIds.size() != triangleCount)
        {
            result.status = Status::InvalidInput;
            return result;
        }

        // 次の番号は、保存済み番号より大きい必要がある。
        for (std::uint64_t pieceId : subsetIds)
        {
            if (pieceId == 0 || pieceId >= availableId)
            {
                result.status = Status::InvalidInput;
                return result;
            }
        }
    }

    // -----------------------------------------------------------------
    // 4. 保存済み番号から処理対象の破片をまとめる
    // -----------------------------------------------------------------
    std::vector<ConnectedComponent> components;

    if (!BuildComponentsFromPieceIds(triangles, *activeIds, components))
    {
        result.status = Status::InvalidInput;
        return result;
    }

    result.subsets.resize(inputSubsets.size());
    result.unchangedSubsets.resize(inputSubsets.size());
    result.unchangedPieceIds.resize(inputSubsets.size());

    bool hasChanges = false;

    // -----------------------------------------------------------------
    // 5. 各破片を独立して切断する
    // -----------------------------------------------------------------
    for (const ConnectedComponent& component : components)
    {
        std::vector<std::vector<VertexType>> componentInput;
        componentInput.resize(inputSubsets.size());

        for (std::size_t triangleIndex : component.triangleIndices)
        {
            const TriangleReference& reference = triangles[triangleIndex];

            const std::vector<VertexType>& source = inputSubsets[reference.subsetIndex];

            std::vector<VertexType>& destination = componentInput[reference.subsetIndex];

            const auto triangleBegin = source.begin() + reference.vertexStart;

            destination.insert(destination.end(), triangleBegin, triangleBegin + 3);
        }

        const auto componentResult =
            SliceComponent(componentInput, plane, options, accept, availableId);

        if (componentResult.status == Status::Unchanged)
        {
            // ---------------------------------------------------------
            // 未切断の破片は、位置・属性・番号を維持する
            // ---------------------------------------------------------
            for (std::size_t subsetIndex = 0; subsetIndex < componentInput.size(); ++subsetIndex)
            {
                const std::vector<VertexType>& source = componentInput[subsetIndex];

                std::vector<VertexType>& destination = result.unchangedSubsets[subsetIndex];

                destination.insert(destination.end(), source.begin(), source.end());

                std::vector<std::uint64_t>& destinationIds = result.unchangedPieceIds[subsetIndex];

                destinationIds.insert(destinationIds.end(), source.size() / 3, component.pieceId);
            }

            continue;
        }

        if (componentResult.status != Status::Cut)
        {
            // 一部の破片だけを提出せず、元モデルを維持する。
            ModelResult<VertexType> stoppedResult;
            stoppedResult.status = componentResult.status;

            return stoppedResult;
        }

        hasChanges = true;

        // -------------------------------------------------------------
        // 切断結果は、頂点と番号を組にして追加する
        // -------------------------------------------------------------
        for (std::size_t subsetIndex = 0; subsetIndex < componentResult.subsets.size(); ++subsetIndex)
        {
            if (!AppendSurface(result.subsets[subsetIndex], componentResult.subsets[subsetIndex]))
            {
                ModelResult<VertexType> stoppedResult;
                stoppedResult.status = Status::InvalidInput;

                return stoppedResult;
            }
        }

        if (!AppendSurface(result.caps, componentResult.caps))
        {
            ModelResult<VertexType> stoppedResult;
            stoppedResult.status = Status::InvalidInput;

            return stoppedResult;
        }
    }

    // 全破片が未変更なら、モデル更新は不要。
    if (!hasChanges)
    {
        return ModelResult<VertexType>{};
    }

    // 幾何と一緒に採用する、更新後の番号カウンター。
    result.nextPieceId = availableId;

    result.status = Status::Cut;
    return result;
}

template<class PositionType>
void MeshCutter::Write(PositionType& position, Vec3 value)
{
    position.x = value.x;
    position.y = value.y;
    position.z = value.z;
}

template<class VertexType>
VertexType MeshCutter::Interpolate(const VertexType& a, const VertexType& b, float t)
{
    // 補間係数が端点を表す場合、元の頂点をそのまま返す。
    // 浮動小数点演算によって、端点と異なる位置を作らない。
    if (t == 0.0f)
        return a;
    if (t == 1.0f)
        return b;

    // 明示的に補間しない属性は、ひとまずaから引き継ぐ。
    VertexType result = a;

    // 位置を線形補間する。
    const Vec3 positionA = Read(a.Position);
    const Vec3 positionB = Read(b.Position);
    const Vec3 interpolatedPosition = positionA + (positionB - positionA) * t;
    Write(result.Position, interpolatedPosition);

    // 法線方向を線形補間する。
    const Vec3 normalA = Read(a.Normal);
    const Vec3 normalB = Read(b.Normal);
    const Vec3 interpolatedNormal = normalA + (normalB - normalA) * t;

    // 線形補間後の法線は、単位長とは限らない。
    Write(result.Normal, Normalize(interpolatedNormal));

    // テクスチャ座標を線形補間する。
    result.TexCoord.x = a.TexCoord.x + (b.TexCoord.x - a.TexCoord.x) * t;
    result.TexCoord.y = a.TexCoord.y + (b.TexCoord.y - a.TexCoord.y) * t;

    // 頂点色の各成分を線形補間する。
    result.Diffuse.x = a.Diffuse.x + (b.Diffuse.x - a.Diffuse.x) * t;
    result.Diffuse.y = a.Diffuse.y + (b.Diffuse.y - a.Diffuse.y) * t;
    result.Diffuse.z = a.Diffuse.z + (b.Diffuse.z - a.Diffuse.z) * t;
    result.Diffuse.w = a.Diffuse.w + (b.Diffuse.w - a.Diffuse.w) * t;

    return result;
}

template <class VertexType>
void MeshCutter::Triangle(std::vector<VertexType>& output, const VertexType& a, 
    const VertexType& b, const VertexType& c)
{
    const Vec3 positionA = Read(a.Position);
    const Vec3 positionB = Read(b.Position);
    const Vec3 positionC = Read(c.Position);
    const double abX = static_cast<double>(positionB.x) - positionA.x;
    const double abY = static_cast<double>(positionB.y) - positionA.y;
    const double abZ = static_cast<double>(positionB.z) - positionA.z;
    const double acX = static_cast<double>(positionC.x) - positionA.x;
    const double acY = static_cast<double>(positionC.y) - positionA.y;
    const double acZ = static_cast<double>(positionC.z) - positionA.z;
    const double crossX = abY * acZ - abZ * acY;
    const double crossY = abZ * acX - abX * acZ;
    const double crossZ = abX * acY - abY * acX;
    const double areaSquared = crossX * crossX + crossY * crossY + crossZ * crossZ;

    // 位置統合で実際に潰れた面だけを除く。
    // 長さepsilonと面積だけを比較して、有効な短辺を持つ面を独立に削除しない。
    // 最終的な閉合と向きは、封口・移動後にも確認する。
    if (std::isfinite(areaSquared) && areaSquared > 0.0)
    {
        output.push_back(a);
        output.push_back(b);
        output.push_back(c);
    }
}

template<class VertexType>
std::size_t MeshCutter::FindOrAddCutNode(const VertexType& vertex, float epsilon, 
    std::vector<VertexType>& nodes, std::vector<std::vector<std::size_t>>& adjacency)
{
    const Vec3 position = Read(vertex.Position);

    // 平方根を使わずに距離を比較する。
    const float epsilonSquared = epsilon * epsilon;

    // 既存の交点を順番に調べ、許容誤差以内の位置があればその番号を返す。
    for (std::size_t index = 0; index < nodes.size(); ++index)
    {
        const Vec3 existingPosition = Read(nodes[index].Position);
        const Vec3 difference = existingPosition - position;
        const float distanceSquared = Dot(difference, difference);

        // 許容誤差以内の位置は、同じ交点として扱う。
        if (distanceSquared <= epsilonSquared)
            return index;
    }

    // 対応する交点がなかったため、新しいノードを作成する。
    const std::size_t newIndex = nodes.size();

    nodes.push_back(vertex);

    // nodesと同じ番号で参照できるよう、隣接リストも追加する。
    adjacency.emplace_back();

    return newIndex;
}

template<class VertexType>
std::vector<MeshCutter::Point2> MeshCutter::ProjectCutNodesToPlane(
    const std::vector<VertexType>& nodes, const Plane& plane)
{
    // 法線とほぼ平行にならない参考軸を選ぶ。
    Vec3 referenceAxis;

    if (std::fabs(plane.normal.x) < 0.9f)
        referenceAxis = { 1.0f, 0.0f, 0.0f };
    else
        referenceAxis = { 0.0f, 1.0f, 0.0f };

    // 切断平面内の第1軸を作成する。
    const Vec3 axisU = Normalize(Cross(referenceAxis, plane.normal));

    // 法線と第1軸に垂直な、第2軸を作成する。
    // plane.normalとaxisUは単位ベクトルなので、
    // この軸も単位長になる。
    const Vec3 axisV = Cross(plane.normal, axisU);

    std::vector<Point2> projected;

    for (const VertexType& node : nodes)
    {
        // 平面上の基準点を、2次元座標系の原点として使用する。
        const Vec3 position = Read(node.Position);
        const double offsetX = static_cast<double>(position.x) - plane.point.x;
        const double offsetY = static_cast<double>(position.y) - plane.point.y;
        const double offsetZ = static_cast<double>(position.z) - plane.point.z;

        Point2 point;

        // 各軸方向の成分を求める。
        point.x = offsetX * axisU.x + offsetY * axisU.y + offsetZ * axisU.z;
        point.y = offsetX * axisV.x + offsetY * axisV.y + offsetZ * axisV.z;

        projected.push_back(point);
    }

    return projected;
}

template<class VertexType>
VertexType MeshCutter::CreateCapVertex(const VertexType& source, 
    const Point2& planePosition, const Vec3& outwardNormal, float uvScale)
{
    // 位置、頂点色、その他の属性は交点から引き継ぐ。
    VertexType vertex = source;

    // 切断面の外向き法線を設定する。
    Write(vertex.Normal, outwardNormal);

    // 切断平面上の2次元座標からUVを生成する。
    vertex.TexCoord.x = static_cast<float>(planePosition.x * uvScale);
    vertex.TexCoord.y = static_cast<float>(planePosition.y * uvScale);

    return vertex;
}

template<class VertexType>
void MeshCutter::AppendCapTriangles(SurfaceResult<VertexType>& result, const std::vector<VertexType>& nodes,
    const std::vector<Point2>& projected, const std::vector<TriangleIndices>& capTriangles, 
    const Plane& plane, const Options& options)
{
    // 負側の切断面は、平面法線の正方向を外側とする。
    const Vec3 negativeSideNormal = plane.normal;

    // 正側では、外向き法線が反対になる。
    const Vec3 positiveSideNormal = plane.normal * -1.0f;

    // 耳切りで決定したノードの組み合わせから、
    // 両側の封口三角形を生成する。
    for (const TriangleIndices& triangle : capTriangles)
    {
        const std::size_t indexA = triangle.indexA;
        const std::size_t indexB = triangle.indexB;
        const std::size_t indexC = triangle.indexC;

        // 負側の封口頂点を作成する。
        const VertexType negativeA = CreateCapVertex(
            nodes[indexA],
            projected[indexA],
            negativeSideNormal,
            options.uvScale);

        const VertexType negativeB = CreateCapVertex(
            nodes[indexB],
            projected[indexB],
            negativeSideNormal,
            options.uvScale);

        const VertexType negativeC = CreateCapVertex(
            nodes[indexC],
            projected[indexC],
            negativeSideNormal,
            options.uvScale);

        // 正側の封口頂点を作成する。
        // 位置は同じだが、法線は負側と反対になる。
        const VertexType positiveA = CreateCapVertex(
            nodes[indexA],
            projected[indexA],
            positiveSideNormal,
            options.uvScale);

        const VertexType positiveB = CreateCapVertex(
            nodes[indexB],
            projected[indexB],
            positiveSideNormal,
            options.uvScale);

        const VertexType positiveC = CreateCapVertex(
            nodes[indexC],
            projected[indexC],
            positiveSideNormal,
            options.uvScale);

        // 負側は耳切りで得た順序を使用する。
        Triangle(result.negative, negativeA, negativeB, negativeC);

        // 正側は逆順にして、幾何法線も反転させる。
        Triangle(result.positive, positiveC, positiveB, positiveA);
    }
}

template<class VertexType>
void MeshCutter::ApplySeparation(SurfaceResult<VertexType>& surface, const Plane& plane, float separation)
{
    // 負側は、平面法線と反対方向へ移動する。
    for (VertexType& vertex : surface.negative)
    {
        const Vec3 position = Read(vertex.Position);

        Write(vertex.Position, position - plane.normal * separation);
    }

    // 正側は、平面法線の方向へ移動する。
    for (VertexType& vertex : surface.positive)
    {
        const Vec3 position = Read(vertex.Position);

        Write(vertex.Position, position + plane.normal * separation);
    }
}

template<class VertexType>
bool MeshCutter::AppendSurface(SurfaceResult<VertexType>& destination, const SurfaceResult<VertexType>& source)
{
    // 追加前に、両方の出力が完全な三角形と番号の組であるか確認する。
    const bool validSource = source.negative.size() % 3 == 0 && source.positive.size() % 3 == 0 &&
        source.negativePieceIds.size() == source.negative.size() / 3 &&
        source.positivePieceIds.size() == source.positive.size() / 3;

    const bool validDestination =
        destination.negative.size() % 3 == 0 && destination.positive.size() % 3 == 0 &&
        destination.negativePieceIds.size() == destination.negative.size() / 3 &&
        destination.positivePieceIds.size() == destination.positive.size() / 3;

    if (!validSource || !validDestination)
        return false;

    // 負側の三角形と番号を、同じ順序で追加する。
    destination.negative.insert(destination.negative.end(), source.negative.begin(),
        source.negative.end());

    destination.negativePieceIds.insert(destination.negativePieceIds.end(),
        source.negativePieceIds.begin(),
        source.negativePieceIds.end());

    // 正側の三角形と番号も、同じ順序で追加する。
    destination.positive.insert(destination.positive.end(), source.positive.begin(),
        source.positive.end());

    destination.positivePieceIds.insert(destination.positivePieceIds.end(),
        source.positivePieceIds.begin(),
        source.positivePieceIds.end());

    return true;
}

template<class VertexType>
bool MeshCutter::AssignSidePieceIds(ModelResult<VertexType>& componentResult, 
    bool positiveSide, float epsilon, std::uint64_t& nextPieceId)
{
    if (nextPieceId == 0)
        return false;

    // -------------------------------------------------------------
    // 1. 現在の側について、全材質の原表面と封口を集める
    // -------------------------------------------------------------
    std::vector<std::vector<VertexType>> sideInput;
    sideInput.reserve(componentResult.subsets.size() + 1);

    for (const SurfaceResult<VertexType>& surface : componentResult.subsets)
    {
        // 材質番号との対応を維持するため、空配列も追加する。
        sideInput.push_back(positiveSide ? surface.positive : surface.negative);
    }

    // 最後の配列には、同じ側の封口を追加する。
    sideInput.push_back(positiveSide ? componentResult.caps.positive
                                    : componentResult.caps.negative);

    // -------------------------------------------------------------
    // 2. この側の内部だけで、連結成分を識別する
    // -------------------------------------------------------------
    std::vector<std::vector<std::uint64_t>> sidePieceIds;

    // InitializePieceIdsは、局所的な番号を1から割り当てる。
    std::uint64_t localNextPieceId = 1;

    if (!InitializePieceIds(sideInput, epsilon, sidePieceIds, localNextPieceId, true))
        return false;

    const std::uint64_t pieceCount = localNextPieceId - 1;

    // -------------------------------------------------------------
    // 3. 局所番号を、モデル内で使用する新しい番号へ変換する
    // -------------------------------------------------------------
    const std::uint64_t maximumId = (std::numeric_limits<std::uint64_t>::max)();

    // 新しい番号を割り当てた後のカウンターも表現できることを確認する。
    if (pieceCount > maximumId - nextPieceId)
        return false;

    for (std::vector<std::uint64_t>& subsetIds : sidePieceIds)
    {
        for (std::uint64_t& pieceId : subsetIds)
        {
            // 局所番号1を、現在のnextPieceIdへ対応させる。
            pieceId = nextPieceId + (pieceId - 1);
        }
    }

    // -------------------------------------------------------------
    // 4. 原表面と封口へ、対応する番号配列を戻す
    // -------------------------------------------------------------
    for (std::size_t subsetIndex = 0; subsetIndex < componentResult.subsets.size(); ++subsetIndex)
    {
        SurfaceResult<VertexType>& surface = componentResult.subsets[subsetIndex];

        std::vector<std::uint64_t>& destination =
            positiveSide
            ? surface.positivePieceIds
            : surface.negativePieceIds;

        destination.swap(sidePieceIds[subsetIndex]);
    }

    std::vector<std::uint64_t>& capIds =
        positiveSide
        ? componentResult.caps.positivePieceIds
        : componentResult.caps.negativePieceIds;

    capIds.swap(sidePieceIds.back());

    // 次の側、または次の成分で使用する番号へ進める。
    nextPieceId += pieceCount;

    return true;
}

template<class VertexType>
bool MeshCutter::AssignNewPieceIds(ModelResult<VertexType>& componentResult, 
    float epsilon, std::uint64_t& nextPieceId)
{
    if (nextPieceId == 0)
        return false;

    // 呼び出し元のカウンターは、両側の成功まで変更しない。
    std::uint64_t availableId = nextPieceId;

    // 負側の原表面と封口を、一緒に採番する。
    if (!AssignSidePieceIds(componentResult, false, epsilon, availableId))
        return false;

    // 正側には、負側で使用していない番号を割り当てる。
    if (!AssignSidePieceIds(componentResult, true, epsilon, availableId))
        return false;

    componentResult.nextPieceId = availableId;
    nextPieceId = availableId;

    return true;
}