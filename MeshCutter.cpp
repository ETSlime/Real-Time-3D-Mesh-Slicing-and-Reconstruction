#include "MeshCutter.h"

XMMATRIX MeshCutter::CreateWorldMatrix(const XMFLOAT3& scale, 
    const XMFLOAT3& rotation, const XMFLOAT3& position)
{
    const XMMATRIX scaleMatrix = XMMatrixScaling(scale.x, scale.y, scale.z);

    // モデルの初期向きを補正するため、Y軸にPIを加える。
    // DrawPlayerと同じ回転を使用する。
    const XMMATRIX rotationMatrix = XMMatrixRotationRollPitchYaw(rotation.x, rotation.y + XM_PI, rotation.z);

    const XMMATRIX translationMatrix = XMMatrixTranslation( position.x, position.y, position.z);

    return scaleMatrix * rotationMatrix * translationMatrix;
}

XMVECTOR MeshCutter::CreateWorldRayDirection(float screenX, float screenY, 
    const XMMATRIX& projectionMatrix, const XMMATRIX& inverseViewMatrix)
{
    // ウィンドウ座標を、-1から1の範囲へ変換する。
    const float ndcX = 2.0f * screenX / static_cast<float>(SCREEN_WIDTH) - 1.0f;

    // ウィンドウ座標とNDCではY軸の向きが逆になる。
    const float ndcY = 1.0f - 2.0f * screenY / static_cast<float>(SCREEN_HEIGHT);

    // 透視投影によるX、Y方向の拡大率を取得する。
    const float projectionScaleX = XMVectorGetX(projectionMatrix.r[0]);
    const float projectionScaleY = XMVectorGetY(projectionMatrix.r[1]);

    // ビュー空間でZ=1となるレイ方向を作成する。
    // 方向ベクトルなので、同次座標のWは0にする。
    const XMVECTOR viewDirection = XMVectorSet(ndcX / projectionScaleX, ndcY / projectionScaleY, 1.0f, 0.0f);

    // 平行移動を適用せず、方向をワールド空間へ変換する。
    const XMVECTOR worldDirection = XMVector3TransformNormal(viewDirection, inverseViewMatrix);

    return XMVector3Normalize(worldDirection);
}

float MeshCutter::Dot(Vec3 a, Vec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

MeshCutter::Vec3 MeshCutter::Cross(Vec3 a, Vec3 b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

MeshCutter::Vec3 MeshCutter::Normalize(Vec3 value)
{
    const float length = std::sqrt(Dot(value, value));

    if (length > 0.0f)
        return value * (1.0f / length);

    return { 0.0f, 0.0f, 0.0f };
}

double MeshCutter::Orient(Point2 a, Point2 b, Point2 c)
{
    const double edgeABX = b.x - a.x;
    const double edgeABY = b.y - a.y;

    const double edgeACX = c.x - a.x;
    const double edgeACY = c.y - a.y;

    return edgeABX * edgeACY - edgeABY * edgeACX;
}

double MeshCutter::Distance2D(Point2 a, Point2 b)
{
    return std::hypot(b.x - a.x, b.y - a.y);
}

double MeshCutter::OrientationError(Point2 a, Point2 b, Point2 c)
{
    // 有向面積の演算誤差と、幾何学的な位置許容誤差を分離する。
    const double productA = std::abs((b.x - a.x) * (c.y - a.y));
    const double productB = std::abs((b.y - a.y) * (c.x - a.x));
    return 16.0 * std::numeric_limits<double>::epsilon() * (productA + productB);
}

void MeshCutter::AppendUniqueIndex(std::vector<std::size_t>& indices, std::size_t index)
{
    if (std::find(indices.begin(), indices.end(), index) == indices.end())
    {
        indices.push_back(index);
    }
}

bool MeshCutter::BuildSurfaceBoundaryLoops(const std::vector<TriangleTopology>& topology,
    std::size_t nodeCount, std::vector<std::vector<std::size_t>>& loops)
{
    std::map<IndexPair, unsigned> edges;
    for (const TriangleTopology& triangle : topology)
    {
        const auto& vertices = triangle.vertexIndices;
        if (vertices[0] == vertices[1] || vertices[1] == vertices[2] || vertices[2] == vertices[0])
            return false;

        for (std::size_t edge = 0; edge < 3; ++edge)
        {
            ++edges[std::minmax(vertices[edge], vertices[(edge + 1) % 3])];
        }
    }

    std::vector<std::vector<std::size_t>> adjacency(nodeCount);
    for (const auto& edge : edges)
    {
        if (edge.second > 2)
        {
            return false;
        }
        if (edge.second == 1)
        {
            const std::size_t a = edge.first.first;
            const std::size_t b = edge.first.second;
            adjacency[a].push_back(b);
            adjacency[b].push_back(a);
        }
    }
    return BuildClosedContours(adjacency, loops);
}

bool MeshCutter::BuildComponentsFromPieceIds(const std::vector<TriangleReference>& triangles,
    const std::vector<std::vector<std::uint64_t>>& pieceIds, std::vector<ConnectedComponent>& components)
{
    components.clear();

    // 破片番号から、出力グループの位置を取得する。
    std::unordered_map<std::uint64_t, std::size_t> componentIndices;

    for (std::size_t triangleIndex = 0;
        triangleIndex < triangles.size();
        ++triangleIndex)
    {
        const TriangleReference& reference =
            triangles[triangleIndex];

        if (reference.subsetIndex >= pieceIds.size())
            return false;

        const std::size_t subsetTriangleIndex = reference.vertexStart / 3;

        const std::vector<std::uint64_t>& subsetIds = pieceIds[reference.subsetIndex];

        if (subsetTriangleIndex >= subsetIds.size())
            return false;

        const std::uint64_t pieceId = subsetIds[subsetTriangleIndex];

        // 初期化済みの入力には、有効な番号が必要。
        if (pieceId == 0)
            return false;

        const auto found = componentIndices.find(pieceId);

        std::size_t componentIndex;

        if (found == componentIndices.end())
        {
            // この破片番号を初めて処理する。
            componentIndex = components.size();

            components.emplace_back();

            // このグループが表す、保存済みの破片番号を保持する。
            components.back().pieceId = pieceId;

            componentIndices.emplace(pieceId, componentIndex);
        }
        else
        {
            componentIndex = found->second;
        }

        components[componentIndex].triangleIndices.push_back(triangleIndex);
    }

    return true;
}

bool MeshCutter::BuildClosedContours(const std::vector<std::vector<std::size_t>>& adjacency, 
    std::vector<std::vector<std::size_t>>& contours)
{
    contours.clear();

    // 各ノードは、輪郭上の前後2つのノードと接続している必要がある。
    for (const std::vector<std::size_t>& neighbors : adjacency)
    {
        if (!neighbors.empty() && neighbors.size() != 2)
            return false;
    }

    const std::size_t nodeCount = adjacency.size();

    // すでに輪郭へ登録したノードを記録する。
    std::vector<bool> visited(nodeCount, false);

    for (std::size_t startIndex = 0; startIndex < nodeCount; ++startIndex)
    {
        if (visited[startIndex] || adjacency[startIndex].empty())
            continue;

        std::vector<std::size_t> contour;

        // nodeCountは、有効なノード番号と重ならない値。
        std::size_t previousIndex = nodeCount;
        std::size_t currentIndex = startIndex;

        while (true)
        {
            // 開始点へ戻る前に処理済みノードへ到達した場合は失敗。
            if (visited[currentIndex])
                return false;

            visited[currentIndex] = true;
            contour.push_back(currentIndex);

            const std::vector<std::size_t>& neighbors = adjacency[currentIndex];

            // 直前のノードではない方へ進む。
            std::size_t nextIndex;

            if (neighbors[0] == previousIndex)
                nextIndex = neighbors[1];
            else
                nextIndex = neighbors[0];

            previousIndex = currentIndex;
            currentIndex = nextIndex;

            // 開始点へ戻った時点で、1つの閉じた輪郭が完成する。
            if (currentIndex == startIndex)
                break;
        }

        // 面を構成するには、少なくとも3つのノードが必要。
        if (contour.size() < 3)
            return false;

        contours.push_back(contour);
    }

    return true;
}

bool MeshCutter::IsPointInsideContour(Point2 point, const std::vector<Point2>& contour)
{
    bool inside = false;

    // 最初に、末尾の頂点から先頭の頂点へ向かう辺を処理する。
    std::size_t previousIndex = contour.size() - 1;

    for (std::size_t currentIndex = 0; currentIndex < contour.size(); ++currentIndex)
    {
        const Point2& current = contour[currentIndex];

        const Point2& previous = contour[previousIndex];

        // 辺の両端が、判定点の高さをまたいでいるか調べる。
        const bool crossesHeight = (current.y > point.y) != (previous.y > point.y);

        if (crossesHeight)
        {
            // 判定点と同じ高さになる、辺上の補間係数。
            const double t = (point.y - current.y) / (previous.y - current.y);

            // 辺と水平線の交点のX座標。
            const double intersectionX = current.x + (previous.x - current.x) * t;

            // 右方向のレイと交差するたびに、内外を切り替える。
            if (point.x < intersectionX)
                inside = !inside;
        }

        previousIndex = currentIndex;
    }

    return inside;
}

bool MeshCutter::HasNestedContours(const std::vector<std::vector<std::size_t>>& contours, 
    const std::vector<Point2>& projected)
{
    for (std::size_t testIndex = 0; testIndex < contours.size(); ++testIndex)
    {
        // 対象輪郭から、代表点として先頭ノードを選ぶ。
        const std::size_t testNodeIndex = contours[testIndex][0];
        // 代表点の2次元座標を取得する。
        const Point2 testPoint = projected[testNodeIndex];

        for (std::size_t otherIndex = 0; otherIndex < contours.size(); ++otherIndex)
        {
            // 自分自身との比較は行わない。
            if (testIndex == otherIndex)
                continue;

            // 比較先の輪郭を、順序付きの2次元座標へ変換する。
            std::vector<Point2> otherContour;

            for (std::size_t nodeIndex : contours[otherIndex])
            {
                otherContour.push_back(projected[nodeIndex]);
            }

            // 代表点が、比較先の輪郭の内部にあれば、入れ子の輪郭が存在する。
            if (IsPointInsideContour(testPoint, otherContour))
                return true;
        }
    }
    // すべての輪郭を調べても、入れ子の輪郭は見つからなかった。
    return false;
}

double MeshCutter::SignedDoubleArea(const std::vector<std::size_t>& contour,
    const std::vector<Point2>& projected)
{
    if (contour.size() < 3)
        return 0.0;

    const Point2& origin = projected[contour.front()];
    double doubleArea = 0.0;
    for (std::size_t index = 1; index + 1 < contour.size(); ++index)
    {
        doubleArea += Orient(origin, projected[contour[index]], projected[contour[index + 1]]);
    }

    return doubleArea;
}

void MeshCutter::EnsureCounterClockwise(std::vector<std::size_t>& contour, 
    const std::vector<Point2>& projected)
{
    const double doubleArea = SignedDoubleArea(contour, projected);

    if (doubleArea < 0.0f)
    {
        // ノード番号の順序だけを反転する。
        // 頂点位置やノード番号そのものは変更しない。
        std::reverse(contour.begin(), contour.end());
    }
}

// 候補三角形の最小高さを求める。無効な向きの場合は0を返す。
double MeshCutter::MinimumTriangleHeight(const TriangleIndices& triangle, const std::vector<Point2>& projected)
{
    const Point2& a = projected[triangle.indexA];
    const Point2& b = projected[triangle.indexB];
    const Point2& c = projected[triangle.indexC];
    const double area = Orient(a, b, c);

    if (area <= OrientationError(a, b, c))
        return 0.0;

    const double longestEdge = (std::max)({ Distance2D(a, b), Distance2D(b, c), Distance2D(c, a) });
    return area / longestEdge;
}

bool MeshCutter::IsInsideEarTriangle(Point2 point, Point2 a, Point2 b, Point2 c)
{
    // epsilonで三角形を膨張させず、各有向面積の演算誤差だけを許容する。
    // 幾何学的には外側にある点で、全候補が遮られることを防ぐ。
    return Orient(a, b, point) >= -OrientationError(a, b, point) &&
        Orient(b, c, point) >= -OrientationError(b, c, point) &&
        Orient(c, a, point) >= -OrientationError(c, a, point);
}

bool MeshCutter::EarClipContour(std::vector<std::size_t> loop, const std::vector<Point2>& projected,
    std::vector<TriangleIndices>& triangles)
{
    triangles.clear();
    if (loop.size() < 3)
        return false;

    // loopは呼び出し側で反時計回りにそろえておく。
    while (loop.size() > 3)
    {
        std::size_t bestIndex = loop.size();
        double bestHeight = 0.0;

        for (std::size_t current = 0; current < loop.size(); ++current)
        {
            const std::size_t previous = (current + loop.size() - 1) % loop.size();
            const std::size_t next = (current + 1) % loop.size();
            const TriangleIndices candidate{ loop[previous], loop[current], loop[next] };
            const double height = MinimumTriangleHeight(candidate, projected);

            if (height <= bestHeight)
                continue;

            bool blocked = false;
            for (std::size_t other : loop)
            {
                if (other == candidate.indexA || other == candidate.indexB ||
                    other == candidate.indexC)
                {
                    continue;
                }

                if (IsInsideEarTriangle(projected[other], projected[candidate.indexA],
                    projected[candidate.indexB], projected[candidate.indexC]))
                {
                    blocked = true;
                    break;
                }
            }

            if (!blocked)
            {
                bestIndex = current;
                bestHeight = height;
            }
        }

        if (bestIndex == loop.size())
            return false;

        // 最初に見つかった耳ではなく、最小高さが最大の耳を採用する。
        const std::size_t previous = (bestIndex + loop.size() - 1) % loop.size();
        const std::size_t next = (bestIndex + 1) % loop.size();
        triangles.push_back({ loop[previous], loop[bestIndex], loop[next] });
        loop.erase(loop.begin() + bestIndex);
    }

    const TriangleIndices last{ loop[0], loop[1], loop[2] };
    if (MinimumTriangleHeight(last, projected) <= 0.0)
        return false;

    triangles.push_back(last);
    return true;
}

bool MeshCutter::ImproveTriangulation(std::vector<TriangleIndices>& triangles,
    const std::vector<Point2>& projected, float epsilon, std::size_t contourSize)
{
    // 反復回数に上限を設ける。改善できなくても、現在の有効な分割を保持する。
    std::size_t remaining = contourSize * contourSize * 2;
    bool changed = true;
    while (changed && remaining > 0)
    {
        remaining--;
        changed = false;
        std::map<IndexPair, IndexPair> edges;

        for (std::size_t index = 0; index < triangles.size() && !changed; ++index)
        {
            const TriangleIndices& triangle = triangles[index];
            const std::size_t vertices[3] = { triangle.indexA, triangle.indexB, triangle.indexC };

            for (std::size_t edge = 0; edge < 3; ++edge)
            {
                const std::size_t a = vertices[edge];
                const std::size_t b = vertices[(edge + 1) % 3];
                const std::size_t oppositeA = vertices[(edge + 2) % 3];
                const IndexPair key = std::minmax(a, b);
                const auto found = edges.find(key);
                if (found == edges.end())
                {
                    edges.emplace(key, IndexPair{ index, edge });
                    continue;
                }

                const std::size_t neighborIndex = found->second.first;
                const std::size_t neighborEdge = found->second.second;
                const TriangleIndices& neighbor = triangles[neighborIndex];
                const std::size_t neighborVertices[3] = { neighbor.indexA, neighbor.indexB,
                                                         neighbor.indexC };

                if (neighborVertices[neighborEdge] != b ||
                    neighborVertices[(neighborEdge + 1) % 3] != a)
                {
                    return false;
                }

                const double oldHeight = (std::min)(MinimumTriangleHeight(triangle, projected),
                    MinimumTriangleHeight(neighbor, projected));
                if (oldHeight >= epsilon)
                    continue;

                // 外周辺は変更せず、隣接2面の共有対角線だけを交換する。
                const std::size_t oppositeB = neighborVertices[(neighborEdge + 2) % 3];
                const TriangleIndices left{ oppositeA, a, oppositeB };
                const TriangleIndices right{ oppositeA, oppositeB, b };
                const double newHeight = (std::min)(MinimumTriangleHeight(left, projected),
                    MinimumTriangleHeight(right, projected));

                // 両面が正しい向きを保ち、細い方の高さが改善する場合だけ採用する。
                if (newHeight > oldHeight * (1.0 + 1.0e-8) + 1.0e-15)
                {
                    triangles[index] = left;
                    triangles[neighborIndex] = right;
                    changed = true;
                    break;
                }
            }
        }
    }
    return true;
}

bool MeshCutter::TriangulateDirect(const std::vector<std::vector<std::size_t>>& loops,
    const std::vector<Point2>& projected, float epsilon, std::vector<TriangleIndices>& triangles)
{
    triangles.clear();
    for (auto loop : loops)
    {
        if (loop.size() < 3)
        {
            return false;
        }
        EnsureCounterClockwise(loop, projected);
        std::vector<TriangleIndices> local;
        if (!EarClipContour(loop, projected, local) ||
            !ImproveTriangulation(local, projected, epsilon, loop.size()))
        {
            return false;
        }
        triangles.insert(triangles.end(), local.begin(), local.end());
    }
    return true;
}

bool MeshCutter::CanSimplifyBoundaryChain(const std::vector<std::size_t>& original,
    const std::vector<Point2>& projected, std::size_t first, std::size_t last, float epsilon)
{
    const Point2& a = projected[original[first]];
    const Point2& b = projected[original[last]];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (length <= epsilon)
        return false;

    // 両端だけでなく、すでに省略した全頂点も改めて確認する。
    // 投影順序が逆転する鎖を、1本の線分として扱わない。
    double previous = -1.0e-12;
    for (std::size_t index = (first + 1) % original.size(); index != last;
        index = (index + 1) % original.size())
    {
        const Point2& point = projected[original[index]];
        const double t = ((point.x - a.x) * dx + (point.y - a.y) * dy) / (length * length);
        if (t < previous || t < 0.0 || t > 1.0 ||
            std::abs(Orient(a, b, point)) > static_cast<double>(epsilon) * length)
        {
            return false;
        }
        previous = t;
    }
    return true;
}

bool MeshCutter::RestoreBoundaryNodes(const std::vector<std::size_t>& original,
    const std::vector<std::size_t>& kept, const std::vector<Point2>& projected, 
    std::vector<TriangleIndices>& triangles)
{
    for (std::size_t edge = 0; edge < kept.size(); ++edge)
    {
        const std::size_t first = kept[edge];
        const std::size_t last = kept[(edge + 1) % kept.size()];
        if ((first + 1) % original.size() == last)
            continue;

        std::size_t found = triangles.size();
        std::size_t opposite = 0;
        for (std::size_t index = 0; index < triangles.size() && found == triangles.size(); ++index)
        {
            const TriangleIndices& triangle = triangles[index];
            const std::size_t vertices[3] = { triangle.indexA, triangle.indexB, triangle.indexC };
            for (std::size_t corner = 0; corner < 3; ++corner)
            {
                if (vertices[corner] == original[first] &&
                    vertices[(corner + 1) % 3] == original[last])
                {
                    found = index;
                    opposite = vertices[(corner + 2) % 3];
                    break;
                }
            }
        }
        if (found == triangles.size())
            return false;

        // 省略した各境界点と対向頂点を結び直す。
        // 原表面の A-B-C と、封口の A-C が食い違うT字接続を作らない。
        std::vector<TriangleIndices> fan;
        for (std::size_t index = first; index != last; index = (index + 1) % original.size())
        {
            const TriangleIndices triangle{ original[index], original[(index + 1) % original.size()],
                                           opposite };
            if (MinimumTriangleHeight(triangle, projected) <= 0.0)
                return false;
            fan.push_back(triangle);
        }
        triangles.erase(triangles.begin() + found);
        triangles.insert(triangles.begin() + found, fan.begin(), fan.end());
    }
    return true;
}

bool MeshCutter::TriangulateWithBoundaryRestore(const std::vector<std::vector<std::size_t>>& loops,
    const std::vector<Point2>& projected, float epsilon, std::vector<TriangleIndices>& triangles)
{
    triangles.clear();
    for (auto original : loops)
    {
        if (original.size() < 3)
            return false;
        EnsureCounterClockwise(original, projected);
        std::vector<std::size_t> kept(original.size());
        std::iota(kept.begin(), kept.end(), std::size_t{ 0 });

        bool changed = true;
        while (changed && kept.size() > 3)
        {
            changed = false;
            for (std::size_t index = 0; index < kept.size(); ++index)
            {
                const std::size_t previous = (index + kept.size() - 1) % kept.size();
                const std::size_t next = (index + 1) % kept.size();
                if (CanSimplifyBoundaryChain(original, projected, kept[previous], kept[next],
                    epsilon))
                {
                    kept.erase(kept.begin() + index);
                    changed = true;
                    break;
                }
            }
        }

        std::vector<std::size_t> core;
        for (std::size_t index : kept)
        {
            core.push_back(original[index]);
        }
        std::vector<TriangleIndices> local;
        if (!EarClipContour(core, projected, local) ||
            !RestoreBoundaryNodes(original, kept, projected, local))
        {
            return false;
        }
        triangles.insert(triangles.end(), local.begin(), local.end());
    }
    return true;
}

bool MeshCutter::TriangulateContours(const std::vector<std::vector<std::size_t>>& loops,
    const std::vector<Point2>& projected, float epsilon, std::vector<TriangleIndices>& triangles)
{
    triangles.clear();
    double bestHeight = -1.0;
    constexpr int simplificationAttempts = 3;

    // 粗い簡略化から順に試す。外周頂点を復元できなければ、その候補は捨てる。
    for (int attempt = 0; attempt <= simplificationAttempts; ++attempt)
    {
        std::vector<TriangleIndices> candidate;
        const float tolerance = epsilon * std::pow(0.1f, static_cast<float>(attempt));
        const bool completed =
            attempt < simplificationAttempts
            ? TriangulateWithBoundaryRestore(loops, projected, tolerance, candidate)
            : TriangulateDirect(loops, projected, epsilon, candidate);
        if (!completed)
            continue;

        double minimumHeight = (std::numeric_limits<double>::max)();
        for (const TriangleIndices& triangle : candidate)
        {
            minimumHeight = (std::min)(minimumHeight, MinimumTriangleHeight(triangle, projected));
        }
        if (minimumHeight > bestHeight)
        {
            bestHeight = minimumHeight;
            triangles = std::move(candidate);
        }
        if (bestHeight >= epsilon)
        {
            break;
        }
    }
    return bestHeight >= 0.0;
}

std::size_t MeshCutter::GridCellHash::operator()(const GridCell& cell) const
{
    // 符号付き整数のハッシュ値を組み合わせる。
    std::size_t seed = std::hash<std::int64_t>{}(cell.x);

    const std::size_t hashY = std::hash<std::int64_t>{}(cell.y);
    const std::size_t hashZ = std::hash<std::int64_t>{}(cell.z);

    seed ^= hashY + 0x9e3779b9u + (seed << 6) + (seed >> 2);
    seed ^= hashZ + 0x9e3779b9u + (seed << 6) + (seed >> 2);

    return seed;
}

bool MeshCutter::TryGetGridCell(Vec3 position, float epsilon, GridCell& cell)
{
    if (!std::isfinite(epsilon) || epsilon <= 0.0f)
        return false;

    // 負座標も正しく分類するため、整数への切り捨てではなくfloorを使う。
    const double cellX = std::floor(static_cast<double>(position.x) / epsilon);
    const double cellY = std::floor(static_cast<double>(position.y) / epsilon);
    const double cellZ = std::floor(static_cast<double>(position.z) / epsilon);

    // int64_tへの変換と、隣接格子への±1を安全に行える範囲に限定する。
    // 2^60を使用し、整数型の上限には十分な余裕を持たせる。
    constexpr double coordinateLimit = 1152921504606846976.0;

    const bool validCoordinates = std::isfinite(cellX) && std::isfinite(cellY) &&
        std::isfinite(cellZ) && std::fabs(cellX) <= coordinateLimit &&
        std::fabs(cellY) <= coordinateLimit &&
        std::fabs(cellZ) <= coordinateLimit;

    if (!validCoordinates)
        return false;

    cell.x = static_cast<std::int64_t>(cellX);
    cell.y = static_cast<std::int64_t>(cellY);
    cell.z = static_cast<std::int64_t>(cellZ);

    return true;
}

bool MeshCutter::FindOrAddTopologyVertex(
    Vec3 position,
    float epsilon,
    std::vector<Vec3>& positions,
    TopologyGrid& grid,
    std::size_t& vertexIndex)
{
    GridCell cell;

    if (!TryGetGridCell(position, epsilon, cell))
        return false;

    const double epsilonSquared = static_cast<double>(epsilon) * epsilon;

    // 許容誤差内の点が隣の格子に入る場合もあるため、
    // 自分の格子を含む27個の格子を調べる。
    for (int offsetX = -1; offsetX <= 1; ++offsetX)
    {
        for (int offsetY = -1; offsetY <= 1; ++offsetY)
        {
            for (int offsetZ = -1; offsetZ <= 1; ++offsetZ)
            {
                const GridCell neighbor{ cell.x + offsetX, cell.y + offsetY, cell.z + offsetZ };

                const auto found = grid.find(neighbor);

                if (found == grid.end())
                    continue;

                for (std::size_t candidateIndex : found->second)
                {
                    const Vec3& candidate = positions[candidateIndex];

                    // 距離計算はdoubleで行い、
                    // floatの二乗による範囲不足を避ける。
                    const double dx = static_cast<double>(position.x) - candidate.x;
                    const double dy = static_cast<double>(position.y) - candidate.y;
                    const double dz = static_cast<double>(position.z) - candidate.z;

                    const double distanceSquared = dx * dx + dy * dy + dz * dz;

                    if (distanceSquared <= epsilonSquared)
                    {
                        vertexIndex = candidateIndex;
                        return true;
                    }
                }
            }
        }
    }

    // 近い代表位置がなければ、新しい番号を登録する。
    vertexIndex = positions.size();

    positions.push_back(position);
    grid[cell].push_back(vertexIndex);

    return true;
}

// 共有辺から三角形の接続関係を作り、連結成分へ分ける。
// 位置番号が重複する三角形や、3面以上が共有する辺は拒否する。
// 初期分組では境界辺を許可し、生成結果ではrequireClosedで閉合も確認する。
bool MeshCutter::BuildConnectedComponents(const std::vector<TriangleTopology>& topology,
    std::vector<ConnectedComponent>&components, bool requireClosed)
{
    components.clear();

    const std::size_t triangleCount = topology.size();

    // -----------------------------------------------------------------
    // 1. 各三角形から、3本の無向辺を作成する
    // -----------------------------------------------------------------
    std::vector<TopologyEdge> edges;
    edges.reserve(triangleCount * 3);

    for (std::size_t triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex)
    {
        const TriangleTopology& triangle = topology[triangleIndex];

        const std::size_t indexA = triangle.vertexIndices[0];
        const std::size_t indexB = triangle.vertexIndices[1];
        const std::size_t indexC = triangle.vertexIndices[2];

        // 許容誤差による位置統合後も、3頂点が異なる必要がある。
        // 潰れた三角形を通して、別の面を接続しない。
        if (indexA == indexB || indexB == indexC || indexC == indexA)
            return false;

        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            const std::size_t nextCorner = (corner + 1) % 3;

            const std::size_t current = triangle.vertexIndices[corner];

            const std::size_t next = triangle.vertexIndices[nextCorner];

            TopologyEdge edge{};

            // 向きに関係なく、同じ辺を同じ端点順で記録する。
            if (current < next)
            {
                edge.vertexA = current;
                edge.vertexB = next;
            }
            else
            {
                edge.vertexA = next;
                edge.vertexB = current;
            }

            edge.triangleIndex = triangleIndex;

            edges.push_back(edge);
        }
    }

    // -----------------------------------------------------------------
    // 2. 同じ幾何辺の記録を隣り合わせにする
    // -----------------------------------------------------------------
    std::sort(edges.begin(), edges.end());

    // 各三角形に、辺を共有する隣接三角形を記録する。
    std::vector<std::vector<std::size_t>> adjacency;
    adjacency.resize(triangleCount);

    std::size_t edgeBegin = 0;

    while (edgeBegin < edges.size())
    {
        std::size_t edgeEnd = edgeBegin + 1;

        // 同じ端点を持つ辺の範囲を取得する。
        while (edgeEnd < edges.size()
            && edges[edgeEnd].vertexA == edges[edgeBegin].vertexA
            && edges[edgeEnd].vertexB == edges[edgeBegin].vertexB)
        {
            ++edgeEnd;
        }

        const std::size_t incidentCount = edgeEnd - edgeBegin;

        if (incidentCount > 2)
        {
#ifdef _DEBUG
            // 問題となった辺の、接続判定用の位置番号。
            const std::size_t problemVertexA = edges[edgeBegin].vertexA;

            const std::size_t problemVertexB = edges[edgeBegin].vertexB;

            // この辺を使用している三角形を、デバッガーで確認する。
            std::vector<std::size_t> incidentTriangles;

            for (std::size_t index = edgeBegin; index < edgeEnd; ++index)
            {
                incidentTriangles.push_back(edges[index].triangleIndex);
            }
#endif // _DEBUG

            // 3面以上が共有する辺は、現在の入力条件では扱わない。
            return false;
        }

        if (requireClosed && incidentCount != 2)
            return false;

        if (incidentCount == 2)
        {
            const std::size_t triangleA = edges[edgeBegin].triangleIndex;

            const std::size_t triangleB = edges[edgeBegin + 1].triangleIndex;

            // 共有辺を持つ三角形を、双方向に接続する。
            adjacency[triangleA].push_back(triangleB);
            adjacency[triangleB].push_back(triangleA);
        }

        // incidentCountが1なら境界辺なので、接続先を追加しない。
        edgeBegin = edgeEnd;
    }

    // -----------------------------------------------------------------
    // 3. 三角形の接続関係をたどり、独立した成分へ分ける
    // -----------------------------------------------------------------
    std::vector<bool> visited(triangleCount, false);

    std::stack<std::size_t> pendingTriangles;

    for (std::size_t startIndex = 0; startIndex < triangleCount; ++startIndex)
    {
        if (visited[startIndex])
            continue;

        // 未訪問の三角形から、新しい連結成分を開始する。
        components.emplace_back();

        ConnectedComponent& component = components.back();

        visited[startIndex] = true;
        pendingTriangles.push(startIndex);

        while (!pendingTriangles.empty())
        {
            const std::size_t currentIndex = pendingTriangles.top();

            pendingTriangles.pop();

            component.triangleIndices.push_back(currentIndex);

            for (std::size_t neighborIndex : adjacency[currentIndex])
            {
                if (visited[neighborIndex])
                    continue;

                // スタックへ追加した時点で訪問済みにし、
                // 同じ三角形の重複登録を防ぐ。
                visited[neighborIndex] = true;
                pendingTriangles.push(neighborIndex);
            }
        }
    }

    return true;
}