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

template<class V, class Accept>
MeshCutter::Result<V> MeshCutter::Slice(const std::vector<V>& input, Plane plane, Options opt, Accept accept)
{
    Result<V> result;

    // 後続処理もこの変数を参照するため、
    // 今回は既存の変数名eを維持する。
    const float e = opt.epsilon;

    // 平面法線の長さを求める。
    const float normalLength = std::sqrt(Dot(plane.normal, plane.normal));

    // 許容誤差には、有限の正数を指定する。
    const bool validEpsilon = std::isfinite(e) && e > 0.0f;

    // 元の実装と同じ条件で、法線の長さを検査する。
    const bool validNormal = std::isfinite(normalLength) && normalLength > e;

    // 入力は、3頂点ごとに1つの三角形を表す。
    const bool validVertexCount = input.size() % 3 == 0;

    // 計算に使用する設定値が有限であることを確認する。
    const bool validOptions = std::isfinite(opt.separation) && std::isfinite(opt.uvScale);

    // 平面上の基準点が有限の座標を持つことを確認する。
    const bool validPlanePoint =
        std::isfinite(plane.point.x)
        && std::isfinite(plane.point.y)
        && std::isfinite(plane.point.z);

    // 入力が不正な場合は、切断処理を中止する。
    if (!validEpsilon
        || !validNormal
        || !validVertexCount
        || !validOptions
        || !validPlanePoint)
    {
        result.status = Status::InvalidInput;

        return result;
    }

    // 頂点から平面までの値を距離として扱えるよう、
    // 平面法線を単位長にそろえる。
    plane.normal = plane.normal * (1.0f / normalLength);

    // 切断面の輪郭構築に使用する交点一覧。
    // 配列の添字をノード番号として使用する。
    std::vector<V> nodes;

    // 各ノードに接続するノード番号を保持する。
    // nodesと同じ数の要素を持つ。
    std::vector<std::vector<std::size_t>> adjacency;

    // 入力全体に、平面の正側と負側の頂点が存在するかを記録する。
    bool positive = false;
    bool negative = false;

    // 入力は展開済みの三角形配列なので、3頂点ずつ処理する。
    for (std::size_t triangleStart = 0; triangleStart < input.size(); triangleStart += 3)
    {
        const V* triangle = &input[triangleStart];

        float distances[3] = {};

        // -------------------------------------------------------------
        // 1. 各頂点から切断平面までの符号付き距離を求める
        // -------------------------------------------------------------
        for (int vertexIndex = 0; vertexIndex < 3; ++vertexIndex)
        {
            const Vec3 position = Read(triangle[vertexIndex].Position);

            const bool validPosition =
                std::isfinite(position.x)
                && std::isfinite(position.y)
                && std::isfinite(position.z);
            if (!validPosition)
            {
                result.status = Status::InvalidInput;
                return result;
            }

            float distance = Dot(position - plane.point, plane.normal);

            // 平面に十分近い頂点は、平面上にあるものとして分類する。
            if (std::fabs(distance) <= e)
                distance = 0.0f;

            distances[vertexIndex] = distance;

            if (distance > 0.0f)
                positive = true;
            else if (distance < 0.0f)
                negative = true;
        }

        // -------------------------------------------------------------
        // 2. 元の表面を、平面の負側と正側へそれぞれ切り分ける
        // -------------------------------------------------------------
        Clip(triangle, distances, -1.0f, e, result.negative);
        Clip(triangle, distances, 1.0f, e, result.positive);

        // -------------------------------------------------------------
        // 3. 封口用の交線を作るため、三角形と平面の交点を集める
        // -------------------------------------------------------------
        std::vector<V> hits;

        for (int currentIndex = 0; currentIndex < 3; ++currentIndex)
        {
            const int nextIndex = (currentIndex + 1) % 3;

            const V& currentVertex = triangle[currentIndex];

            const V& nextVertex = triangle[nextIndex];
            const float currentDistance = distances[currentIndex];
            const float nextDistance = distances[nextIndex];

            // 平面上にある元頂点も、交点として登録する。
            if (currentDistance == 0.0f)
                AddUniqueIntersection(currentVertex, e, hits);

            // 両端が平面の反対側にある場合、辺の内部に交点がある。
            const bool crossesPlane =
                (currentDistance < 0.0f
                    && nextDistance > 0.0f)
                || (currentDistance > 0.0f
                    && nextDistance < 0.0f);

            if (crossesPlane)
            {
                const float t = currentDistance / (currentDistance - nextDistance);

                const V intersection = Interpolate(currentVertex, nextVertex, t);

                AddUniqueIntersection(intersection, e, hits);
            }
        }

        // 3つの異なる頂点が平面上にある場合は、
        // 共面として扱い、今回の切断を中止する。
        if (hits.size() == 3)
        {
            result.status = Status::OpenOrAmbiguousContour;
            return result;
        }

        // -------------------------------------------------------------
        // 4. 2つの交点を、輪郭構築用の交線として登録する
        // -------------------------------------------------------------
        if (hits.size() == 2)
        {
            // 交線の両端が、呼び出し元の切断条件を満たすか確認する。
            const bool accepted = accept(Read(hits[0].Position)) && accept(Read(hits[1].Position));

            if (!accepted)
            {
                // 途中まで生成した結果は採用せず、
                // 変更なしの空の結果を返す。
                return Result<V>{};
            }

            // 同じ交点を統合し、交線の両端をノード番号へ変換する。
            const std::size_t nodeIndexA = FindOrAddCutNode(hits[0], e, nodes, adjacency);
            const std::size_t nodeIndexB = FindOrAddCutNode(hits[1], e, nodes, adjacency);

            // 許容誤差によって両端が同じノードになった場合は、
            // 長さを持つ交線として登録しない。
            if (nodeIndexA != nodeIndexB)
            {
                // 交線の両端のノード番号を、隣接リストへ追加する。
                std::vector<std::size_t>& neighborsA = adjacency[nodeIndexA];
                std::vector<std::size_t>& neighborsB = adjacency[nodeIndexB];

                // 同じ交線を重複して登録しない。
                const bool alreadyConnected = std::find(neighborsA.begin(), neighborsA.end(), nodeIndexB) != neighborsA.end();

                if (!alreadyConnected)
                {
                    // 交線の接続関係を双方向に記録する。
                    neighborsA.push_back(nodeIndexB);
                    neighborsB.push_back(nodeIndexA);
                }
            }
        }
    }

    // 入力全体に正側と負側の両方が存在しなければ、
    // 2つの側へ分割する必要はない。
    const bool hasBothSides = positive && negative;

    // 交線を構成するノードがない場合も、変更しない。
    if (!hasBothSides || nodes.empty())
        return Result<V>{};

    // 交線の接続関係から、順序付きの閉じた輪郭を取得する。
    std::vector<std::vector<std::size_t>> loops;

    // 輪郭を構築できなければ、切断は成立しない。
    if (!BuildClosedContours(adjacency, loops))
    {
        result.status = Status::OpenOrAmbiguousContour;

        return result;
    }

    // 封口処理用に、交点を切断平面の2次元座標へ変換する。
    const std::vector<Point2> projected = ProjectCutNodesToPlane(nodes, plane);

    // 現在の封口処理は、穴を持つ輪郭には対応していない。
    if (HasNestedContours(loops, projected))
    {
        result.status = Status::UnsupportedHole;
        return result;
    }

    // 閉じた輪郭を、封口用の三角形へ分割する。
    std::vector<TriangleIndices> capTriangles;

    if (!TriangulateContours(loops, projected, e, capTriangles))
    {
        // 輪郭の三角形分割に失敗した場合は、切断は成立しない。
        result.status = Status::TriangulationFailed;
        return result;
    }

    // 三角形のノード番号から、両側の封口頂点を生成する。
    AppendCapTriangles(result, nodes, projected, capTriangles, plane, opt);

    // 負側の原表面と切断面を、同じ量だけ移動する。
    for (V& vertex : result.negative)
    {
        const Vec3 position = Read(vertex.Position);

        const Vec3 separatedPosition = position - plane.normal * opt.separation;

        Write(vertex.Position, separatedPosition);
    }

    // 正側の原表面と切断面を、反対方向へ移動する。
    for (V& vertex : result.positive)
    {
        const Vec3 position = Read(vertex.Position);

        const Vec3 separatedPosition = position + plane.normal * opt.separation;

        Write(vertex.Position, separatedPosition);
    }

    // 両側の表面と封口が完成したことを通知する。
    result.status = Status::Cut;

    return result;
}

template<class V>
MeshCutter::Result<V> MeshCutter::Slice(const std::vector<V>& input, Plane plane, Options opt)
{
    return Slice(input, plane, opt, [](Vec3) { return true; });
}

template<class V>
MeshCutter::Result<V> MeshCutter::Slice(const std::vector<V>& input, Plane plane)
{
    return Slice(input, plane, Options{});
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

template<class VertexType>
void MeshCutter::Triangle(std::vector<VertexType>& output, const VertexType& a, const VertexType& b, const VertexType& c, float epsilon)

{
    // 三角形の頂点位置を取得する。
    const Vec3 positionA = Read(a.Position);
    const Vec3 positionB = Read(b.Position);
    const Vec3 positionC = Read(c.Position);

    // 頂点Aを始点とする2本の辺を求める。
    const Vec3 edgeAB = positionB - positionA;
    const Vec3 edgeAC = positionC - positionA;

    // 外積の長さは、三角形の面積の2倍になる。
    const Vec3 areaVector = Cross(edgeAB, edgeAC);

    // 平方根を計算せずに、面積の大きさを比較する。
    const float areaVectorLengthSquared = Dot(areaVector, areaVector);

    // epsilonは長さの許容誤差。
    // 外積の長さの2乗と比較するため、4乗のしきい値を使用する。
    const float minimumAreaVectorLengthSquared = epsilon * epsilon * epsilon * epsilon;

    // 元の実装と同じ条件で、十分な面積がある場合だけ追加する。
    if (areaVectorLengthSquared > minimumAreaVectorLengthSquared)
    {
        // 頂点の並び順は変更しない。
        output.push_back(a);
        output.push_back(b);
        output.push_back(c);
    }
}

template<class VertexType>
void MeshCutter::Clip(const VertexType* triangle, const float* distances, float side, float epsilon, std::vector<VertexType>& output)

{
    // クリッピング後の頂点を、元の輪郭順に保持する。
    std::vector<VertexType> polygon;

    // 元の三角形の3辺を順番に処理する。
    for (int currentIndex = 0; currentIndex < 3; ++currentIndex)
    {
        // 最後の頂点の次は、先頭の頂点になる。
        const int nextIndex = (currentIndex + 1) % 3;

        // 現在の頂点と次の頂点を取得する。
        const VertexType& currentVertex = triangle[currentIndex];
        const VertexType& nextVertex = triangle[nextIndex];

        // 平面までの符号付き距離を取得する。
        const float currentDistance = distances[currentIndex];
        const float nextDistance = distances[nextIndex];

        // 平面上の頂点も、残す側に含める。
        const bool currentInside = currentDistance * side >= 0.0f;
        const bool nextInside = nextDistance * side >= 0.0f;

        // 現在の頂点が残す側にあれば、その頂点を追加する。
        if (currentInside)
            polygon.push_back(currentVertex);

        // 辺の両端で内外が切り替わる場合は、平面との交点を追加する。
        if (currentInside != nextInside)
        {
            // 交点の位置を、辺の両端の距離から線形補間する。
            const float t = currentDistance / (currentDistance - nextDistance);

            // 交点の頂点属性を、辺の両端から線形補間する。
            const VertexType intersection = Interpolate(currentVertex, nextVertex, t);

            // 交点を追加する。
            polygon.push_back(intersection);
        }
    }

    // 生成した凸多角形を、先頭頂点を共有する三角形へ分割する。
    // 頂点が3個未満の場合、このループは実行されない。
    for (std::size_t index = 1; index + 1 < polygon.size(); ++index)
    {
        Triangle(output, polygon[0], polygon[index], polygon[index + 1], epsilon);
    }
}

template<class VertexType>
void MeshCutter::AddUniqueIntersection(const VertexType& vertex, 
    float epsilon, std::vector<VertexType>& intersections)
{
    const Vec3 position = Read(vertex.Position);

    // 平方根を使わずに距離を比較する。
    const float epsilonSquared = epsilon * epsilon;

    // 既存の交点を順番に調べ、許容誤差以内の位置があれば追加しない。
    for (const VertexType& existing : intersections)
    {
        const Vec3 difference = position - Read(existing.Position);
        const float distanceSquared = Dot(difference, difference);

        if (distanceSquared <= epsilonSquared)
            return;
    }

    intersections.push_back(vertex);
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
        const Vec3 offset = Read(node.Position) - plane.point;

        Point2 point;

        // 各軸方向の成分を求める。
        point.x = Dot(offset, axisU);
        point.y = Dot(offset, axisV);

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
    vertex.TexCoord.x = planePosition.x * uvScale;
    vertex.TexCoord.y = planePosition.y * uvScale;

    return vertex;
}

template<class VertexType>
void MeshCutter::AppendCapTriangles(Result<VertexType>& result, const std::vector<VertexType>& nodes, 
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
        Triangle(result.negative, negativeA, negativeB, negativeC, options.epsilon);

        // 正側は逆順にして、幾何法線も反転させる。
        Triangle(result.positive, positiveC, positiveB, positiveA, options.epsilon);
    }
}
