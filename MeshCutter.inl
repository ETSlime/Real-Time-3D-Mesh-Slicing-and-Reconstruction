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

template<class VertexType, class Accept>
bool MeshCutter::ClipSubset(
    const std::vector<VertexType>& input,
    const Plane& plane,
    float epsilon,
    Accept& accept,
    SurfaceResult<VertexType>& surface,
    CutContext<VertexType>& context,
    Status& failureStatus)
{
    // falseを返した場合に使用する中断理由。
    failureStatus = Status::Unchanged;

    // 入力は、3頂点ごとに1つの三角形を表す。
    if (input.size() % 3 != 0)
    {
        failureStatus = Status::InvalidInput;
        return false;
    }

    // 出力先は、このサブセットの原表面専用とする。
    surface.negative.clear();
    surface.positive.clear();

    // 共有するcontextの内容は消去しない。
    for (std::size_t triangleStart = 0; triangleStart < input.size(); triangleStart += 3)
    {
        const VertexType* triangle = &input[triangleStart];

        float distances[3] = {};

        // -------------------------------------------------------------
        // 1. 各頂点の位置と、平面までの符号付き距離を確認する
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
                failureStatus = Status::InvalidInput;
                return false;
            }

            float distance =
                Dot(position - plane.point, plane.normal);

            // 許容誤差内の頂点は、平面上として分類する。
            if (std::fabs(distance) <= epsilon)
            {
                distance = 0.0f;
            }

            distances[vertexIndex] = distance;

            // 正側・負側の存在は、モデル全体について蓄積する。
            if (distance > 0.0f)
            {
                context.hasPositive = true;
            }
            else if (distance < 0.0f)
            {
                context.hasNegative = true;
            }
        }

        // -------------------------------------------------------------
        // 2. 原表面を、現在のサブセットの両側出力へ追加する
        // -------------------------------------------------------------
        Clip(triangle, distances, -1.0f, epsilon, surface.negative);
        Clip(triangle, distances, 1.0f, epsilon, surface.positive);

        // -------------------------------------------------------------
        // 3. この三角形と平面との交点を集める
        // -------------------------------------------------------------
        std::vector<VertexType> hits;

        for (int currentIndex = 0; currentIndex < 3; ++currentIndex)
        {
            const int nextIndex = (currentIndex + 1) % 3;

            const VertexType& currentVertex = triangle[currentIndex];

            const VertexType& nextVertex = triangle[nextIndex];

            const float currentDistance = distances[currentIndex];

            const float nextDistance = distances[nextIndex];

            // 平面上にある元頂点を登録する。
            if (currentDistance == 0.0f)
            {
                AddUniqueIntersection(currentVertex, epsilon, hits);
            }

            // 両端が厳密に異なる側にある場合、辺の内部で交差する。
            const bool crossesPlane =
                (currentDistance < 0.0f && nextDistance > 0.0f)
                || (currentDistance > 0.0f && nextDistance < 0.0f);

            if (crossesPlane)
            {
                const float t = currentDistance / (currentDistance - nextDistance);

                const VertexType intersection = Interpolate(currentVertex, nextVertex, t);

                AddUniqueIntersection(intersection, epsilon,hits);
            }
        }

        // 共面三角形の扱いは、現在の制限を維持する。
        if (hits.size() == 3)
        {
            failureStatus = Status::OpenOrAmbiguousContour;
            return false;
        }

        // 交点が2つなければ、長さを持つ交線は登録しない。
        if (hits.size() != 2)
            continue;

        // -------------------------------------------------------------
        // 4. 交線をモデル全体の共有データへ追加する
        // -------------------------------------------------------------
        const bool accepted =
            accept(Read(hits[0].Position))
            && accept(Read(hits[1].Position));

        if (!accepted)
        {
            failureStatus = Status::Unchanged;
            return false;
        }

        const std::size_t nodeIndexA = FindOrAddCutNode(hits[0], epsilon, context.nodes, context.adjacency);
        const std::size_t nodeIndexB = FindOrAddCutNode(hits[1], epsilon, context.nodes, context.adjacency);

        // 許容誤差によって両端が統合された場合は、線分にしない。
        if (nodeIndexA == nodeIndexB)
            continue;

        std::vector<std::size_t>& neighborsA = context.adjacency[nodeIndexA];
        std::vector<std::size_t>& neighborsB = context.adjacency[nodeIndexB];

        // 同じ無向線分を二重登録しない。
        const bool alreadyConnected =
            std::find(
                neighborsA.begin(),
                neighborsA.end(),
                nodeIndexB) != neighborsA.end();

        if (!alreadyConnected)
        {
            neighborsA.push_back(nodeIndexB);
            neighborsB.push_back(nodeIndexA);
        }
    }

    // ここでは閉輪郭や封口の成否を判定しない。
    return true;
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

    // 現在は1つの入力に対して作成する。
    // SliceModelでは、全サブセットに対して同じcontextを使用する。
    CutContext<V> context;

    // この入力の原表面だけを保持する。
    SurfaceResult<V> surface;

    Status failureStatus = Status::Unchanged;

    const bool completed = ClipSubset(
        input,
        plane,
        e,
        accept,
        surface,
        context,
        failureStatus);

    if (!completed)
    {
        // 途中の表面と交線は採用せず、中断理由だけを返す。
        Result<V> stoppedResult;
        stoppedResult.status = failureStatus;

        return stoppedResult;
    }

    // 頂点をコピーせず、原表面の配列を既存の結果へ引き渡す。
    result.negative.swap(surface.negative);
    result.positive.swap(surface.positive);

    // 後続の封口処理では、共有データを参照する。
    const auto& nodes = context.nodes;
    const auto& adjacency = context.adjacency;

    const bool hasBothSides = context.hasPositive && context.hasNegative;

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

template<class VertexType, class Accept>
MeshCutter::ModelResult<VertexType> MeshCutter::SliceModel(
    const std::vector<std::vector<VertexType>>& inputSubsets,
    Plane plane,
    Options options,
    Accept accept)
{
    ModelResult<VertexType> result;

    // -----------------------------------------------------------------
    // 1. モデル全体で共通の切断設定を確認する
    // -----------------------------------------------------------------
    const float epsilon = options.epsilon;

    const float normalLength = std::sqrt(Dot(plane.normal, plane.normal));

    const bool validEpsilon = std::isfinite(epsilon) && epsilon > 0.0f;

    const bool validNormal = std::isfinite(normalLength) && normalLength > epsilon;

    const bool validOptions =
        std::isfinite(options.separation)
        && std::isfinite(options.uvScale);

    const bool validPlanePoint =
        std::isfinite(plane.point.x)
        && std::isfinite(plane.point.y)
        && std::isfinite(plane.point.z);

    if (!validEpsilon
        || !validNormal
        || !validOptions
        || !validPlanePoint)
    {
        result.status = Status::InvalidInput;
        return result;
    }

    // 全サブセットで、同じ単位法線を使用する。
    plane.normal = plane.normal * (1.0f / normalLength);

    // 入力サブセットとの番号対応を維持する。
    result.subsets.resize(inputSubsets.size());

    // -----------------------------------------------------------------
    // 2. 全サブセットの原表面を切り分け、交線を共有する
    // -----------------------------------------------------------------
    CutContext<VertexType> context;

    for (std::size_t subsetIndex = 0; subsetIndex < inputSubsets.size(); ++subsetIndex)
    {
        Status failureStatus = Status::Unchanged;

        const bool completed = ClipSubset(
            inputSubsets[subsetIndex],
            plane,
            epsilon,
            accept,
            result.subsets[subsetIndex],
            context,
            failureStatus);

        if (!completed)
        {
            // 一部の結果だけを採用しないよう、
            // 中断理由のみを持つ結果を返す。
            ModelResult<VertexType> stoppedResult;
            stoppedResult.status = failureStatus;

            return stoppedResult;
        }
    }

    // -----------------------------------------------------------------
    // 3. モデル全体について、切断が必要か確認する
    // -----------------------------------------------------------------
    const bool hasBothSides =
        context.hasPositive && context.hasNegative;

    if (!hasBothSides || context.nodes.empty())
    {
        return ModelResult<VertexType>{};
    }

    // -----------------------------------------------------------------
    // 4. 全材質の交線を接続し、閉じた輪郭を復元する
    // -----------------------------------------------------------------
    std::vector<std::vector<std::size_t>> loops;

    if (!BuildClosedContours(context.adjacency, loops))
    {
        result.status = Status::OpenOrAmbiguousContour;
        return result;
    }

    const std::vector<Point2> projected =
        ProjectCutNodesToPlane(context.nodes, plane);

    // 穴を持つ輪郭は、現在の制限として引き続き拒否する。
    if (HasNestedContours(loops, projected))
    {
        result.status = Status::UnsupportedHole;
        return result;
    }

    // -----------------------------------------------------------------
    // 5. 輪郭を三角形分割し、両側の封口を生成する
    // -----------------------------------------------------------------
    std::vector<TriangleIndices> capTriangles;

    if (!TriangulateContours(
        loops,
        projected,
        epsilon,
        capTriangles))
    {
        result.status = Status::TriangulationFailed;
        return result;
    }

    // 既存の封口生成関数はResultを受け取るため、
    // 封口だけを保持する一時出力として使用する。
    Result<VertexType> capResult;

    AppendCapTriangles(
        capResult,
        context.nodes,
        projected,
        capTriangles,
        plane,
        options);

    // 配列の所有権を交換し、頂点の再コピーを避ける。
    result.caps.negative.swap(capResult.negative);
    result.caps.positive.swap(capResult.positive);

    // -----------------------------------------------------------------
    // 6. 全サブセットの表面と封口を、同じ量だけ分離する
    // -----------------------------------------------------------------
    for (SurfaceResult<VertexType>& surface : result.subsets)
    {
        ApplySeparation(
            surface,
            plane,
            options.separation);
    }

    ApplySeparation(
        result.caps,
        plane,
        options.separation);

    // 原表面と封口の準備が、モデル全体について完了した。
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

template<class VertexType>
void MeshCutter::ApplySeparation(
    SurfaceResult<VertexType>& surface,
    const Plane& plane,
    float separation)
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