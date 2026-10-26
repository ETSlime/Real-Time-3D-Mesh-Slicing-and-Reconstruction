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

float MeshCutter::Orient(Point2 a, Point2 b, Point2 c)
{
    const float edgeABX = b.x - a.x;
    const float edgeABY = b.y - a.y;

    const float edgeACX = c.x - a.x;
    const float edgeACY = c.y - a.y;

    return edgeABX * edgeACY - edgeABY * edgeACX;
}

bool MeshCutter::BuildClosedContours(const std::vector<std::vector<std::size_t>>& adjacency, 
    std::vector<std::vector<std::size_t>>& contours)
{
    contours.clear();

    // 各ノードは、輪郭上の前後2つのノードと接続している必要がある。
    for (const std::vector<std::size_t>& neighbors : adjacency)
    {
        if (neighbors.size() != 2)
            return false;
    }

    const std::size_t nodeCount = adjacency.size();

    // すでに輪郭へ登録したノードを記録する。
    std::vector<bool> visited(nodeCount, false);

    for (std::size_t startIndex = 0; startIndex < nodeCount; ++startIndex)
    {
        if (visited[startIndex])
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
            const float t = (point.y - current.y) / (previous.y - current.y);

            // 辺と水平線の交点のX座標。
            const float intersectionX = current.x + (previous.x - current.x) * t;

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

float MeshCutter::SignedDoubleArea(const std::vector<std::size_t>& contour, 
    const std::vector<Point2>& projected)
{
    float doubleArea = 0.0f;

    for (std::size_t index = 0; index < contour.size(); ++index)
    {
        // 最後の頂点から先頭の頂点へ向かう辺も含める。
        const std::size_t nextIndex = (index + 1) % contour.size();

        const Point2& current = projected[contour[index]];

        const Point2& next = projected[contour[nextIndex]];

        doubleArea += current.x * next.y - current.y * next.x;
    }

    return doubleArea;
}

void MeshCutter::EnsureCounterClockwise(std::vector<std::size_t>& contour, 
    const std::vector<Point2>& projected)
{
    const float doubleArea = SignedDoubleArea(contour, projected);

    if (doubleArea < 0.0f)
    {
        // ノード番号の順序だけを反転する。
        // 頂点位置やノード番号そのものは変更しない。
        std::reverse(contour.begin(), contour.end());
    }
}


bool MeshCutter::TriangulateContours(const std::vector<std::vector<std::size_t>>& loops, 
    const std::vector<Point2>& projected, float epsilon, std::vector<TriangleIndices>& capTriangles)
{
    capTriangles.clear();

    // Orientの結果は面積と同じ次元を持つため、
    // 長さの許容誤差を2乗した値と比較する。
    const float areaTolerance = epsilon * epsilon;

    // 各輪郭をコピーし、元の輪郭情報を維持したまま処理する。
    for (auto loop : loops)
    {
        // 元の輪郭を保持し、このコピーから頂点を取り除いていく。
        EnsureCounterClockwise(loop, projected);

        // 4頂点以上ある間は、耳を1つずつ切り取る。
        while (loop.size() > 3)
        {
            bool earFound = false;

            for (std::size_t currentIndex = 0; currentIndex < loop.size(); ++currentIndex)
            {
                const std::size_t previousIndex = (currentIndex + loop.size() - 1) % loop.size();

                const std::size_t nextIndex = (currentIndex + 1) % loop.size();

                // 現在の輪郭で隣り合う3つのノードを取得する。
                const std::size_t nodeA = loop[previousIndex];
                const std::size_t nodeB = loop[currentIndex];
                const std::size_t nodeC = loop[nextIndex];

                // 3つのノードの2次元座標を取得する。
                const Point2& pointA = projected[nodeA];
                const Point2& pointB = projected[nodeB];
                const Point2& pointC = projected[nodeC];

                // 反時計回りの輪郭なので、正の向きが凸角になる。
                const float cornerArea = Orient(pointA, pointB, pointC);

                // 凹角、共線、または面積が小さすぎる候補は除外する。
                if (cornerArea <= areaTolerance)
                    continue;

                bool containsOtherVertex = false;

                // 候補三角形の内部や辺上に、
                // 他の輪郭頂点が存在しないか調べる。
                for (std::size_t otherNode : loop)
                {
                    // 候補三角形自身の3頂点は判定対象から除く。
                    if (otherNode == nodeA || otherNode == nodeB || otherNode == nodeC)
                        continue;

                    const Point2& point = projected[otherNode];

                    // 候補三角形の3辺に対して、点が左側にあるかを調べる。
                    const float sideAB = Orient(pointA, pointB, point);
                    const float sideBC = Orient(pointB, pointC, point);
                    const float sideCA = Orient(pointC, pointA, point);

                    // 反時計回りの三角形では、
                    // 内部の点は3辺すべての左側にある。
                    // 許容誤差内の辺上の点も含めて扱う。
                    const bool insideOrOnTriangle =
                        sideAB >= -areaTolerance
                        && sideBC >= -areaTolerance
                        && sideCA >= -areaTolerance;

                    // 候補三角形の内部や辺上に、他の輪郭頂点が存在する場合は除外する。
                    if (insideOrOnTriangle)
                    {
                        containsOtherVertex = true;
                        break;
                    }
                }

                if (containsOtherVertex)
                    continue;

                // この候補は耳として切り取れる。
                capTriangles.push_back(TriangleIndices{ nodeA, nodeB, nodeC });

                // 中央の頂点Bだけを現在の輪郭から取り除く。
                // nodesやprojectedの要素は削除しない。
                loop.erase(loop.begin() + currentIndex);

                earFound = true;

                // 輪郭が変わったため、次のwhile周回で調べ直す。
                break;
            }

            // どの候補も耳として採用できなければ、
            // 現在の条件では三角形分割を完了できない。
            if (!earFound)
                return false;
        }

        // 最後に残った3頂点を確認する。
        const std::size_t nodeA = loop[0];
        const std::size_t nodeB = loop[1];
        const std::size_t nodeC = loop[2];

        const float finalArea = Orient(projected[nodeA], projected[nodeB], projected[nodeC]);

        if (finalArea <= areaTolerance)
            return false;

        // 最後の三角形を追加し、この輪郭の分割を完了する。
        capTriangles.push_back(TriangleIndices{ nodeA, nodeB, nodeC });
    }

    return true;
}
