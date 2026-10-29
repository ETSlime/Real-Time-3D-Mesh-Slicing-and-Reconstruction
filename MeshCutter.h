#pragma once
#include "main.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <stack>
#include <limits>
#include <map>
#include <numeric>
#include <utility>

// メッシュ切断のCPU側計算を担当する。
// 描画バッファやプレイヤーの状態は直接操作しない。
class MeshCutter
{
public:
    // 静的関数のみを使用する。
    MeshCutter() = delete;

    // DirectXに依存しない3次元ベクトル。
    struct Vec3
    {
        float x;
        float y;
        float z;

        // ベクトル同士を加算する。
        Vec3 operator+(const Vec3& other) const
        {
            return {x + other.x, y + other.y, z + other.z};
        }

        // ベクトル同士を減算する。
        Vec3 operator-(const Vec3& other) const
        {
            return {x - other.x, y - other.y, z - other.z};
        }

        // ベクトルを実数倍する。
        Vec3 operator*(float scale) const
        {
            return {x * scale, y * scale, z * scale};
        }
    };

    // 切断計算に使用するローカル空間の平面。
    struct Plane
    {
        Vec3 point;
        Vec3 normal;
    };

    // 切断計算の設定値。
    struct Options
    {
        // 平面上の判定や交点統合に使用する基準許容誤差。
        // 接続を確定できない場合、成分内だけ最大2回半減して再試行する。
        float epsilon = 0.0001f;

        // 切断後に、それぞれの側を移動させる距離。
        float separation = 1.5f;

        // 切断面の平面座標をUVへ変換する倍率。
        float uvScale = 0.1f;
    };

    // 切断処理の結果を表す。
    enum class Status
    {
        // 切断が成立せず、元のモデルを維持する。
        Unchanged,

        // 両側のメッシュ生成と封口処理が完了した。
        Cut,

        // 入力形式や設定値が不正。
        InvalidInput,

        // 輪郭や、生成後の面接続を一意に確定できない。
        OpenOrAmbiguousContour,

        // 入れ子の輪郭による穴を検出した。
        UnsupportedHole,

        // 切断面の三角形分割に失敗した。
        TriangulationFailed
    };

    // 平面の両側にある三角形と、その破片番号を保持する。
    template<class VertexType>
    struct SurfaceResult
    {
        // 負側の展開済み三角形頂点。
        std::vector<VertexType> negative;

        // 正側の展開済み三角形頂点。
        std::vector<VertexType> positive;

        // negativeの各三角形が属する破片番号。
        // negativePieceIds[i]は、negativeの3*iから始まる三角形に対応する。
        std::vector<std::uint64_t> negativePieceIds;

        // positiveの各三角形が属する破片番号。
        std::vector<std::uint64_t> positivePieceIds;
    };

    // モデル全体の切断結果と、編集用の破片情報。
    template<class VertexType>
    struct ModelResult
    {
        Status status = Status::Unchanged;

        // 切断を採用した成分の原表面。
        // 添字は入力サブセット番号に対応する。
        std::vector<SurfaceResult<VertexType>> subsets;

        // 切断しなかった成分の原表面。
        // 元の位置と材質の対応を維持する。
        std::vector<std::vector<VertexType>> unchangedSubsets;

        // unchangedSubsetsの各三角形に対応する破片番号。
        std::vector<std::vector<std::uint64_t>> unchangedPieceIds;

        // 新しく生成した封口と、その破片番号。
        SurfaceResult<VertexType> caps;

        // この結果を採用した後に使用する、次の破片番号。
        std::uint64_t nextPieceId = 1;
    };

    // ------------------------------------------------------------------------
    // 表示と切断操作で使用する座標変換
    // ------------------------------------------------------------------------

    // x、y、zを持つ既存の型から、計算用ベクトルを読み取る。
    template<class PositionType>
    static Vec3 Read(const PositionType& position);

    // 拡大、回転、平行移動の順に適用するワールド行列を作成する。
    static XMMATRIX CreateWorldMatrix(
        const XMFLOAT3& scale,
        const XMFLOAT3& rotation,
        const XMFLOAT3& position);

    // スクリーン座標から、ワールド空間の単位レイ方向を求める。
    // 現在の透視投影行列と、全画面ビューポートを前提とする。
    static XMVECTOR CreateWorldRayDirection(
        float screenX,
        float screenY,
        const XMMATRIX& projectionMatrix,
        const XMMATRIX& inverseViewMatrix);

    // 新規モデルの初期連結成分に、破片番号を割り当てる。
    // 出力の外側はサブセット番号、内側は三角形番号に対応する。
    // requireClosedがtrueの場合は、生成結果の閉合も必要。
    // 成功時のみpieceIdsとnextPieceIdを更新する。
    template<class VertexType>
    static bool InitializePieceIds(const std::vector<std::vector<VertexType>>& inputSubsets, float epsilon, 
        std::vector<std::vector<std::uint64_t>>& pieceIds, std::uint64_t& nextPieceId, bool requireClosed = false);

    // 持続的な破片番号に基づいて、モデルを切断する。
    // 番号配列が省略された場合は、新規モデルとして初期化する。
    // 初期化済みモデルでは、保存済みの番号とカウンターを渡す。
    template<class VertexType, class Accept>
    static ModelResult<VertexType> SliceModel(
        const std::vector<std::vector<VertexType>>&inputSubsets, 
        Plane plane, Options options, Accept accept, 
        const std::vector<std::vector<std::uint64_t>>& inputPieceIds = {},
        std::uint64_t nextPieceId = 1);

    // 1つの破片について、材質境界を越えて切断と封口を行う。
    // 接続失敗時は局所許容誤差を細分化し、成功時のみ破片番号を確定する。
    // planeとoptionsはSliceModelで検証済みであること。
    template<class VertexType, class Accept>
    static ModelResult<VertexType> SliceComponent(
        const std::vector<std::vector<VertexType>>& inputSubsets,
        const Plane& plane,
        const Options& options,
        Accept& accept,
        std::uint64_t& nextPieceId);

private:

    using IndexPair = std::pair<std::size_t, std::size_t>;
    static constexpr unsigned PositiveSideMask = 1u;
    static constexpr unsigned NegativeSideMask = 2u;
    static constexpr unsigned BothSidesMask = PositiveSideMask | NegativeSideMask;

    // ------------------------------------------------------------------------
    // 内部計算用の型
    // ------------------------------------------------------------------------
    
    // 切断平面上の2次元座標。描画頂点のfloat型は変更しない。
    struct Point2
    {
        double x;
        double y;
    };

    // 封口三角形を構成する3つのノード番号。
    // 頂点データそのものではなく、nodesへの参照番号を保持する。
    struct TriangleIndices
    {
        std::size_t indexA;
        std::size_t indexB;
        std::size_t indexC;
    };

    // 入力内の1つの三角形の位置を表す。
    // 頂点データそのものは複製しない。
    struct TriangleReference
    {
        // 元の材質サブセット番号。
        std::size_t subsetIndex;

        // サブセット内で、この三角形が始まる頂点位置。
        // 入力は展開済みなので、ここから3頂点を参照する。
        std::size_t vertexStart;
    };

    // 同じ編集対象として扱う三角形の集合。
    struct ConnectedComponent
    {
        // TriangleReference一覧への参照番号。
        std::vector<std::size_t> triangleIndices;

        // 保存済みの破片番号。
        // 幾何接続だけから作成した、未採番の成分では0。
        std::uint64_t pieceId = 0;
    };

    // 接続判定専用の、三角形の位置番号。
    // 元の描画頂点番号とは別に管理する。
    struct TriangleTopology
    {
        std::size_t vertexIndices[3];
    };

    // 接続判定用の無向辺。
    // 頂点番号は、小さい方をvertexAに保存する。
    struct TopologyEdge
    {
        std::size_t vertexA;
        std::size_t vertexB;

        // この辺を持つ三角形の番号。
        std::size_t triangleIndex;

        // 同じ端点を持つ辺が隣り合うように並べる。
        bool operator<(const TopologyEdge& other) const
        {
            if (vertexA != other.vertexA)
            {
                return vertexA < other.vertexA;
            }

            if (vertexB != other.vertexB)
            {
                return vertexB < other.vertexB;
            }

            return triangleIndex < other.triangleIndex;
        }
    };

    // 空間ハッシュで使用する格子座標。
    struct GridCell
    {
        std::int64_t x;
        std::int64_t y;
        std::int64_t z;

        bool operator==(const GridCell& other) const
        {
            return x == other.x
                && y == other.y
                && z == other.z;
        }
    };

    // 格子座標をunordered_mapのキーとして使用する。
    struct GridCellHash
    {
        std::size_t operator()(const GridCell& cell) const;
    };

    // 各格子に、その内部の代表位置番号を保存する。
    using TopologyGrid = std::unordered_map<GridCell, std::vector<std::size_t>, GridCellHash>;

    // 1回のモデル切断で、全サブセットが共有する作業データ。
    // サブセットを切り替えても、この内容は初期化しない。
    template<class VertexType>
    struct CutContext
    {
        // 全サブセットから収集した、封口輪郭用の交点。
        // 配列の添字を共通ノード番号として使用する。
        std::vector<VertexType> nodes;

        // 材質境界をまたいで共有する、元の位置番号と分類。
        std::vector<Vec3> inputPositions;
        std::vector<double> inputDistances;
        std::vector<std::vector<std::size_t>> inputVertexIds;

        // (i,i)は元頂点、(min(i,j),max(i,j))は元辺を表す。
        std::map<IndexPair, std::size_t> cutNodeLookup;

        // 交線に接する三角形の正側・負側フラグ。
        std::map<IndexPair, unsigned> contourEdges;
        // 共通ノード間の接続関係。
        // nodesと同じ数の要素を持つ。
        std::vector<std::vector<std::size_t>> adjacency;

        // モデル全体に、平面の正側の頂点が存在するか。
        bool hasPositive = false;

        // モデル全体に、平面の負側の頂点が存在するか。
        bool hasNegative = false;
    };

    // ------------------------------------------------------------------------
    // 基本的なベクトル演算と座標変換
    // ------------------------------------------------------------------------
    
    // 内積を求める。
    static float Dot(Vec3 a, Vec3 b);

    // 外積を求める。
    static Vec3 Cross(Vec3 a, Vec3 b);

    // ベクトルを単位長へ正規化する。
    // 長さが0の場合は、元の実装と同じくゼロベクトルを返す。
    static Vec3 Normalize(Vec3 value);

    // 計算結果を、x、y、zを持つ既存の型へ書き戻す。
    template<class PositionType>
    static void Write(PositionType& position, Vec3 value);

    // 2次元の有向面積の2倍を求める。
    // 正なら左回り、負なら右回り、0なら3点が同一直線上にある。
    static double Orient(Point2 a, Point2 b, Point2 c);
    // 2次元の距離をdoubleで求める。
    static double Distance2D(Point2 a, Point2 b);

    // 有向面積の演算誤差の許容値。位置epsilonとは区別する。
    static double OrientationError(Point2 a, Point2 b, Point2 c);

    // ------------------------------------------------------------------------
    // 頂点補間と三角形のクリッピング
    // ------------------------------------------------------------------------

    // 辺の両端の頂点属性を、同じ比率で線形補間する。
    // t=0でa、t=1でbに対応する位置になる。
    template<class VertexType>
    static VertexType Interpolate(const VertexType& a, const VertexType& b, float t);

    // 完全に潰れた三角形を除外し、指定された頂点順で結果へ追加する。
    template <class VertexType>
    static void Triangle(std::vector<VertexType>& output, 
        const VertexType& a, const VertexType& b, const VertexType& c);

    // ------------------------------------------------------------------------
    // 材質境界を越えた連結成分の構築
    // ------------------------------------------------------------------------

    // 全サブセットの三角形を、参照情報の一覧にまとめる。
    // 不正な頂点数や座標を検出した場合はfalseを返す。
    // 失敗した場合、呼び出し元は出力を使用しない。
    template<class VertexType>
    static bool CollectTriangleReferences(
        const std::vector<std::vector<VertexType>>& inputSubsets,
        std::vector<TriangleReference>& triangles);

    // 位置を格子座標へ変換する。
    // 整数座標として扱えない場合はfalseを返す。
    static bool TryGetGridCell(
        Vec3 position,
        float epsilon,
        GridCell& cell);

    // 許容誤差内の代表位置を探し、なければ新しく登録する。
    // 元の描画頂点や、その属性は変更しない。
    static bool FindOrAddTopologyVertex(
        Vec3 position,
        float epsilon,
        std::vector<Vec3>& positions,
        TopologyGrid& grid,
        std::size_t& vertexIndex);

    // 全三角形に、接続判定用の位置番号を割り当てる。
    // trianglesはCollectTriangleReferencesの成功結果を使用する。
    // 失敗した場合、呼び出し元は出力を使用しない。
    template<class VertexType>
    static bool BuildTriangleTopology(
        const std::vector<std::vector<VertexType>>& inputSubsets,
        const std::vector<TriangleReference>& triangles,
        float epsilon,
        std::vector<Vec3>& positions,
        std::vector<TriangleTopology>& topology);

    // 共有辺から三角形の接続関係を作り、連結成分へ分ける。
    // 位置番号が重複する三角形や、3面以上が共有する辺は拒否する。
    // requireClosedがtrueの場合、1面だけが持つ境界辺も拒否する。
    static bool BuildConnectedComponents(
        const std::vector<TriangleTopology>& topology,
        std::vector<ConnectedComponent>& components,
        bool requireClosed = false);

    // 保存済みの破片番号から、三角形を処理グループへ分ける。
    // 異なる番号の三角形は、位置が一致しても接続しない。
    // trianglesはCollectTriangleReferencesの成功結果を使用する。
    // 失敗した場合、呼び出し元はcomponentsを使用しない。
    static bool BuildComponentsFromPieceIds(const std::vector<TriangleReference>& triangles,
        const std::vector<std::vector<std::uint64_t>>& pieceIds,
        std::vector<ConnectedComponent>& components);

    // ------------------------------------------------------------------------
    // 交点の統合と閉じた輪郭の構築
    // ------------------------------------------------------------------------

    // 同じ位置の交点が存在すれば、その番号を返す。
    // 存在しなければ新しい交点を追加し、その番号を返す。
    template<class VertexType>
    static std::size_t FindOrAddCutNode(const VertexType& vertex, float epsilon,
        std::vector<VertexType>& nodes, std::vector<std::vector<std::size_t>>& adjacency);

    // 交線の接続関係から、閉じた輪郭を構築する。
    // 成功した場合だけ、呼び出し元はcontoursを使用する。
    static bool BuildClosedContours(const std::vector<std::vector<std::size_t>>& adjacency,
        std::vector<std::vector<std::size_t>>& contours);

    // ------------------------------------------------------------------------
    // 切断平面上の2次元輪郭処理
    // ------------------------------------------------------------------------

    // 交点を切断平面上の2次元座標へ投影する。
    // 戻り値の添字は、入力ノードの番号と一致する。
    template<class VertexType>
    static std::vector<Point2> ProjectCutNodesToPlane(const std::vector<VertexType>& nodes, const Plane& plane);

    // 右方向への水平レイと輪郭の交差回数から、点の内外を判定する。
    // 輪郭は、境界順に並んだ3点以上の単純多角形を前提とする。
    static bool IsPointInsideContour(Point2 point, const std::vector<Point2>& contour);

    // ある輪郭が、別の輪郭の内部にあるか調べる。
    // 輪郭同士が交差・接触しないことを前提とする。
    static bool HasNestedContours(const std::vector<std::vector<std::size_t>>& contours,
        const std::vector<Point2>& projected);

    // 輪郭の符号付き面積の2倍を求める。
    // 正なら反時計回り、負なら時計回りを表す。
    static double SignedDoubleArea(const std::vector<std::size_t>& contour, 
        const std::vector<Point2>& projected);

    // 輪郭の頂点順を、2次元座標上の反時計回りに統一する。
    static void EnsureCounterClockwise(std::vector<std::size_t>& contour,
        const std::vector<Point2>& projected);

    // 閉じた輪郭を耳切り法で三角形へ分割する。
    // 失敗した場合、呼び出し元は出力結果を使用しない。
    static bool TriangulateContours(const std::vector<std::vector<std::size_t>>& loops,
        const std::vector<Point2>& projected, float epsilon, std::vector<TriangleIndices>& capTriangles);

    // ------------------------------------------------------------------------
    // 封口用の頂点生成
    // ------------------------------------------------------------------------

    // 交点から、切断面専用の頂点を作成する。
    // 元の表面で使用する頂点は変更しない。
    template<class VertexType>
    static VertexType CreateCapVertex(const VertexType& source, const Point2& planePosition,
        const Vec3& outwardNormal, float uvScale);

    // 三角形分割の結果から、両側の封口頂点を生成する。
    template<class VertexType>
    static void AppendCapTriangles(SurfaceResult<VertexType>& result,
        const std::vector<VertexType>& nodes, const std::vector<Point2>& projected,
        const std::vector<TriangleIndices>& capTriangles, const Plane& plane, const Options& options);

    // 両側の頂点を、平面法線に沿って反対方向へ移動する。
    // 原表面と封口に、同じ移動量を適用する。
    template<class VertexType>
    static void ApplySeparation(SurfaceResult<VertexType>& surface, const Plane& plane, float separation);

    // 両側の頂点配列と破片番号を、対応関係を維持して追加する。
    // sourceとdestinationには、採番済みの結果を指定する。
    // 対応数が不正な場合は、追加せずfalseを返す。
    template<class VertexType>
    static bool AppendSurface(SurfaceResult<VertexType>& destination, const SurfaceResult<VertexType>& source);

    // 1つの成分を切断した結果について、片側の新しい破片番号を割り当てる。
    // 全材質の原表面と封口を、一緒に接続判定する。
    // falseの場合、呼び出し元はcomponentResultを採用しない。
    template<class VertexType>
    static bool AssignSidePieceIds(ModelResult<VertexType>& componentResult,
        bool positiveSide, float epsilon, std::uint64_t& nextPieceId, bool requireClosed = true);

    // 両側の新しい破片番号を割り当てる。
    // 両側とも成功した場合だけ、呼び出し元のnextPieceIdを更新する。
    template<class VertexType>
    static bool AssignNewPieceIds(ModelResult<VertexType>& componentResult,
        float epsilon, std::uint64_t& nextPieceId, bool requireClosed = true);

    // ------------------------------------------------------------------------
    // 共通交点、三角化、移動後の検証を担当する内部処理
    // ------------------------------------------------------------------------

    // 入力位置と符号付き距離を、材質をまたいで一度だけ計算する。
    template <class VertexType>
    static bool InitializeCutContext(const std::vector<std::vector<VertexType>>& inputSubsets,
        const Plane& plane, float epsilon, CutContext<VertexType>& context);

    // 元頂点または元辺の番号から、共有する交点番号を取得する。
    template <class VertexType>
    static std::size_t FindOrAddSharedCutNode(IndexPair key, const VertexType& vertex,
        float epsilon, CutContext<VertexType>& context);

    // 描画属性を個別に補間し、位置だけを共有キャッシュへそろえる。
    template <class VertexType>
    static VertexType InterpolateSharedEdge(const VertexType& a, const VertexType& b,
        std::size_t indexA, std::size_t indexB, float epsilon,
        CutContext<VertexType>& context, std::size_t& nodeIndex);

    // 交点を再計算せず、事前計算済みの頂点から片側を生成する。
    template <class VertexType>
    static void AppendClippedTriangle(const VertexType* triangle, const double* distances,
        const VertexType* intersections, const bool* crossed,
        int side, std::vector<VertexType>& output);

    // 原表面を切り分け、隣接面が存在する側を交線ごとに蓄積する。
    template <class VertexType>
    static bool ClipSubset(const std::vector<VertexType>& input,
        const std::vector<std::size_t>& inputIds, float epsilon, SurfaceResult<VertexType>& surface, 
        CutContext<VertexType>& context, Status& failureStatus);

    // 接触辺を除外し、実際の切断輪郭について筆划範囲を確認する。
    template <class VertexType, class Accept>
    static bool BuildContourAdjacency(CutContext<VertexType>& context, Accept& accept);

    // 同じ番号を二度追加しない。位置の近さとは区別して扱う。
    static void AppendUniqueIndex(std::vector<std::size_t>& indices, std::size_t index);

    // 位置番号ごとの辺使用数から、平面領域の外周を取り出す。
    static bool BuildSurfaceBoundaryLoops(const std::vector<TriangleTopology>& topology,
        std::size_t nodeCount, std::vector<std::vector<std::size_t>>& loops);

    // 耳候補の品質を、最長辺に対する高さで比較する。
    static double MinimumTriangleHeight(const TriangleIndices& triangle,
        const std::vector<Point2>& projected);

    // 内包判定では、幾何学的epsilonではなく有向面積の演算誤差を使う。
    static bool IsInsideEarTriangle(Point2 point, Point2 a, Point2 b, Point2 c);

    // 反時計回りの1輪郭を耳切りする。
    static bool EarClipContour(std::vector<std::size_t> loop, const std::vector<Point2>& projected,
        std::vector<TriangleIndices>& triangles);

    // 外周辺を維持し、細い三角形を改善できる共有対角線だけを交換する。
    static bool ImproveTriangulation(std::vector<TriangleIndices>& triangles,
        const std::vector<Point2>& projected, float epsilon, std::size_t contourSize);

    // 簡略化を行わない耳切りと、内部対角線の改善を組み合わせる。
    static bool TriangulateDirect(const std::vector<std::vector<std::size_t>>& loops,
        const std::vector<Point2>& projected, float epsilon, std::vector<TriangleIndices>& triangles);

    // 元の全頂点について、単調な近共線鎖として扱えるか確認する。
    static bool CanSimplifyBoundaryChain(const std::vector<std::size_t>& original,
        const std::vector<Point2>& projected, std::size_t first, std::size_t last, float epsilon);

    // 省略した境界点をすべて戻し、原表面と封口の辺分割を一致させる。
    static bool RestoreBoundaryNodes(const std::vector<std::size_t>& original,
        const std::vector<std::size_t>& kept, const std::vector<Point2>& projected,
        std::vector<TriangleIndices>& triangles);

    // 近共線鎖を一時的に簡略化し、耳切り後に外周を復元する。
    static bool TriangulateWithBoundaryRestore(const std::vector<std::vector<std::size_t>>& loops,
        const std::vector<Point2>& projected, float epsilon, std::vector<TriangleIndices>& triangles);

    // 実際の3次元面が、期待する法線側を向いているかdoubleで確認する。
    template <class VertexType>
    static bool HasValidSurfaceWinding(const std::vector<VertexType>& vertices, Vec3 normal);

    // floatで分離した後に反転した封口だけを、最終座標で再分割する。
    template <class VertexType>
    static bool RepairMovedCaps(SurfaceResult<VertexType>& caps,
        const std::vector<VertexType>& nodes, const std::vector<std::vector<std::size_t>>& loops,
        const std::vector<Point2>& originalProjected, const Plane& plane, const Options& options);

    // 元の向き、同一法線、平面性から、既存面の再分割対象を限定する。
    template <class VertexType>
    static bool TryGetFlatSurfacePlane(const std::vector<VertexType>& surface,
        const std::vector<VertexType>& reference, float epsilon, Plane& plane);

    // 境界再分割用の位置番号と、属性を保持した代表頂点を準備する。
    template <class VertexType>
    static bool BuildFlatSurfaceData(const std::vector<VertexType>& surface, float epsilon,
        std::vector<VertexType>& nodes, std::vector<TriangleTopology>& topology);

    // 対角線を変更してもUVと色の補間を維持できる範囲か確認する。
    template <class VertexType>
    static bool HasAffineSurfaceAttributes(const std::vector<VertexType>& nodes,
        const std::vector<TriangleTopology>& topology, const std::vector<Point2>& projected,
        const std::vector<VertexType>& drawingVertices, float epsilon);

    // 再切断された既存封口など、修復可能な平面領域だけを再分割する。
    template <class VertexType>
    static bool RepairFlatSurface(std::vector<VertexType>& surface,
        const std::vector<VertexType>& reference, float epsilon);

    // 頂点のfloat間隔と距離演算誤差から、局所細分化の下限を求める。
    template <class VertexType>
    static double MinimumRefinementEpsilon(const std::vector<std::vector<VertexType>>& input,
        const Plane& plane, float separation);

    // 1つの許容誤差で、分離後の閉合確認と採番までを試みる。
    // 切断前の共通位置番号から、閉形状か開いた表面かを判定する。
    // 閉形状の封口失敗を、開いた表面へ変更する処理ではない。
    template <class VertexType>
    static bool ClassifyInputSurface(const CutContext<VertexType>& context, bool& closed);

    template <class VertexType, class Accept>
    static ModelResult<VertexType> SliceComponentAttempt(
        const std::vector<std::vector<VertexType>>& inputSubsets, const Plane& plane,
        const Options& options, Accept& accept, std::uint64_t& nextPieceId);
};

// テンプレートの定義を、利用側の翻訳単位から参照可能にする。
#include "MeshCutter.inl"
