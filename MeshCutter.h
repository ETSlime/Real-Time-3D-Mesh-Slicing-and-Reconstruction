#pragma once
#include "main.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

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
        // 平面上の判定や交点の統合に使用する位置許容誤差。
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

        // 交線が閉じない、または輪郭を一意に構築できない。
        OpenOrAmbiguousContour,

        // 入れ子の輪郭による穴を検出した。
        UnsupportedHole,

        // 切断面の三角形分割に失敗した。
        TriangulationFailed
    };

    // 頂点型を維持したまま、切断結果を保持する。
    template<class VertexType>
    struct Result
    {
        Status status = Status::Unchanged;

        // 平面法線の反対側にある三角形頂点。
        std::vector<VertexType> negative;

        // 平面法線が向く側にある三角形頂点。
        std::vector<VertexType> positive;
    };

    // 元サブセットの表面を、平面の両側へ切り分けた結果。
    // 封口用の三角形は含めない。
    template<class VertexType>
    struct SurfaceResult
    {
        // 平面法線の反対側に残る原表面。
        std::vector<VertexType> negative;

        // 平面法線が向く側に残る原表面。
        std::vector<VertexType> positive;
    };

    // 全サブセットをまとめて処理した、モデル全体の切断結果。
    template<class VertexType>
    struct ModelResult
    {
        // 切断の成否は、モデル全体について判定する。
        Status status = Status::Unchanged;

        // 元サブセットごとの原表面を保持する。
        // 入力と同じ順序・要素数を維持し、材質の対応付けに使用する。
        std::vector<SurfaceResult<VertexType>> subsets;

        // 全サブセットの交線から生成した、両側の封口。
        // 元表面とは分離し、呼び出し元で封口用の材質を設定する。
        SurfaceResult<VertexType> caps;
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

    // 三角形配列を平面で切断し、両側のメッシュを生成する。
    // inputとplaneは、同じローカル座標空間で指定する。
    template<class V, class Accept>
    static Result<V> Slice(const std::vector<V>& input, Plane plane, Options opt, Accept accept);

    // 画面上のドラッグ範囲を制限せずに切断する。
    template<class V>
    static Result<V> Slice(const std::vector<V>& input, Plane plane, Options opt);

    // 設定を省略した場合は、既定値を使用する。
    template<class V>
    static Result<V> Slice(const std::vector<V>& input, Plane plane);

    // 全サブセットの三角形を、同じ平面でまとめて切断する。
    // 原表面はサブセット別に保持し、封口はモデル全体で生成する。
    // 各サブセットの入力は、3頂点ごとに1つの三角形を表す。
    template<class VertexType, class Accept>
    static ModelResult<VertexType> SliceModel(
        const std::vector<std::vector<VertexType>>& inputSubsets,
        Plane plane,
        Options options,
        Accept accept);

private:
    // ------------------------------------------------------------------------
    // 内部計算用の型
    // ------------------------------------------------------------------------
    
    // 切断平面上の2次元座標。
    struct Point2
    {
        float x;
        float y;
    };

    // 封口三角形を構成する3つのノード番号。
    // 頂点データそのものではなく、nodesへの参照番号を保持する。
    struct TriangleIndices
    {
        std::size_t indexA;
        std::size_t indexB;
        std::size_t indexC;
    };

    // 1回のモデル切断で、全サブセットが共有する作業データ。
    // サブセットを切り替えても、この内容は初期化しない。
    template<class VertexType>
    struct CutContext
    {
        // 全サブセットから収集した、封口輪郭用の交点。
        // 配列の添字を共通ノード番号として使用する。
        std::vector<VertexType> nodes;

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
    static float Orient(Point2 a, Point2 b, Point2 c);

    // ------------------------------------------------------------------------
    // 頂点補間と三角形のクリッピング
    // ------------------------------------------------------------------------

    // 辺の両端の頂点属性を、同じ比率で線形補間する。
    // t=0でa、t=1でbに対応する位置になる。
    template<class VertexType>
    static VertexType Interpolate(const VertexType& a, const VertexType& b, float t);

    // 退化した三角形を除外し、指定された頂点順で結果へ追加する。
    template<class VertexType>
    static void Triangle(std::vector<VertexType>& output, const VertexType& a,
        const VertexType& b, const VertexType& c, float epsilon);

    // 三角形を平面の片側へクリッピングし、結果を三角形として追加する。
    // sideが+1なら正側、-1なら負側を残す。
    template<class VertexType>
    static void Clip(const VertexType* triangle, const float* distances,
        float side, float epsilon, std::vector<VertexType>& output);

    // 1つのサブセットについて、原表面を切り分けて交線を収集する。
    // plane.normalは、呼び出し前に単位長へ正規化しておく。
    // contextは全サブセットで共有し、この関数では初期化しない。
    // falseの場合、failureStatusに中断理由を設定する。
    template<class VertexType, class Accept>
    static bool ClipSubset(
        const std::vector<VertexType>& input,
        const Plane& plane,
        float epsilon,
        Accept& accept,
        SurfaceResult<VertexType>& surface,
        CutContext<VertexType>& context,
        Status& failureStatus);

    // ------------------------------------------------------------------------
    // 交点の統合と閉じた輪郭の構築
    // ------------------------------------------------------------------------

    // 1つの三角形から取得した交点の重複を除く。
    // 同じ位置の交点が登録済みであれば追加しない。
    template<class VertexType>
    static void AddUniqueIntersection(const VertexType& vertex, 
        float epsilon, std::vector<VertexType>& intersections);

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
    static float SignedDoubleArea(const std::vector<std::size_t>& contour, 
        const std::vector<Point2>& projected);

    // 輪郭の頂点順を、2次元座標上の反時計回りに統一する。
    static void EnsureCounterClockwise(std::vector<std::size_t>& contour,
        const std::vector<Point2>& projected);

    // 閉じた輪郭を耳切り法で三角形へ分割する。
    // 失敗した場合、呼び出し元は出力結果を使用しない。
    static bool TriangulateContours(
        const std::vector<std::vector<std::size_t>>& loops,
        const std::vector<Point2>& projected,
        float epsilon,
        std::vector<TriangleIndices>& capTriangles);

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
    static void AppendCapTriangles(
        Result<VertexType>& result,
        const std::vector<VertexType>& nodes,
        const std::vector<Point2>& projected,
        const std::vector<TriangleIndices>& capTriangles,
        const Plane& plane,
        const Options& options);

    // 両側の頂点を、平面法線に沿って反対方向へ移動する。
    // 原表面と封口に、同じ移動量を適用する。
    template<class VertexType>
    static void ApplySeparation(
        SurfaceResult<VertexType>& surface,
        const Plane& plane,
        float separation);
};

// テンプレートの定義を、利用側の翻訳単位から参照可能にする。
#include "MeshCutter.inl"
