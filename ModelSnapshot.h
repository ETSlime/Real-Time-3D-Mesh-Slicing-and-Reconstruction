#pragma once
#include "model.h"
#include <memory>
#include <vector>
#include <cstdint>

//=============================================================================
// ある時点のモデル形状とマテリアル情報を保持する
// GPU バッファーは保持せず、復元時に再作成する。
//=============================================================================
class ModelSnapshot
{
public:
    // 所有するテクスチャ参照の二重解放を防ぐため、
    // スナップショット本体のコピーは禁止する。
    ModelSnapshot(const ModelSnapshot&) = delete;
    ModelSnapshot& operator=(const ModelSnapshot&) = delete;

    // スナップショットが所有するテクスチャ参照を解放する。
    ~ModelSnapshot();

    // 現在のモデルから、独立したCPU側スナップショットを作成する。
    static std::unique_ptr<ModelSnapshot> Capture(const DX11_MODEL* model);

    // スナップショットからモデルを復元する。
    // 新しいリソースの作成に失敗した場合、現在のモデルは変更しない。
    HRESULT Restore(DX11_MODEL* target) const;

    // 復元処理から読み取るための、変更不可の配列参照。
    const std::vector<VERTEX_3D>& GetVertices() const
    {
        return m_Vertices;
    }

    // 保存したインデックス配列を取得する。
    const std::vector<unsigned short>& GetIndices() const
    {
        return m_Indices;
    }

    // 保存したサブセット配列を取得する。
    const std::vector<SUBSET>& GetSubsets() const
    {
        return m_Subsets;
    }

private:
    // Captureからのみ生成する。
    ModelSnapshot() = default;

    std::vector<VERTEX_3D> m_Vertices;
    std::vector<unsigned short> m_Indices;
    std::vector<SUBSET> m_Subsets;

    // 保存時点の三角形と破片番号の対応表。
    std::vector<std::uint64_t> m_TrianglePieceIds;

    // 保存時点の、次に使用する破片番号。
    std::uint64_t m_NextPieceId = 1;

    // テクスチャ参照の取得が完了したことを示す。
    bool m_HasTextureReferences = false;

    // 保存したモデル状態の切断回数。
    std::uint32_t m_CutCount = 0;
};