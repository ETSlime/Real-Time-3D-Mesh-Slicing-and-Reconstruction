#include "ModelSnapshot.h"
#include "renderer.h"
#include <algorithm>
#include <limits>
#include <new>

ModelSnapshot::~ModelSnapshot()
{
    // 配列コピーの途中で失敗した場合は、
    // まだテクスチャ参照を取得していない。
    if (!m_HasTextureReferences)
        return;

    for (const SUBSET& subset : m_Subsets)
    {
        ID3D11ShaderResourceView* texture = subset.Material.Texture;

        if (texture != nullptr)
            texture->Release();
    }
}

std::unique_ptr<ModelSnapshot> ModelSnapshot::Capture(const DX11_MODEL* model)
{
    if (model == nullptr || model->modelData == nullptr)
        return nullptr;

    const MODEL& source = *model->modelData;

    // この機能では、読み込み済みの描画可能なモデルを対象とする。
    if (source.VertexNum == 0
        || source.IndexNum == 0
        || source.SubsetNum == 0
        || source.VertexArray == nullptr
        || source.IndexArray == nullptr
        || source.SubsetArray == nullptr)
    {
        return nullptr;
    }

    // コンストラクターが private のため、クラス内で直接生成する。
    std::unique_ptr<ModelSnapshot> snapshot(new ModelSnapshot);

    // 元モデルの配列とは別の領域へ、各要素をコピーする。
    snapshot->m_Vertices.assign(source.VertexArray, source.VertexArray + source.VertexNum);
    snapshot->m_Indices.assign(source.IndexArray, source.IndexArray + source.IndexNum);
    snapshot->m_Subsets.assign(source.SubsetArray, source.SubsetArray + source.SubsetNum);

    // 幾何配列と同じ時点の破片情報を保存する。
    snapshot->m_TrianglePieceIds = source.TrianglePieceIds;
    snapshot->m_NextPieceId = source.NextPieceId;

    // 配列の確保とコピーが完了してから、テクスチャ参照を取得する。
    // 各サブセットにつき1つの参照を保持する。
    for (const SUBSET& subset : snapshot->m_Subsets)
    {
        ID3D11ShaderResourceView* texture = subset.Material.Texture;

        if (texture != nullptr)
            texture->AddRef();
    }

    snapshot->m_HasTextureReferences = true;

    // 形状と対応する切断回数を保存する。
    snapshot->m_CutCount = model->modelData->CutCount;

    return snapshot;
}

HRESULT ModelSnapshot::Restore(DX11_MODEL* target) const

{
    // 復元先とスナップショットの内容を確認する。
    if (target == nullptr)
        return E_INVALIDARG;

    if (m_Vertices.empty() ||
        m_Indices.empty() ||
        m_Subsets.empty())
        return E_INVALIDARG;

    // MODEL の要素数は unsigned short で保持している。
    const size_t maximumCount = (std::numeric_limits<unsigned short>::max)();

    if (m_Vertices.size() > maximumCount ||
        m_Indices.size() > maximumCount ||
        m_Subsets.size() > maximumCount)
        return E_INVALIDARG;

    ID3D11Device* device = GetDevice();

    if (device == nullptr)
        return E_FAIL;

    // 復元に失敗した場合は、このオブジェクトが自動的に解放する。
    struct BufferOwner
    {
        ID3D11Buffer* buffer = nullptr;

        ~BufferOwner()
        {
            if (buffer != nullptr)
            {
                buffer->Release();
            }
        }

        BufferOwner() = default;
        BufferOwner(const BufferOwner&) = delete;
        BufferOwner& operator=(const BufferOwner&) = delete;
    };

    try
    {
        // 現在のモデルとは独立した CPU 配列を作成する。
        auto restoredVertices = std::make_unique<VERTEX_3D[]>(m_Vertices.size());
        auto restoredIndices = std::make_unique<unsigned short[]>(m_Indices.size());
        auto restoredSubsets = std::make_unique<SUBSET[]>(m_Subsets.size());

        std::copy(m_Vertices.begin(), m_Vertices.end(), restoredVertices.get());
        std::copy(m_Indices.begin(), m_Indices.end(), restoredIndices.get());
        std::copy(m_Subsets.begin(), m_Subsets.end(), restoredSubsets.get());

        // MODEL 本体も、現在のモデルを変更する前に確保しておく。
        auto restoredModel = std::make_unique<MODEL>();

        // 旧モデルを変更する前に、編集用の破片情報も準備する。
        restoredModel->TrianglePieceIds = m_TrianglePieceIds;
        restoredModel->NextPieceId = m_NextPieceId;

        BufferOwner restoredVertexBuffer;
        BufferOwner restoredIndexBuffer;

        // 復元した頂点配列から頂点バッファーを作成する。
        D3D11_BUFFER_DESC vertexBufferDescription = {};
        vertexBufferDescription.Usage = D3D11_USAGE_DEFAULT;
        vertexBufferDescription.ByteWidth = static_cast<UINT>(sizeof(VERTEX_3D) * m_Vertices.size());
        vertexBufferDescription.BindFlags = D3D11_BIND_VERTEX_BUFFER;

        D3D11_SUBRESOURCE_DATA vertexInitialData = {};
        vertexInitialData.pSysMem = restoredVertices.get();

        HRESULT result = device->CreateBuffer(
            &vertexBufferDescription,
            &vertexInitialData,
            &restoredVertexBuffer.buffer);

        if (FAILED(result))
            return result;

        // 復元した索引配列からインデックスバッファーを作成する。
        D3D11_BUFFER_DESC indexBufferDescription = {};
        indexBufferDescription.Usage = D3D11_USAGE_DEFAULT;
        indexBufferDescription.ByteWidth = static_cast<UINT>(sizeof(unsigned short) * m_Indices.size());
        indexBufferDescription.BindFlags = D3D11_BIND_INDEX_BUFFER;

        D3D11_SUBRESOURCE_DATA indexInitialData = {};
        indexInitialData.pSysMem = restoredIndices.get();

        result = device->CreateBuffer(
            &indexBufferDescription,
            &indexInitialData,
            &restoredIndexBuffer.buffer);

        if (FAILED(result))
            return result;

        // 復元後のモデルが所有するテクスチャ参照を確保する。
        // スナップショットが所有する参照とは別に管理する。
        for (const SUBSET& subset : m_Subsets)
        {
            ID3D11ShaderResourceView* texture = subset.Material.Texture;

            if (texture != nullptr)
            {
                texture->AddRef();
            }
        }

        // 必要なリソースの準備が完了したため、旧モデルを解放する。
        UnloadModel(target);

        // スナップショットと同じ切断回数へ戻す。
        restoredModel->CutCount = m_CutCount;

        // CPU 配列の所有権を復元後の MODEL に移す。
        restoredModel->VertexArray = restoredVertices.release();
        restoredModel->VertexNum = static_cast<unsigned short>(m_Vertices.size());

        restoredModel->IndexArray = restoredIndices.release();
        restoredModel->IndexNum = static_cast<unsigned short>(m_Indices.size());

        restoredModel->SubsetArray = restoredSubsets.release();
        restoredModel->SubsetNum = static_cast<unsigned short>(m_Subsets.size());

        target->modelData = restoredModel.release();

        // GPU バッファーの所有権も復元先へ移す。
        target->VertexBuffer = restoredVertexBuffer.buffer;
        restoredVertexBuffer.buffer = nullptr;

        target->IndexBuffer = restoredIndexBuffer.buffer;
        restoredIndexBuffer.buffer = nullptr;

        return S_OK;
    }
    catch (const std::bad_alloc&)
    {
        // メモリ確保に失敗した場合も、現在のモデルは維持する。
        return E_OUTOFMEMORY;
    }
}
