// Included by model.cpp after model.h. Kept separate for fault-injection tests.
// All queued arrays are consumed on success AND failure. Textures in queues are borrowed.
HRESULT TryUpdateModel(DX11_MODEL* model)
{
    if (!model || !model->modelData) return E_INVALIDARG;
    struct QueueCleanup {
        DX11_MODEL* model;
        ~QueueCleanup() {
            for (int i=0;i<model->newVertexArrays.getSize();++i) delete model->newVertexArrays[i];
            for (int i=0;i<model->oldVertexArrays.getSize();++i) delete model->oldVertexArrays[i];
            model->newVertexArrays.clear(); model->oldVertexArrays.clear();
            model->newSubsetMaterial.clear(); model->oldSubsetMaterial.clear();
        }
    } cleanup{model};
    if (model->newVertexArrays.getSize()!=model->newSubsetMaterial.getSize() ||
        model->oldVertexArrays.getSize()!=model->oldSubsetMaterial.getSize()) return E_INVALIDARG;
    try {
        std::size_t total=0, subsetCount=0;
        const std::size_t limit=(std::numeric_limits<unsigned short>::max)();
        auto count=[&](SimpleArray<SimpleArray<VERTEX_3D>*>& arrays) {
            for (int i=0;i<arrays.getSize();++i) {
                if (!arrays[i]) return false;
                auto size=arrays[i]->getSize();
                if (size<0 || size%3 || static_cast<std::size_t>(size)>limit-total) return false;
                if (size) {total+=size;++subsetCount;}
            }
            return true;
        };
        if (!count(model->newVertexArrays)||!count(model->oldVertexArrays)||!total||subsetCount>limit) return E_INVALIDARG;
        std::unique_ptr<VERTEX_3D[]> vertices(new VERTEX_3D[total]);
        std::unique_ptr<unsigned short[]> indices(new unsigned short[total]);
        std::unique_ptr<SUBSET[]> subsets(new SUBSET[subsetCount]);
        std::size_t offset=0, subsetIndex=0;
        auto append=[&](SimpleArray<SimpleArray<VERTEX_3D>*>& arrays, SimpleArray<MODEL_MATERIAL>& materials) {
            for(int i=0;i<arrays.getSize();++i) {
                auto* array=arrays[i];int count=array->getSize();if(!count) continue;
                auto& subset=subsets[subsetIndex++];
                subset.StartIndex=static_cast<unsigned short>(offset);
                subset.IndexNum=static_cast<unsigned short>(count);subset.Material=materials[i];
                for(int j=0;j<count;++j) {
                    vertices[offset]=(*array)[j];
                    indices[offset]=static_cast<unsigned short>(offset);++offset;
                }
            }
        };
        append(model->newVertexArrays,model->newSubsetMaterial);
        append(model->oldVertexArrays,model->oldSubsetMaterial);
        struct BufferOwner {
            ID3D11Buffer* pointer=nullptr;
            ~BufferOwner(){if(pointer) pointer->Release();}
        } vertexBuffer,indexBuffer;
        D3D11_BUFFER_DESC desc={};D3D11_SUBRESOURCE_DATA data={};
        desc.Usage=D3D11_USAGE_DEFAULT;desc.ByteWidth=static_cast<UINT>(sizeof(VERTEX_3D)*total);
        desc.BindFlags=D3D11_BIND_VERTEX_BUFFER;data.pSysMem=vertices.get();
        HRESULT hr=GetDevice()->CreateBuffer(&desc,&data,&vertexBuffer.pointer);
        if(FAILED(hr)) return hr;
        desc.ByteWidth=static_cast<UINT>(sizeof(unsigned short)*total);desc.BindFlags=D3D11_BIND_INDEX_BUFFER;
        data.pSysMem=indices.get();
        hr=GetDevice()->CreateBuffer(&desc,&data,&indexBuffer.pointer);
        if(FAILED(hr)) return hr;
        // 旧モデルを破棄する前に、新しいMODEL本体を確保する。
        // 確保に失敗した場合は、旧モデルをそのまま残す。
        std::unique_ptr<MODEL> nextModelData(new MODEL{});
        // Each final subset owns one reference, even when multiple subsets share a texture.
        for(std::size_t i=0;i<subsetCount;++i) if(subsets[i].Material.Texture) subsets[i].Material.Texture->AddRef();
        // 新しい配列とバッファの準備が完了したため、旧モデルを解放する。
        UnloadModel(model);

        // 新しく確保したMODELの所有権をモデルへ引き渡す。
        model->modelData = nextModelData.release();
        auto& target=*model->modelData;
        target.VertexArray=vertices.release();target.VertexNum=static_cast<unsigned short>(total);
        target.IndexArray=indices.release();target.IndexNum=static_cast<unsigned short>(total);
        target.SubsetArray=subsets.release();target.SubsetNum=static_cast<unsigned short>(subsetCount);
        model->VertexBuffer=vertexBuffer.pointer;vertexBuffer.pointer=nullptr;
        model->IndexBuffer=indexBuffer.pointer;indexBuffer.pointer=nullptr;
        return S_OK;
    } catch (const std::bad_alloc&) { return E_OUTOFMEMORY; }
}

void UpdateModel(DX11_MODEL* model)
{
    // Compatibility wrapper for existing callers. New code should inspect TryUpdateModel.
    (void)TryUpdateModel(model);
}
