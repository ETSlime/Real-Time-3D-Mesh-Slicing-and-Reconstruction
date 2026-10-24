//=============================================================================
//
// モデルの処理 [model.cpp]
// Author : 
//
//=============================================================================
#define _CRT_SECURE_NO_WARNINGS
#include "main.h"
#include "model.h"
#include "ObjImporter.h"
#include "camera.h"
#include <memory>
#include <limits>
#include <new>



//*****************************************************************************
// 構造体定義
//*****************************************************************************


//*****************************************************************************
// グローバル変数
//*****************************************************************************




//=============================================================================
// 初期化処理
//=============================================================================
bool LoadModel( const char *FileName, DX11_MODEL *Model )
{
	// OBJまたは参照先MTLを読み込めなかった場合は、
	// GPUバッファの作成へ進まない。
	if (!ObjImporter::LoadObj(FileName, Model->modelData))
	{
		return false;
	}

	// 頂点バッファ生成
	{
		D3D11_BUFFER_DESC bd;
		ZeroMemory( &bd, sizeof(bd) );
		bd.Usage = D3D11_USAGE_DEFAULT;
		bd.ByteWidth = sizeof( VERTEX_3D ) * Model->modelData->VertexNum;
		bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		bd.CPUAccessFlags = 0;

		D3D11_SUBRESOURCE_DATA sd;
		ZeroMemory( &sd, sizeof(sd) );
		sd.pSysMem = Model->modelData->VertexArray;

		// 頂点バッファの作成結果を確認する。
		const HRESULT result = GetDevice()->CreateBuffer( &bd, &sd, &Model->VertexBuffer );
		if (FAILED(result))
		{
			UnloadModel(Model);
			return false;
		}
	}


	// インデックスバッファ生成
	{
		D3D11_BUFFER_DESC bd;
		ZeroMemory( &bd, sizeof(bd) );
		bd.Usage = D3D11_USAGE_DEFAULT;
		bd.ByteWidth = sizeof( unsigned short ) * Model->modelData->IndexNum;
		bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
		bd.CPUAccessFlags = 0;

		D3D11_SUBRESOURCE_DATA sd;
		ZeroMemory( &sd, sizeof(sd) );
		sd.pSysMem = Model->modelData->IndexArray;

		// インデックスバッファの作成結果を確認する。
		const HRESULT result = GetDevice()->CreateBuffer( &bd, &sd, &Model->IndexBuffer );
		if (FAILED(result))
		{
			UnloadModel(Model);
			return false;
		}
	}

	// サブセット設定
	//{
	//	Model->SubsetArray = new DX11_SUBSET[Model->model->SubsetNum ];
	//	Model->SubsetNum = Model->model->SubsetNum;

		for( unsigned short i = 0; i < Model->modelData->SubsetNum; i++ )
		{
			//Model->SubsetArray[i].StartIndex = Model->model->SubsetArray[i].StartIndex;
			//Model->SubsetArray[i].IndexNum = Model->model->SubsetArray[i].IndexNum;

			//Model->SubsetArray[i].Material.Material = Model->model->SubsetArray[i].Material.Material;

			D3DX11CreateShaderResourceViewFromFile( GetDevice(),
													Model->modelData->SubsetArray[i].Material.TextureName,
													NULL,
													NULL,
													&Model->modelData->SubsetArray[i].Material.Texture,
													NULL );
		}
	//}

	//delete[] model.VertexArray;
	//delete[] model.IndexArray;
	//delete[] model.SubsetArray;

		// CPU側のモデルデータと、描画用バッファの準備が完了した。
		return true;
}


//=============================================================================
// 終了処理
//=============================================================================
void UnloadModel(DX11_MODEL* model)
{
	if (model == nullptr)
		return;

	// GPU側の頂点バッファを解放する。
	if (model->VertexBuffer != nullptr)
	{
		model->VertexBuffer->Release();
		model->VertexBuffer = nullptr;
	}

	// GPU側のインデックスバッファを解放する。
	if (model->IndexBuffer != nullptr)
	{
		model->IndexBuffer->Release();
		model->IndexBuffer = nullptr;
	}

	if (model->modelData == nullptr)
	{
		return;
	}

	MODEL* data = model->modelData;

	// 各サブセットが所有するテクスチャ参照を解放する。
	if (data->SubsetArray != nullptr)
	{
		for (unsigned short i = 0; i < data->SubsetNum; ++i)
		{
			if (data->SubsetArray[i].Material.Texture != nullptr)
			{
				data->SubsetArray[i].Material.Texture->Release();
			}
		}
	}

	// CPU側の各配列を解放する。
	delete[] data->VertexArray;
	delete[] data->IndexArray;
	delete[] data->SubsetArray;

	// 配列を保持していたMODEL本体も解放する。
	delete data;

	// 再度呼び出されても解放済みデータを参照しないようにする。
	model->modelData = nullptr;
}

//=============================================================================
// 描画処理
//=============================================================================
void DrawModel( DX11_MODEL *Model )
{

	// 頂点バッファ設定
	UINT stride = sizeof( VERTEX_3D );
	UINT offset = 0;
	GetDeviceContext()->IASetVertexBuffers( 0, 1, &Model->VertexBuffer, &stride, &offset );

	// インデックスバッファ設定
	GetDeviceContext()->IASetIndexBuffer( Model->IndexBuffer, DXGI_FORMAT_R16_UINT, 0 );

	// プリミティブトポロジ設定
	GetDeviceContext()->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );

	for( unsigned short i = 0; i < Model->modelData->SubsetNum; i++ )
	{
		// マテリアル設定
		if (Model->modelData->SubsetArray[i].Material.Material.LoadMaterial)
			SetMaterial( Model->modelData->SubsetArray[i].Material.Material );

		// テクスチャ設定
		if (Model->modelData->SubsetArray[i].Material.Material.noTexSampling == 0)
		{
			GetDeviceContext()->PSSetShaderResources(0, 1, &Model->modelData->SubsetArray[i].Material.Texture);
		}

		// ポリゴン描画
		GetDeviceContext()->DrawIndexed( Model->modelData->SubsetArray[i].IndexNum, Model->modelData->SubsetArray[i].StartIndex, 0 );
	}


}








// モデルの全マテリアルのディフューズを取得する。Max16個分にしてある
void GetModelDiffuse(DX11_MODEL *Model, XMFLOAT4 *diffuse)
{
	int max = (Model->modelData->SubsetNum < MODEL_MAX_MATERIAL) ? Model->modelData->SubsetNum : MODEL_MAX_MATERIAL;

	for (unsigned short i = 0; i < max; i++)
	{
		// ディフューズ設定
		diffuse[i] = Model->modelData->SubsetArray[i].Material.Material.Diffuse;
	}
}


// モデルの指定マテリアルのディフューズをセットする。
void SetModelDiffuse(DX11_MODEL *Model, int mno, XMFLOAT4 diffuse)
{
	// ディフューズ設定
	Model->modelData->SubsetArray[mno].Material.Material.Diffuse = diffuse;
}


#include "ModelUpdate.inl"
