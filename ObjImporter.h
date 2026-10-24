#pragma once
#define _CRT_SECURE_NO_WARNINGS

#include "main.h"
#include "renderer.h"
#include "SimpleArray.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

//*********************************************************
// 構造体
//*********************************************************
// マテリアル構造体
struct MODEL_MATERIAL
{
	char						Name[256];
	MATERIAL					Material;
	char						TextureName[256];
	ID3D11ShaderResourceView* Texture;

	MODEL_MATERIAL()
	{
		Material.noTexSampling = 1;
		Material.LoadMaterial = FALSE;
		Material.Ambient = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
		Material.Diffuse = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
		Material.Specular = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
		Material.Emission = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
		Material.Shininess = 0.0f;
		Texture = NULL;
	}
};

// 描画サブセット構造体
struct SUBSET
{
	unsigned short	StartIndex;
	unsigned short	IndexNum;
	MODEL_MATERIAL	Material;

	SUBSET()
	{
		StartIndex = 0;
		IndexNum = 0;
	}
};

// モデル構造体
struct MODEL
{
	VERTEX_3D* VertexArray;
	unsigned short	VertexNum;
	unsigned short* IndexArray;
	unsigned short	IndexNum;
	SUBSET* SubsetArray;
	unsigned short	SubsetNum;
};


struct DX11_MODEL_MATERIAL
{
	MATERIAL					Material;
	ID3D11ShaderResourceView* Texture;
};

// 描画サブセット構造体
struct DX11_SUBSET
{
	unsigned short	StartIndex;
	unsigned short	IndexNum;
	DX11_MODEL_MATERIAL	Material;
};

struct DX11_MODEL
{
	ID3D11Buffer* VertexBuffer;
	ID3D11Buffer* IndexBuffer;

	//DX11_SUBSET		*SubsetArray;
	//unsigned short	SubsetNum;

	MODEL* modelData;
	SimpleArray<SimpleArray<VERTEX_3D>*> newVertexArrays;
	SimpleArray<SimpleArray<VERTEX_3D>*> oldVertexArrays;
	SimpleArray<MODEL_MATERIAL> newSubsetMaterial;
	SimpleArray<MODEL_MATERIAL> oldSubsetMaterial;

	DX11_MODEL()
	{
		modelData = new MODEL{};
		VertexBuffer = NULL;
		IndexBuffer = NULL;

	}
};

// OBJファイルの解析を行うユーティリティクラス。
// DirectXのリソースは扱わず、CPU側のデータだけを生成する。
class ObjImporter
{
public:
    // 静的関数のみを使用するため、インスタンスの生成を禁止する。
    ObjImporter() = delete;

    // OBJを解析し、モデルのCPU側データを生成する。
    static bool LoadObj(const char* fileName, MODEL* model);

private:
    // OBJから参照されるMTLを解析し、マテリアル配列を生成する。
	// MTLの読み込みに成功した場合はtrue、失敗した場合はfalseを返す。
	static bool LoadMaterial(const char* FileName, MODEL_MATERIAL** MaterialArray, unsigned short* MaterialNum);
};