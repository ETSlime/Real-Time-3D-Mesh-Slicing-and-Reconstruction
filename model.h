//=============================================================================
//
// モデルの処理 [model.h]
// Author :
//
//=============================================================================
#pragma once

#include "main.h"
#include "ObjImporter.h"

#define MODEL_MAX_MATERIAL		(16)		// １モデルのMaxマテリアル数




//*****************************************************************************
// プロトタイプ宣言
//*****************************************************************************
// モデルの読み込みに成功した場合はtrue、失敗した場合はfalseを返す。
// 読み込み先には、既存リソースを持たないモデルを指定する。
bool LoadModel( const char *FileName, DX11_MODEL *Model );
// モデルが所有する描画リソースとCPU側データを解放する
void UnloadModel( DX11_MODEL *Model );
void DrawModel( DX11_MODEL *Model );

// モデルのマテリアルのディフューズを取得する。Max16個分にしてある
void GetModelDiffuse(DX11_MODEL *Model, XMFLOAT4 *diffuse);

// モデルの指定マテリアルのディフューズをセットする。
void SetModelDiffuse(DX11_MODEL *Model, int mno, XMFLOAT4 diffuse);
// Queued arrays transfer ownership to either update function and are always consumed.
// Queue texture pointers are borrowed; committed subsets each own an AddRef.
HRESULT TryUpdateModel(DX11_MODEL* Model);
void UpdateModel(DX11_MODEL* Model);


