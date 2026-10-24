//=============================================================================
//
// モデル処理 [player.h]
// Author : 
//
//=============================================================================
#pragma once
#include "SimpleArray.h"
#include "model.h"
//*****************************************************************************
// マクロ定義
//*****************************************************************************
#define MAX_PLAYER		(1)					// プレイヤーの数

#define	PLAYER_SIZE		(5.0f)				// 当たり判定の大きさ


//*****************************************************************************
// 構造体定義
//*****************************************************************************
struct PLAYER
{
	XMFLOAT3		pos;		// ポリゴンの位置
	XMFLOAT3		rot;		// ポリゴンの向き(回転)
	XMFLOAT3		scl;		// ポリゴンの大きさ(スケール)

	XMFLOAT4X4		mtxWorld;	// ワールドマトリックス

	BOOL			load;
	DX11_MODEL		model;		// モデル情報

	float			spd;		// 移動スピード
	float			dir;		// 向き
	float			size;		// 当たり判定の大きさ
	int				shadowIdx;	// 影のIndex
	BOOL			use;


};

struct Plane
{
	XMFLOAT3	point;
	XMFLOAT3	normal;
	XMFLOAT3	localPoint;
	XMFLOAT3	localNormal;
	XMFLOAT4X4	localmtxWorld;
};

//*****************************************************************************
// プロトタイプ宣言
//*****************************************************************************
HRESULT InitPlayer(void);
void UninitPlayer(void);
void UpdatePlayer(void);
void DrawPlayer(void);

PLAYER *GetPlayer(void);
void UpdateMeshCut(void);
void DrawCuttingPlane(void);
void MakeCuttingPlaneVertex(void);
void UpdateCuttingPlane(void);
void CutMeshes(DX11_MODEL* model);
void SetLocalCuttingPlane(void);
void ResetPlayerPresentation(void);

// 新しいモデルの読み込みに成功した場合だけ、現在のモデルを置き換える。
bool ReplacePlayerModel(const char* FileName);
// モデルの位置や形状を変更せず、進行中のドラッグ操作だけを終了する。
void CancelPlayerCutGesture(void);
