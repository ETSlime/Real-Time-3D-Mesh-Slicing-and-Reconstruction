//=============================================================================
//
// モデル処理 [player.h]
// Author : 
//
//=============================================================================
#pragma once
#include "SimpleArray.h"
#include "model.h"
#include "renderer.h"
#include "MeshCutter.h"
#include <memory>
#include <limits>

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

//=============================================================================
// プレイヤーの切断操作と、モデル更新の受け渡しを補助する
//=============================================================================
class PlayerCutSupport
{
public:
    PlayerCutSupport() = delete;

    // 1つのサブセットについて、更新前に切断結果を一時保持する。
    struct PendingSubset
    {
		// 切断した場合は負側、切断しなかった場合は元の頂点を保持する。
        std::unique_ptr<SimpleArray<VERTEX_3D>> primaryVertices;

		// 切断した場合だけ存在する、正側の頂点配列。
        std::unique_ptr<SimpleArray<VERTEX_3D>> additionalVertices;

        // 更新後のサブセットへ引き継ぐマテリアル。
        MODEL_MATERIAL material;
    };

    // ローカル空間の交点が、画面上のドラッグ範囲内にあるか調べる。
    static bool IsInsideStroke(MeshCutter::Vec3 point);

    // 切断計算の頂点配列を、既存モデル更新用の配列へコピーする。
	static std::unique_ptr<SimpleArray<VERTEX_3D>> CopyVertices(const std::vector<VERTEX_3D>& vertices);

	// 更新用配列からモデルを再構築し、準備完了後に旧モデルと置き換える
	static HRESULT TryUpdateModel(DX11_MODEL* model);
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
// 取り消しとやり直しのショートカットを処理する。
static bool UpdateModelHistoryShortcuts(void);