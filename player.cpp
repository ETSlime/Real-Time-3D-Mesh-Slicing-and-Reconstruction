//=============================================================================
//
// モデル処理 [player.cpp]
// Author : 
//
//=============================================================================
#include "main.h"
#include "renderer.h"
#include "model.h"
#include "input.h"
#include "camera.h"
#include "debugproc.h"
#include "player.h"
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include "MeshCutter.h"
#include <memory>
#include <limits>

//*****************************************************************************
// マクロ定義
//*****************************************************************************
#define	MODEL_PLAYER		"data/MODEL/01_cube.obj"		// 読み込むモデル名

#define	VALUE_MOVE			(2.0f)							// 移動量
#define	VALUE_ROTATE		(XM_PI * 0.02f)					// 回転量

#define PLAYER_SHADOW_SIZE	(0.4f)							// 影の大きさ
#define PLAYER_OFFSET_Y		(7.0f)							// プレイヤーの足元をあわせる


//*****************************************************************************
// プロトタイプ宣言
//*****************************************************************************


//*****************************************************************************
// グローバル変数
//*****************************************************************************
static PLAYER				g_Player;						// プレイヤー
static BOOL isDragging = FALSE;
static float startX = 0, startY = 0;
static float currentX = 0, currentY = 0;
static float endX = 0, endY = 0;
static ID3D11Buffer* g_CuttingPlaneVertexBuffer = NULL;	// 頂点バッファ
static Plane cuttingPlane;
//=============================================================================
// 初期化処理
//=============================================================================
HRESULT InitPlayer(void)
{
	MakeCuttingPlaneVertex();
	//g_Player.model.modelData = new MODEL;
	LoadModel(MODEL_PLAYER, &g_Player.model);
	g_Player.load = TRUE;

    // 起動時にも、実行中のリセットと同じ初期状態を使用する。
    ResetPlayerPresentation();

	return S_OK;
}

//=============================================================================
// 終了処理
//=============================================================================
void UninitPlayer(void)
{
    if (g_CuttingPlaneVertexBuffer) {
        g_CuttingPlaneVertexBuffer->Release();
        g_CuttingPlaneVertexBuffer = nullptr;
    }
    isDragging = FALSE;
	// モデルの解放処理
	if (g_Player.load)
	{
		UnloadModel(&g_Player.model);
		g_Player.load = FALSE;
	}

}

//=============================================================================
// 更新処理
//=============================================================================
void UpdatePlayer(void)
{
	CAMERA *cam = GetCamera();

	// 移動させちゃう
	if (GetKeyboardPress(DIK_LEFT))
	{	// 左へ移動
		g_Player.spd = VALUE_MOVE;
		g_Player.dir = XM_PI / 2;
	}
	if (GetKeyboardPress(DIK_RIGHT))
	{	// 右へ移動
		g_Player.spd = VALUE_MOVE;
		g_Player.dir = -XM_PI / 2;
	}
	if (GetKeyboardPress(DIK_UP))
	{	// 上へ移動
		g_Player.spd = VALUE_MOVE;
		g_Player.dir = XM_PI;
	}
	if (GetKeyboardPress(DIK_DOWN))
	{	// 下へ移動
		g_Player.spd = VALUE_MOVE;
		g_Player.dir = 0.0f;
	}


#ifdef _DEBUG
    // Rキーでプレイヤーの表示状態を初期状態に戻す。
	if (GetKeyboardPress(DIK_R))
	{
        ResetPlayerPresentation();

        // 同じフレームで移動や切断操作を再開しないようにする。
        return;
	}
#endif


	//	// Key入力があったら移動処理する
	if (g_Player.spd > 0.0f)
	{
		//g_Player.rot.y = g_Player.dir;
		g_Player.rot.y = g_Player.dir + cam->rot.y;

		// 入力のあった方向へプレイヤーを向かせて移動させる
		g_Player.pos.x -= sinf(g_Player.rot.y) * g_Player.spd;
		g_Player.pos.z -= cosf(g_Player.rot.y) * g_Player.spd;

		//cam->at = g_Player.pos;
	}


	g_Player.spd *= 0.8f;


#ifdef _DEBUG	// デバッグ情報を表示する
	PrintDebugProc("Player:↑ → ↓ ←　Space\n");
	PrintDebugProc("Player:X:%f Y:%f Z:%f\n", g_Player.pos.x, g_Player.pos.y, g_Player.pos.z);
#endif

	UpdateMeshCut();
}

//=============================================================================
// 描画処理
//=============================================================================
void DrawPlayer(void)
{
	// カリング無効
	SetCullingMode(CULL_MODE_NONE);

	XMMATRIX mtxScl, mtxRot, mtxTranslate, mtxWorld;

	// ワールドマトリックスの初期化
	mtxWorld = XMMatrixIdentity();

	// スケールを反映
	mtxScl = XMMatrixScaling(g_Player.scl.x, g_Player.scl.y, g_Player.scl.z);
	mtxWorld = XMMatrixMultiply(mtxWorld, mtxScl);

	// 回転を反映
	mtxRot = XMMatrixRotationRollPitchYaw(g_Player.rot.x, g_Player.rot.y + XM_PI, g_Player.rot.z);
	mtxWorld = XMMatrixMultiply(mtxWorld, mtxRot);

	// 移動を反映
	mtxTranslate = XMMatrixTranslation(g_Player.pos.x, g_Player.pos.y, g_Player.pos.z);
	mtxWorld = XMMatrixMultiply(mtxWorld, mtxTranslate);

	// ワールドマトリックスの設定
	SetWorldMatrix(&mtxWorld);

	XMStoreFloat4x4(&g_Player.mtxWorld, mtxWorld);


	// モデル描画
	MATERIAL material;
	ZeroMemory(&material, sizeof(material));
	material.Diffuse = XMFLOAT4(1.0f, 1.0f, 0.0f, 1.0f);
	material.noTexSampling = TRUE;
	SetMaterial(material);
	DrawModel(&g_Player.model);


	//DrawCuttingPlane();

	// カリング設定を戻す
	SetCullingMode(CULL_MODE_BACK);
}

namespace {
XMMATRIX PlayerWorldMatrix()
{
    return XMMatrixScaling(g_Player.scl.x, g_Player.scl.y, g_Player.scl.z)
        * XMMatrixRotationRollPitchYaw(g_Player.rot.x, g_Player.rot.y + XM_PI, g_Player.rot.z)
        * XMMatrixTranslation(g_Player.pos.x, g_Player.pos.y, g_Player.pos.z);
}

bool IsInsideStroke(meshcut::Vec3 point)
{
    CAMERA* camera = GetCamera();
    XMVECTOR clip = XMVector4Transform(XMVectorSet(point.x, point.y, point.z, 1),
        PlayerWorldMatrix() * XMLoadFloat4x4(&camera->mtxView)
        * XMLoadFloat4x4(&camera->mtxProjection));
    float w = XMVectorGetW(clip);
    if (w <= 0.00001f) return false;
    float x = (XMVectorGetX(clip) / w + 1) * 0.5f * SCREEN_WIDTH;
    float y = (1 - XMVectorGetY(clip) / w) * 0.5f * SCREEN_HEIGHT;
    // A pixel margin also permits perfectly horizontal/vertical strokes.
    const float margin = 2.0f;
    return x >= (std::min)(startX, endX) - margin
        && x <= (std::max)(startX, endX) + margin
        && y >= (std::min)(startY, endY) - margin
        && y <= (std::max)(startY, endY) + margin;
}

struct PendingSubset {
    std::unique_ptr<SimpleArray<VERTEX_3D>> negative, positive;
    MODEL_MATERIAL material;
};
std::unique_ptr<SimpleArray<VERTEX_3D>> CopyVertices(const std::vector<VERTEX_3D>& vertices)
{
    std::unique_ptr<SimpleArray<VERTEX_3D>> array(new SimpleArray<VERTEX_3D>);
    for (const auto& vertex : vertices) array->push_back(vertex);
    return array;
}
}

void CutMeshes(DX11_MODEL* model)
{
    if (!model || !model->modelData || !model->modelData->SubsetArray) return;
    SetLocalCuttingPlane();
    meshcut::Plane plane{meshcut::Read(cuttingPlane.localPoint), meshcut::Read(cuttingPlane.localNormal)};
    std::vector<PendingSubset> pending;
    bool changed = false;
    std::size_t totalVertices = 0, totalSubsets = 0;
    for (std::size_t i = 0; i < model->modelData->SubsetNum; ++i) {
        const auto& subset = model->modelData->SubsetArray[i];
        std::size_t start = subset.StartIndex, count = subset.IndexNum;
        const MODEL& source=*model->modelData;
        if (!source.VertexArray || !source.IndexArray || start>source.IndexNum ||
            count>source.IndexNum-start || count%3) return;
        std::vector<VERTEX_3D> input;
        input.reserve(count);
        for (std::size_t j=0;j<count;++j) {
            unsigned short index=source.IndexArray[start+j];
            if (index>=source.VertexNum) return;
            input.push_back(source.VertexArray[index]);
        }
        auto result = meshcut::Slice(input, plane, meshcut::Options{}, IsInsideStroke);
        if (result.status != meshcut::Status::Cut && result.status != meshcut::Status::Unchanged) {
            PrintDebugProc("Mesh cut rejected: subset %u, status %d\n", static_cast<unsigned>(i), static_cast<int>(result.status));
            return; // Nothing has been published; preserve the entire original model.
        }
        PendingSubset staged;
        staged.material = subset.Material;
        if (result.status == meshcut::Status::Cut) {
            changed = true;
            totalVertices += result.negative.size() + result.positive.size();
            totalSubsets += 2;
            staged.negative = CopyVertices(result.negative);
            staged.positive = CopyVertices(result.positive);
        } else {
            totalVertices += input.size(); ++totalSubsets;
            staged.negative = CopyVertices(input);
        }
        pending.push_back(std::move(staged));
    }
    if (!changed) return;
    // Conservative guard: existing model code uses 16-bit counts/start offsets.
    const auto limit = (std::numeric_limits<unsigned short>::max)();
    if (totalVertices > limit || totalSubsets > limit) {
        PrintDebugProc("Mesh cut rejected: legacy 16-bit model capacity exceeded.\n");
        return;
    }
    // TryUpdateModel consumes queued arrays on every result.
    // Refuse to overwrite outstanding work owned by another operation.
    if (model->newVertexArrays.getSize() || model->oldVertexArrays.getSize()) return;
    model->newSubsetMaterial.clear();
    model->oldSubsetMaterial.clear();
    for (auto& subset : pending) {
        model->newVertexArrays.push_back(subset.negative.release());
        model->newSubsetMaterial.push_back(subset.material);
        if (subset.positive) {
            model->newVertexArrays.push_back(subset.positive.release());
            model->newSubsetMaterial.push_back(subset.material);
        }
    }
    HRESULT hr=TryUpdateModel(model);
    if (FAILED(hr)) PrintDebugProc("Mesh cut model update failed: 0x%08X\n", static_cast<unsigned>(hr));
}

void SetLocalCuttingPlane(void)
{
    XMMATRIX world = PlayerWorldMatrix();
    XMStoreFloat4x4(&g_Player.mtxWorld, world);
    XMMATRIX inverse = XMMatrixInverse(nullptr, world);
    XMVECTOR point = XMVector3TransformCoord(XMLoadFloat3(&cuttingPlane.point), inverse);
    // World -> local plane normal uses world transpose (row-vector convention).
    XMVECTOR normal = XMVector3Normalize(XMVector3TransformNormal(
        XMLoadFloat3(&cuttingPlane.normal), XMMatrixTranspose(world)));
    XMStoreFloat3(&cuttingPlane.localPoint, point);
    XMStoreFloat3(&cuttingPlane.localNormal, normal);
    XMStoreFloat4x4(&cuttingPlane.localmtxWorld, world);
}

//=============================================================================
// プレイヤーの表示状態と切断操作を初期状態に戻す
//=============================================================================
void ResetPlayerPresentation(void)
{
    // モデルの位置、回転、拡大率を初期状態に戻す。
    g_Player.pos = { 0.0f, PLAYER_OFFSET_Y, 0.0f };
    g_Player.rot = { 0.0f, 0.0f, 0.0f };
    g_Player.scl = { 1.0f, 1.0f, 1.0f };

    // 移動の慣性と移動方向をリセットする。
    g_Player.spd = 0.0f;
    g_Player.dir = 0.0f;

    // プレイヤーの基本状態を初期値に戻す。
    g_Player.size = PLAYER_SIZE;
    g_Player.use = TRUE;

    // モデル変更前のドラッグ操作を引き継がないようにする。
    isDragging = FALSE;

    startX = 0.0f;
    startY = 0.0f;

    currentX = 0.0f;
    currentY = 0.0f;

    endX = 0.0f;
    endY = 0.0f;

    // 前回の切断平面の情報をクリアする。
    cuttingPlane = {};

    // 描画処理と同じ初期姿勢でワールド行列を作成する。
    const XMMATRIX world =
        XMMatrixScaling(
            g_Player.scl.x,
            g_Player.scl.y,
            g_Player.scl.z)
        * XMMatrixRotationRollPitchYaw(
            g_Player.rot.x,
            g_Player.rot.y + XM_PI,
            g_Player.rot.z)
        * XMMatrixTranslation(
            g_Player.pos.x,
            g_Player.pos.y,
            g_Player.pos.z);

    XMStoreFloat4x4(&g_Player.mtxWorld, world);
}

//=============================================================================
// モデルを読み込み、成功した場合だけ現在のモデルと置き換える
//=============================================================================
bool ReplacePlayerModel(const char* FileName)
{
    // 現在表示しているモデルには触れず、別のモデルへ読み込む。
    DX11_MODEL replacement;

    if (!LoadModel(FileName, &replacement))
    {
        // 失敗した読み込み先を破棄する。
        // LoadModel側で解放済みの場合も、ポインタは空になっている。
        UnloadModel(&replacement);

        return false;
    }

    // 読み込みが完了したリソースだけを交換する。
    // 交換後、replacementは以前表示していたモデルを保持する。
    std::swap(g_Player.model.modelData, replacement.modelData);
    std::swap(g_Player.model.VertexBuffer, replacement.VertexBuffer);
    std::swap(g_Player.model.IndexBuffer, replacement.IndexBuffer);

    // 不要になった旧モデルのリソースを解放する。
    UnloadModel(&replacement);

    delete replacement.modelData;
    replacement.modelData = nullptr;

    // 新しいモデルが有効になってから表示状態をリセットする。
    g_Player.load = TRUE;

    ResetPlayerPresentation();
    ResetCamera();

    return true;
}

//=============================================================================
// 進行中の切断操作を取り消す
//=============================================================================
void CancelPlayerCutGesture(void)
{
    isDragging = FALSE;

    startX = startY = 0.0f;
    currentX = currentY = 0.0f;
    endX = endY = 0.0f;
}

//=============================================================================
// プレイヤー情報を取得
//=============================================================================
PLAYER *GetPlayer(void)
{
	return &g_Player;
}

void UpdateMeshCut(void)
{
	if (IsMouseLeftTriggered())
	{
		isDragging = TRUE;
		startX = GetMousePosX();
		startY = GetMousePosY();
		currentX = startX;
		currentY = startY;
		endX = 0.0f;
		endY = 0.0f;
	}
	else if (IsMouseLeftPressed() && isDragging == TRUE)
	{
		long newX = GetMousePosX();
		long newY = GetMousePosY();
		currentX = newX;
		currentY = newY;

	}
	else if (!IsMouseLeftPressed() && isDragging == TRUE)
	{
		isDragging = FALSE;
		endX = GetMousePosX();
		endY = GetMousePosY();

		UpdateCuttingPlane();
	}
	//UpdateCuttingPlane();
	PrintDebugProc("StartX: %f, StartY: %f\n", startX, startY);
	PrintDebugProc("CurrentX: %f, CurrentY: %f\n", currentX, currentY);
	PrintDebugProc("EndX: %f, EndY: %f\n", endX, endY);

	
}

void MakeCuttingPlaneVertex()
{
	// 頂点バッファ生成
	D3D11_BUFFER_DESC bd;
	ZeroMemory(&bd, sizeof(bd));
	bd.Usage = D3D11_USAGE_DYNAMIC;
	bd.ByteWidth = sizeof(VERTEX_3D) * 4;
	bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

	GetDevice()->CreateBuffer(&bd, NULL, &g_CuttingPlaneVertexBuffer);
}

void UpdateCuttingPlane()
{
    float dx = endX - startX, dy = endY - startY;
    if (dx * dx + dy * dy < 4.0f) return;
    CAMERA* camera = GetCamera();
    XMMATRIX projection = XMLoadFloat4x4(&camera->mtxProjection);
    XMMATRIX inverseView = XMLoadFloat4x4(&camera->mtxInvView);
    auto ray = [&](float x, float y) {
        XMVECTOR direction = XMVectorSet(
            (2*x/SCREEN_WIDTH-1) / XMVectorGetX(projection.r[0]),
            (1-2*y/SCREEN_HEIGHT) / XMVectorGetY(projection.r[1]), 1, 0);
        return XMVector3Normalize(XMVector3TransformNormal(direction, inverseView));
    };
    XMVECTOR first = ray(startX, startY), last = ray(endX, endY);
    XMVECTOR normal = XMVector3Cross(last, first);
    if (XMVectorGetX(XMVector3LengthSq(normal)) < 1e-12f) return;
    XMVECTOR origin = XMVector3TransformCoord(XMVectorZero(), inverseView);
    XMStoreFloat3(&cuttingPlane.point, origin);
    XMStoreFloat3(&cuttingPlane.normal, XMVector3Normalize(normal));
    CutMeshes(&g_Player.model);

    if (!g_CuttingPlaneVertexBuffer) return;
    D3D11_MAPPED_SUBRESOURCE mapped = {};
    if (FAILED(GetDeviceContext()->Map(g_CuttingPlaneVertexBuffer, 0,
        D3D11_MAP_WRITE_DISCARD, 0, &mapped))) return;
    auto* vertices = static_cast<VERTEX_3D*>(mapped.pData);
    const XMVECTOR positions[] = {origin+first*15.0f, origin+first*300.0f,
                                 origin+last*15.0f, origin+last*300.0f};
    for (int i=0; i<4; ++i) {
        vertices[i] = {};
        XMStoreFloat3(&vertices[i].Position, positions[i]);
        vertices[i].Normal = cuttingPlane.normal;
        vertices[i].Diffuse = XMFLOAT4(1,1,1,1);
        vertices[i].TexCoord = XMFLOAT2(static_cast<float>(i%2), static_cast<float>(i/2));
    }
    GetDeviceContext()->Unmap(g_CuttingPlaneVertexBuffer, 0);
}

void DrawCuttingPlane()
{
    if (!g_CuttingPlaneVertexBuffer) return;
	// 頂点バッファ設定
	UINT stride = sizeof(VERTEX_3D);
	UINT offset = 0;
	GetDeviceContext()->IASetVertexBuffers(0, 1, &g_CuttingPlaneVertexBuffer, &stride, &offset);

	// プリミティブトポロジ設定
	GetDeviceContext()->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);

	MATERIAL material;
	ZeroMemory(&material, sizeof(material));
	material.Diffuse = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	material.noTexSampling = TRUE;
	SetMaterial(material);

	// ワールドマトリックスの初期化
	XMMATRIX mtxWorld = XMMatrixIdentity();

	// ワールドマトリックスの設定
	SetWorldMatrix(&mtxWorld);

	GetDeviceContext()->Draw(4, 0);
}