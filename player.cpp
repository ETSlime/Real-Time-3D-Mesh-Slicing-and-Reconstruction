//=============================================================================
//
// モデル処理 [player.cpp]
// Author : 
//
//=============================================================================
#include "main.h"
#include "model.h"
#include "input.h"
#include "camera.h"
#include "debugproc.h"
#include "player.h"
#include "ModelHistory.h"

//*****************************************************************************
// マクロ定義
//*****************************************************************************
#define	MODEL_PLAYER		"data/MODEL/01_cube.obj"		// 読み込むモデル名

#define	VALUE_MOVE			(2.0f)							// 移動量
#define	VALUE_ROTATE		(XM_PI * 0.02f)					// 回転量

#define PLAYER_SHADOW_SIZE	(0.4f)							// 影の大きさ
#define PLAYER_OFFSET_Y		(7.0f)							// プレイヤーの足元をあわせる
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif


//*****************************************************************************
// グローバル変数
//*****************************************************************************
static PLAYER				g_Player;						// プレイヤー
static BOOL g_IsDragging = FALSE;
static float g_StartX = 0, g_StartY = 0;
static float g_CurrentX = 0, g_CurrentY = 0;
static float g_EndX = 0, g_EndY = 0;
static ID3D11Buffer* g_CuttingPlaneVertexBuffer = NULL;	// 頂点バッファ
static Plane g_CuttingPlane;
static bool g_ShowCuttingPlane = false;         // 切断平面のデバッグ表示を有効にするか。
static bool g_HasCuttingPlane = false;          // 表示可能な切断平面の頂点が作成済みか。
static ModelHistory g_ModelHistory;             // 現在のプレイヤーモデルに対する編集履歴。

//=============================================================================
// 初期化処理
//=============================================================================
HRESULT InitPlayer(void)
{
	MakeCuttingPlaneVertex();
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
    // 終了時に、履歴が保持しているモデルデータとテクスチャ参照を解放する。
    g_ModelHistory.Clear();

    if (g_CuttingPlaneVertexBuffer) 
    {
        g_CuttingPlaneVertexBuffer->Release();
        g_CuttingPlaneVertexBuffer = nullptr;
    }

    g_IsDragging = FALSE;

	// モデルの解放処理
	if (g_Player.load)
	{
		UnloadModel(&g_Player.model);
		g_Player.load = FALSE;
	}

    g_HasCuttingPlane = false;
}

//=============================================================================
// 更新処理
//=============================================================================
void UpdatePlayer(void)
{
    // 履歴操作を行ったフレームでは、通常のプレイヤー更新を省略する。
    if (UpdateModelHistoryShortcuts())
        return;

	CAMERA *cam = GetCamera();

    // F2を押した瞬間だけ、切断平面の表示設定を切り替える。
    static bool previousF3Pressed = false;

    const bool f3Pressed = GetKeyboardPress(DIK_F3) != 0;

    if (f3Pressed && !previousF3Pressed)
    {
        g_ShowCuttingPlane = !g_ShowCuttingPlane;
    }

    previousF3Pressed = f3Pressed;

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
	//PrintDebugProc("Player:↑ → ↓ ←　Space\n");
	//PrintDebugProc("Player:X:%f Y:%f Z:%f\n", g_Player.pos.x, g_Player.pos.y, g_Player.pos.z);
#endif

	UpdateMeshCut();

    // 操作方法と現在の表示設定を表示する。
    const char* fillModeText =
        GetFillMode() == FILL_MODE_WIREFRAME
        ? "ワイヤーフレーム"
        : "塗りつぶし";

    const char* cuttingPlaneText =
        g_ShowCuttingPlane ? "表示" : "非表示";

    PrintDebugProc(
        "F1: 表示モード切替 [%s]\n",
        fillModeText);

    PrintDebugProc(
        "F3: 切断平面の表示切替 [%s]\n",
        cuttingPlaneText);

    PrintDebugProc("Ctrl + Z: 元に戻す\n");
    PrintDebugProc("Ctrl + Y / Ctrl + Shift + Z: やり直す\n");
}

//=============================================================================
// 描画処理
//=============================================================================
void DrawPlayer(void)
{
	// カリング無効
	SetCullingMode(CULL_MODE_NONE);

    // 表示と切断判定で、同じワールド行列を使用する。
    XMMATRIX mtxWorld = MeshCutter::CreateWorldMatrix(
        g_Player.scl,
        g_Player.rot,
        g_Player.pos);

    // 描画に使用するワールド行列を設定する。
    SetWorldMatrix(&mtxWorld);

    // 他の処理から参照できるよう、現在の行列を保存する。
    XMStoreFloat4x4(&g_Player.mtxWorld, mtxWorld);


	// モデル描画
	MATERIAL material;
	ZeroMemory(&material, sizeof(material));
	material.Diffuse = XMFLOAT4(1.0f, 1.0f, 0.0f, 1.0f);
	material.noTexSampling = TRUE;
	SetMaterial(material);
	DrawModel(&g_Player.model);


	DrawCuttingPlane();

	// カリング設定を戻す
	SetCullingMode(CULL_MODE_BACK);
}

//=============================================================================
// モデルの各サブセットを切断し、全結果が準備できた場合だけ更新する
//=============================================================================
void CutMeshes(DX11_MODEL* model)
{
    // ------------------------------------------------------------------------
    // 1. 入力モデルと切断平面を準備する
    // ------------------------------------------------------------------------
    if (model == nullptr
        || model->modelData == nullptr
        || model->modelData->SubsetArray == nullptr)
		return;

    // ワールド空間の切断平面を、モデルのローカル空間へ変換する。
    SetLocalCuttingPlane();

    const MeshCutter::Plane plane
    {
        MeshCutter::Read(g_CuttingPlane.localPoint),
        MeshCutter::Read(g_CuttingPlane.localNormal)
    };

    // ------------------------------------------------------------------------
    // 2. 全サブセットの入力を準備する
    // ------------------------------------------------------------------------
    const MODEL& source = *model->modelData;

    if (source.VertexArray == nullptr
        || source.IndexArray == nullptr
        || source.SubsetNum == 0)
        return;

    // 外側の添字は、元モデルのサブセット番号と一致させる。
    std::vector<std::vector<VERTEX_3D>> inputSubsets;
    inputSubsets.resize(source.SubsetNum);

    // 封口には、最初の空でないサブセットの材質を使用する。
    // SubsetNumは、有効なサブセット番号と重ならない初期値。
    std::size_t capMaterialIndex = source.SubsetNum;

    for (std::size_t subsetIndex = 0; subsetIndex < source.SubsetNum; ++subsetIndex)
    {
        const SUBSET& subset = source.SubsetArray[subsetIndex];

        const std::size_t startIndex = subset.StartIndex;
        const std::size_t indexCount = subset.IndexNum;

        // 減算前に開始位置を確認し、範囲外参照を防ぐ。
        if (startIndex > source.IndexNum
            || indexCount > source.IndexNum - startIndex)
            return;

        // 3インデックスごとに1つの三角形を構成する。
        if (indexCount % 3 != 0)
            return;

        // 空のサブセットは番号だけ維持し、頂点を追加しない。
        if (indexCount == 0)
            continue;

        if (capMaterialIndex == source.SubsetNum)
            capMaterialIndex = subsetIndex;

        std::vector<VERTEX_3D>& input = inputSubsets[subsetIndex];
        input.reserve(indexCount);

        // インデックスを解決し、展開した三角形頂点を保存する。
        for (std::size_t offset = 0; offset < indexCount; ++offset)
        {
            const unsigned short vertexIndex = source.IndexArray[startIndex + offset];

            if (vertexIndex >= source.VertexNum)
                return;

            input.push_back(source.VertexArray[vertexIndex]);
        }
    }

    // 三角形を持つサブセットがなければ、処理を行わない。
    if (capMaterialIndex == source.SubsetNum)
        return;

    // ------------------------------------------------------------------------
    // 3. モデル全体を一度に切断する
    // ------------------------------------------------------------------------
    const auto result = MeshCutter::SliceModel(
        inputSubsets,
        plane,
        MeshCutter::Options{},
        PlayerCutSupport::IsInsideStroke);

    // 未変更の場合は、元モデルと編集履歴を維持する。
    if (result.status == MeshCutter::Status::Unchanged)
        return;

    // 一部のサブセットだけを採用せず、モデル全体の結果で判断する。
    if (result.status != MeshCutter::Status::Cut)
    {
        PrintDebugProc(
            "モデル全体の切断に失敗しました: status %d\n",
            static_cast<int>(result.status));

        return;
    }

    // ------------------------------------------------------------------------
    // 4. 原表面と封口を、材質付きの一時結果へ変換する
    // ------------------------------------------------------------------------
    std::vector<PlayerCutSupport::PendingSubset> pending;

    std::size_t totalVertexCount = 0;
    std::size_t totalSubsetCount = 0;

    // 原サブセットを順に処理し、最後の1回で封口を処理する。
    for (std::size_t outputIndex = 0; outputIndex <= result.subsets.size(); ++outputIndex)
    {
        const bool isCap = outputIndex == result.subsets.size();

        // 封口と原表面は、同じ形式の両側頂点配列を持つ。
        const MeshCutter::SurfaceResult<VERTEX_3D>& surface =
            isCap
            ? result.caps
            : result.subsets[outputIndex];

        const bool hasNegative = !surface.negative.empty();
        const bool hasPositive = !surface.positive.empty();

        // 両側とも空なら、描画用サブセットを作成しない。
        if (!hasNegative && !hasPositive)
            continue;

        PlayerCutSupport::PendingSubset staged;

        // 原表面は元の材質、封口は選択した材質を使用する。
        const std::size_t materialIndex = isCap ? capMaterialIndex : outputIndex;
        staged.material = source.SubsetArray[materialIndex].Material;

        if (hasNegative)
        {
            // 負側があれば、基本出力として保持する。
            staged.primaryVertices = PlayerCutSupport::CopyVertices(surface.negative);

            totalVertexCount += surface.negative.size();
            ++totalSubsetCount;

            if (hasPositive)
            {
                // 両側に頂点がある場合、正側を追加出力にする。
                staged.additionalVertices = PlayerCutSupport::CopyVertices(surface.positive);

                totalVertexCount += surface.positive.size();
                ++totalSubsetCount;
            }
        }
        else
        {
            // 正側しかない場合は、それを基本出力として保持する。
            staged.primaryVertices = PlayerCutSupport::CopyVertices(surface.positive);

            totalVertexCount += surface.positive.size();
            ++totalSubsetCount;
        }

        pending.push_back(std::move(staged));
    }

    // ------------------------------------------------------------------------
    // 5. 既存モデル形式の容量と、更新キューの状態を確認する
    // ------------------------------------------------------------------------
        
    // 現在のモデルは、頂点数や開始位置などを16ビットで保持している。
    const std::size_t maximumCount = (std::numeric_limits<unsigned short>::max)();

    if (totalVertexCount > maximumCount
        || totalSubsetCount > maximumCount)
    {
        PrintDebugProc( "Mesh cut rejected: legacy 16-bit model capacity exceeded.\n");
        return;
    }

    // 他の処理が保持する更新用配列を上書きしない。
    const bool hasPendingUpdate = model->newVertexArrays.getSize() != 0
        || model->oldVertexArrays.getSize() != 0;

    if (hasPendingUpdate)
	{
		PrintDebugProc("Mesh cut rejected: model update already pending.\n");
		return;
	}

    // ------------------------------------------------------------------------
	// 6. 取り消し履歴を準備し、モデル更新キューへ切断結果を引き渡す
    // ------------------------------------------------------------------------

    // モデルを変更する前に、取り消し用の状態を保存する。
    const HRESULT historyResult = g_ModelHistory.PrepareCut(model);

    if (FAILED(historyResult))
    {
        PrintDebugProc(
            "取り消し履歴の保存に失敗しました: 0x%08X\n",
            static_cast<unsigned int>(historyResult));

        return;
    }

    // この関数を抜ける際に、切り取り履歴を必ず確定または取り消す。
    struct CutHistoryGuard
    {
        ModelHistory& history;
        bool succeeded = false;

        ~CutHistoryGuard()
        {
            history.FinishCut(succeeded);
        }
    };

    CutHistoryGuard historyGuard{ g_ModelHistory };


    // ------------------------------------------------------------------------
    // 7. 完成した全サブセットを、既存のモデル更新キューへ引き渡す
    // ------------------------------------------------------------------------

    model->newSubsetMaterial.clear();
    model->oldSubsetMaterial.clear();

    for (PlayerCutSupport::PendingSubset& subset : pending)
    {
        // 基本の出力は、切断の有無にかかわらず存在する。
        // release後の配列は、TryUpdateModelが解放を担当する。
        model->newVertexArrays.push_back(subset.primaryVertices.release());
        model->newSubsetMaterial.push_back(subset.material);

        // 切断によって追加の出力が発生した場合だけ登録する。
        if (subset.additionalVertices)
        {
            model->newVertexArrays.push_back(subset.additionalVertices.release());
            model->newSubsetMaterial.push_back(subset.material);
        }
    }

    // ------------------------------------------------------------------------
    // 8. CPU配列とGPUバッファを更新する
    // ------------------------------------------------------------------------
    
    // TryUpdateModelは、成功・失敗のどちらでも更新キューを消費する。
    // GPUバッファの作成に失敗した場合は、元のモデルを維持する。
    HRESULT updateResult = PlayerCutSupport::TryUpdateModel(model);

    if (FAILED(updateResult))
    {
        PrintDebugProc(
            "切り取り結果の反映に失敗しました: 0x%08X\n",
            static_cast<unsigned int>(updateResult));

        return;
    }

    // モデルの置き換えに成功したため、切り取り履歴を確定する。
    historyGuard.succeeded = true;
}

//=============================================================================
// ワールド空間の切断平面を、モデルのローカル空間へ変換する
//=============================================================================
void SetLocalCuttingPlane(void)
{
    // 描画時と同じ変換を使用する。
    const XMMATRIX worldMatrix =
        MeshCutter::CreateWorldMatrix(
            g_Player.scl,
            g_Player.rot,
            g_Player.pos);

    XMStoreFloat4x4(&g_Player.mtxWorld, worldMatrix);

    // ワールド座標からローカル座標へ戻すための逆行列。
    const XMMATRIX inverseWorldMatrix = XMMatrixInverse(nullptr, worldMatrix);

    // 切断平面上の点をローカル空間へ変換する。
    // 点の変換には平行移動も含める。
    const XMVECTOR worldPoint = XMLoadFloat3(&g_CuttingPlane.point);
    const XMVECTOR localPoint = XMVector3TransformCoord(worldPoint, inverseWorldMatrix);

    // ワールド法線をローカル法線へ戻す場合は、
    // ワールド行列の転置を使用する。
    const XMMATRIX transposedWorldMatrix = XMMatrixTranspose(worldMatrix);

	// ワールド法線をローカル法線へ変換する。
    const XMVECTOR worldNormal = XMLoadFloat3(&g_CuttingPlane.normal);
    XMVECTOR localNormal =
        XMVector3TransformNormal(
            worldNormal,
            transposedWorldMatrix);

    // スケール変換後も単位法線として使用できるようにする。
    localNormal = XMVector3Normalize(localNormal);

    // 三角形の切断判定で使用するローカル平面を保存する。
    XMStoreFloat3(&g_CuttingPlane.localPoint, localPoint);
    XMStoreFloat3(&g_CuttingPlane.localNormal, localNormal);

    // 交点をワールド空間へ戻す処理のために保存する。
    XMStoreFloat4x4(&g_CuttingPlane.localmtxWorld, worldMatrix);
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
    g_IsDragging = FALSE;

    g_StartX = 0.0f;
    g_StartY = 0.0f;

    g_CurrentX = 0.0f;
    g_CurrentY = 0.0f;

    g_EndX = 0.0f;
    g_EndY = 0.0f;

    // 前回の切断平面の情報をクリアする。
    g_CuttingPlane = {};

    // 前のモデルで生成した切断平面を表示しない。
    g_HasCuttingPlane = false;

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

    // 新しいモデルが有効になってから表示状態をリセットする。
    g_Player.load = TRUE;

    ResetPlayerPresentation();
    ResetCamera();

    // 別のモデルへ切り替えたため、以前の編集履歴を破棄する。
    g_ModelHistory.Clear();

    return true;
}

//=============================================================================
// 進行中の切断操作を取り消す
//=============================================================================
void CancelPlayerCutGesture(void)
{
    g_IsDragging = FALSE;

    g_StartX = g_StartY = 0.0f;
    g_CurrentX = g_CurrentY = 0.0f;
    g_EndX = g_EndY = 0.0f;
}

bool UpdateModelHistoryShortcuts(void)
{
    // 修飾キーは左右どちらでも使用できる。
    const bool controlPressed = GetKeyboardPress(DIK_LCONTROL) || GetKeyboardPress(DIK_RCONTROL);
    const bool shiftPressed = GetKeyboardPress(DIK_LSHIFT) || GetKeyboardPress(DIK_RSHIFT);

    const bool zPressed = GetKeyboardPress(DIK_Z) != 0;
    const bool yPressed = GetKeyboardPress(DIK_Y) != 0;

    // 押し続けている間に、毎フレーム実行されることを防ぐ。
    static bool previousZPressed = false;
    static bool previousYPressed = false;

    const bool zTriggered = zPressed && !previousZPressed;
    const bool yTriggered = yPressed && !previousYPressed;

    previousZPressed = zPressed;
    previousYPressed = yPressed;

    // キー状態は毎フレーム更新し、操作できない場合だけ処理を省略する。
    if (!g_Player.load || g_IsDragging)
        return false;

    if (!controlPressed)
        return false;

    HRESULT result = S_FALSE;

    // Ctrl + Shift + Z は、Ctrl + Z より先に判定する。
    if (yTriggered || (shiftPressed && zTriggered))
    {
        result = g_ModelHistory.Redo(&g_Player.model);
    }
    else if (zTriggered)
    {
        result = g_ModelHistory.Undo(&g_Player.model);
    }
    else
    {
        return false;
    }

    if (result == S_OK)
    {
        // 過去の切断平面は、復元後の形状と一致するとは限らない。
        g_HasCuttingPlane = false;
    }
    else if (FAILED(result))
    {
        PrintDebugProc(
            "編集履歴の復元に失敗しました: 0x%08X\n",
            static_cast<unsigned int>(result));
    }

    // 履歴が空の場合も、ショートカット入力として処理済みにする。
    return true;
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
        g_IsDragging = TRUE;
        g_StartX = GetMousePosX();
        g_StartY = GetMousePosY();
		g_CurrentX = g_StartX;
		g_CurrentY = g_StartY;
        g_EndX = 0.0f;
        g_EndY = 0.0f;
	}
	else if (IsMouseLeftPressed() && g_IsDragging == TRUE)
	{
		long newX = GetMousePosX();
		long newY = GetMousePosY();
		g_CurrentX = newX;
		g_CurrentY = newY;

	}
	else if (!IsMouseLeftPressed() && g_IsDragging == TRUE)
	{
        g_IsDragging = FALSE;
        g_EndX = GetMousePosX();
        g_EndY = GetMousePosY();

		UpdateCuttingPlane();
	}

	//PrintDebugProc("StartX: %f, StartY: %f\n", startX, startY);
	//PrintDebugProc("CurrentX: %f, CurrentY: %f\n", currentX, currentY);
	//PrintDebugProc("EndX: %f, EndY: %f\n", endX, endY);
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

//=============================================================================
// マウス操作から切断平面を作成し、メッシュの切断を実行する
//=============================================================================
void UpdateCuttingPlane()
{
    // ごく短いドラッグでは、安定した切断方向を決められない。
    const float dragX = g_EndX - g_StartX;
    const float dragY = g_EndY - g_StartY;

    const float dragLengthSquared =
        dragX * dragX + dragY * dragY;

    // 2ピクセル未満の操作は切断として扱わない。
    if (dragLengthSquared < 4.0f)
		return;

    CAMERA* camera = GetCamera();

	// スクリーン座標からワールド空間のレイを作成するために必要な行列を取得する。
    XMMATRIX projectionMatrix = XMLoadFloat4x4(&camera->mtxProjection);
    XMMATRIX inverseViewMatrix = XMLoadFloat4x4(&camera->mtxInvView);

    // ドラッグ開始位置と終了位置に対応するレイ方向を求める。
    const XMVECTOR startDirection =
        MeshCutter::CreateWorldRayDirection(
            g_StartX,
            g_StartY,
            projectionMatrix,
            inverseViewMatrix);

    const XMVECTOR endDirection =
        MeshCutter::CreateWorldRayDirection(
            g_EndX,
            g_EndY,
            projectionMatrix,
            inverseViewMatrix);

    // ビュー空間の原点をワールド空間へ戻すと、カメラ位置になる。
    const XMVECTOR rayOrigin = XMVector3TransformCoord(XMVectorZero(), inverseViewMatrix);

    // 2本のレイを含む平面の法線を求める。
    const XMVECTOR normalCandidate = XMVector3Cross(endDirection, startDirection);

    // レイがほぼ平行な場合は、法線を安定して正規化できない。
    const float normalLengthSquared = XMVectorGetX(XMVector3LengthSq(normalCandidate));
	if (normalLengthSquared < 1.0e-12f)
		return;

	// レイの交差方向を正規化して、切断平面の法線として使用する。
    const XMVECTOR planeNormal = XMVector3Normalize(normalCandidate);

    // 切断平面はカメラ位置を通り、2本のレイを含む。
    XMStoreFloat3(&g_CuttingPlane.point, rayOrigin);
    XMStoreFloat3(&g_CuttingPlane.normal, planeNormal);

    // 計算したワールド空間の平面を使用して切断する。
    if (g_Player.load)
        CutMeshes(&g_Player.model);

    // 以下は切断平面のデバッグ表示用データを更新する処理。
    // メッシュ切断の結果には影響しない。
    if (!g_CuttingPlaneVertexBuffer) return;

    D3D11_MAPPED_SUBRESOURCE mappedResource = {};

    const HRESULT result = GetDeviceContext()->Map(
        g_CuttingPlaneVertexBuffer,
        0,
        D3D11_MAP_WRITE_DISCARD,
        0,
        &mappedResource);

    // 書き込み先を取得できなかった場合は、頂点を更新しない。
    if (FAILED(result))
		return;

    VERTEX_3D* vertices = static_cast<VERTEX_3D*>(mappedResource.pData);

    // デバッグ表示に使用する、レイ上の近点と遠点。
    // 実際の切断範囲を15から300に制限する値ではない。
    const float previewNearDistance = 15.0f;
    const float previewFarDistance = 300.0f;

    const XMVECTOR previewPositions[4] =
    {
        rayOrigin + startDirection * previewNearDistance,
        rayOrigin + startDirection * previewFarDistance,
        rayOrigin + endDirection * previewNearDistance,
        rayOrigin + endDirection * previewFarDistance
    };

    for (int i = 0; i < 4; ++i)
    {
        // 未設定の頂点属性が残らないように初期化する。
        vertices[i] = {};

        XMStoreFloat3(
            &vertices[i].Position,
            previewPositions[i]);

        vertices[i].Normal = g_CuttingPlane.normal;

        vertices[i].Diffuse =
            XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);

        // 元の4頂点に対応するUV配置を維持する。
        vertices[i].TexCoord = XMFLOAT2(
            static_cast<float>(i % 2),
            static_cast<float>(i / 2));
    }

    // Mapで取得した書き込み先の使用を終了する。
    GetDeviceContext()->Unmap(g_CuttingPlaneVertexBuffer, 0);

    // 頂点バッファへの書き込みが完了した。
    g_HasCuttingPlane = true;
}

void DrawCuttingPlane()
{
    // 非表示設定、未生成、またはバッファ未作成の場合は描画しない。
    if (!g_ShowCuttingPlane
        || !g_HasCuttingPlane
        || g_CuttingPlaneVertexBuffer == nullptr)
		return;

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

bool PlayerCutSupport::IsInsideStroke(MeshCutter::Vec3 point)

{
    CAMERA* camera = GetCamera();

    // 描画時と同じワールド行列を使用する。
    const XMMATRIX worldMatrix =
        MeshCutter::CreateWorldMatrix(g_Player.scl, g_Player.rot, g_Player.pos);

    const XMMATRIX viewMatrix = XMLoadFloat4x4(&camera->mtxView);
    const XMMATRIX projectionMatrix = XMLoadFloat4x4(&camera->mtxProjection);
    const XMMATRIX worldViewProjection = worldMatrix * viewMatrix * projectionMatrix;

    // 点として変換するため、同次座標のWを1にする。
    const XMVECTOR localPoint = XMVectorSet(point.x, point.y, point.z, 1.0f);

    // この段階では透視除算を行わず、クリップ座標を取得する。
    const XMVECTOR clipPoint = XMVector4Transform(localPoint, worldViewProjection);

    const float clipW = XMVectorGetW(clipPoint);

    // 現在の透視投影で、カメラ後方またはWが小さすぎる点を除外する。
    if (clipW <= 0.00001f)
        return false;

    // 透視除算により、正規化デバイス座標へ変換する。
    const float ndcX = XMVectorGetX(clipPoint) / clipW;
    const float ndcY = XMVectorGetY(clipPoint) / clipW;

    // 正規化デバイス座標からウィンドウ座標へ変換する。
    const float screenX = (ndcX + 1.0f) * 0.5f * SCREEN_WIDTH;
    const float screenY = (1.0f - ndcY) * 0.5f * SCREEN_HEIGHT;

    // 水平・垂直ドラッグも扱えるよう、既存の余白を維持する。
    const float margin = 2.0f;

    const float minimumX = (std::min)(g_StartX, g_EndX) - margin;
    const float maximumX = (std::max)(g_StartX, g_EndX) + margin;
    const float minimumY = (std::min)(g_StartY, g_EndY) - margin;
    const float maximumY = (std::max)(g_StartY, g_EndY) + margin;

    return screenX >= minimumX
        && screenX <= maximumX
        && screenY >= minimumY
        && screenY <= maximumY;
}

std::unique_ptr<SimpleArray<VERTEX_3D>> PlayerCutSupport::CopyVertices(const std::vector<VERTEX_3D>& vertices)
{
    std::unique_ptr<SimpleArray<VERTEX_3D>> array(new SimpleArray<VERTEX_3D>);

    for (const VERTEX_3D& vertex : vertices)
    {
        array->push_back(vertex);
    }

    return array;
}

//=============================================================================
// 更新用配列からモデルを再構築し、準備完了後に旧モデルと置き換える
//=============================================================================
HRESULT PlayerCutSupport::TryUpdateModel(DX11_MODEL* model)
{
    if (model == nullptr || model->modelData == nullptr)
    {
        return E_INVALIDARG;
    }

    // この更新処理が受け取った一時配列の解放を担当する。
    // 関数を抜ける際に、成功・失敗を問わず実行する。
    struct QueueCleanup
    {
        DX11_MODEL* target;

        ~QueueCleanup()
        {
            // 新しい頂点データとして受け取った配列を解放する。
            for (int index = 0; index < target->newVertexArrays.getSize(); ++index)
            {
                delete target->newVertexArrays[index];
            }

            // 既存データとして受け取った配列を解放する。
            for (int index = 0; index < target->oldVertexArrays.getSize(); ++index)
            {
                delete target->oldVertexArrays[index];
            }

            // 解放済みポインタをキューに残さない。
            target->newVertexArrays.clear();
            target->oldVertexArrays.clear();

            // マテリアルの一時的な対応表もクリアする。
            // ここではテクスチャのReleaseは行わない。
            target->newSubsetMaterial.clear();
            target->oldSubsetMaterial.clear();
        }
    };

	// 関数のスコープを抜ける際に、キューの解放処理を自動的に実行する。
	QueueCleanup cleanup{ model };

    // ------------------------------------------------------------------------
    // 1. 頂点配列とマテリアルの対応関係を確認する
    // ------------------------------------------------------------------------
    if (model->newVertexArrays.getSize()
        != model->newSubsetMaterial.getSize()
        || model->oldVertexArrays.getSize()
        != model->oldSubsetMaterial.getSize())
        return E_INVALIDARG;

    try
    {
        // 新旧のキューを、同じ処理で順番に読み取る。
        // 各キューと対応するマテリアルキューの順序をそろえる。
        SimpleArray<SimpleArray<VERTEX_3D>*>* vertexQueues[2] =
        {
            &model->newVertexArrays,
            &model->oldVertexArrays
        };

        SimpleArray<MODEL_MATERIAL>* materialQueues[2] =
        {
            &model->newSubsetMaterial,
            &model->oldSubsetMaterial
        };

        // --------------------------------------------------------------------
        // 2. 必要な頂点数とサブセット数を集計する
        // --------------------------------------------------------------------
        const std::size_t maximumCount = (std::numeric_limits<unsigned short>::max)();

        std::size_t totalVertexCount = 0;
        std::size_t totalSubsetCount = 0;

        for (int queueIndex = 0; queueIndex < 2; ++queueIndex)
        {
            SimpleArray<SimpleArray<VERTEX_3D>*>& vertexArrays = *vertexQueues[queueIndex];

            for (int arrayIndex = 0; arrayIndex < vertexArrays.getSize(); ++arrayIndex)
            {
                SimpleArray<VERTEX_3D>* vertices = vertexArrays[arrayIndex];

                if (vertices == nullptr)
                    return E_INVALIDARG;

                const int vertexCount = vertices->getSize();

                // 三角形の展開配列なので、頂点数は3の倍数である必要がある。
                if (vertexCount < 0 || vertexCount % 3 != 0)
                    return E_INVALIDARG;

                // 空の配列は、出力サブセットとして数えない。
                if (vertexCount == 0)
                    continue;

                const std::size_t count = static_cast<std::size_t>(vertexCount);

                // 加算前に確認し、16ビット形式の容量超過を防ぐ。
                if (count > maximumCount - totalVertexCount)
                    return E_INVALIDARG;

                totalVertexCount += count;
                ++totalSubsetCount;
            }
        }

        if (totalVertexCount == 0 || totalSubsetCount > maximumCount)
            return E_INVALIDARG;

        // --------------------------------------------------------------------
        // 3. 新しいCPU側配列を確保する
        // --------------------------------------------------------------------

        // 旧モデルにはまだ触れず、別の配列へ結果を構築する。
        std::unique_ptr<VERTEX_3D[]> newVertices(new VERTEX_3D[totalVertexCount]);
        std::unique_ptr<unsigned short[]> newIndices(new unsigned short[totalVertexCount]);
        std::unique_ptr<SUBSET[]> newSubsets(new SUBSET[totalSubsetCount]);

        std::size_t vertexOffset = 0;
        std::size_t outputSubsetIndex = 0;

        // --------------------------------------------------------------------
        // 4. 各入力配列を、新しいモデル配列へコピーする
        // --------------------------------------------------------------------
        for (int queueIndex = 0; queueIndex < 2; ++queueIndex)
        {
            SimpleArray<SimpleArray<VERTEX_3D>*>& vertexArrays = *vertexQueues[queueIndex];

            SimpleArray<MODEL_MATERIAL>& materials = *materialQueues[queueIndex];

            for (int arrayIndex = 0; arrayIndex < vertexArrays.getSize(); ++arrayIndex)
            {
                SimpleArray<VERTEX_3D>* vertices = vertexArrays[arrayIndex];

                const int vertexCount = vertices->getSize();

                if (vertexCount == 0)
                    continue;

                SUBSET& outputSubset = newSubsets[outputSubsetIndex];

                outputSubset.StartIndex = static_cast<unsigned short>(vertexOffset);
                outputSubset.IndexNum = static_cast<unsigned short>(vertexCount);
                outputSubset.Material = materials[arrayIndex];

                for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex)
                {
                    newVertices[vertexOffset] = (*vertices)[vertexIndex];

                    // 入力は展開済みなので、頂点と同じ番号を使用する。
                    newIndices[vertexOffset] = static_cast<unsigned short>(vertexOffset);

                    ++vertexOffset;
                }

                ++outputSubsetIndex;
            }
        }

        // --------------------------------------------------------------------
        // 5. 新しいGPUバッファを作成する
        // --------------------------------------------------------------------

        // まだモデルへ渡していないバッファを、一時的に所有する。
        struct BufferOwner
        {
            ID3D11Buffer* pointer = nullptr;

            ~BufferOwner()
            {
                if (pointer != nullptr)
                {
                    pointer->Release();
                }
            }
        };

        BufferOwner newVertexBuffer;
        BufferOwner newIndexBuffer;

        D3D11_BUFFER_DESC vertexBufferDescription = {};
        vertexBufferDescription.Usage = D3D11_USAGE_DEFAULT;
        vertexBufferDescription.ByteWidth =static_cast<UINT>(sizeof(VERTEX_3D) * totalVertexCount);
        vertexBufferDescription.BindFlags = D3D11_BIND_VERTEX_BUFFER;

        D3D11_SUBRESOURCE_DATA vertexBufferData = {};
        vertexBufferData.pSysMem = newVertices.get();

        HRESULT result = GetDevice()->CreateBuffer(
            &vertexBufferDescription,
            &vertexBufferData,
            &newVertexBuffer.pointer);

        if (FAILED(result))
            return result;

        D3D11_BUFFER_DESC indexBufferDescription = {};
        indexBufferDescription.Usage = D3D11_USAGE_DEFAULT;
        indexBufferDescription.ByteWidth = static_cast<UINT>(sizeof(unsigned short) * totalVertexCount);
        indexBufferDescription.BindFlags = D3D11_BIND_INDEX_BUFFER;

        D3D11_SUBRESOURCE_DATA indexBufferData = {};
        indexBufferData.pSysMem = newIndices.get();

        result = GetDevice()->CreateBuffer(
            &indexBufferDescription,
            &indexBufferData,
            &newIndexBuffer.pointer);

        if (FAILED(result))
            return result;

        // --------------------------------------------------------------------
        // 6. 新しいモデルへの置き換えを確定する
        // --------------------------------------------------------------------

        // UnloadModelはMODEL本体も削除するため、
        // 置き換え先を旧モデルの破棄前に確保する。
        std::unique_ptr<MODEL> newModelData(new MODEL{});

        // 新しい各サブセットが、テクスチャ参照を1つずつ所有する。
        // 旧モデルのReleaseより前に参照数を増やす。
        for (std::size_t subsetIndex = 0; subsetIndex < totalSubsetCount; ++subsetIndex)
        {
            ID3D11ShaderResourceView* texture = newSubsets[subsetIndex].Material.Texture;

            if (texture != nullptr)
                texture->AddRef();
        }

        // CPU配列とGPUバッファの準備が完了したため、
        // ここで初めて旧モデルを解放する。
        UnloadModel(model);

        model->modelData = newModelData.release();

        MODEL& target = *model->modelData;

        // 配列の所有権を、新しいモデルへ引き渡す。
        target.VertexArray = newVertices.release();
        target.VertexNum = static_cast<unsigned short>(totalVertexCount);

        target.IndexArray = newIndices.release();
        target.IndexNum = static_cast<unsigned short>(totalVertexCount);

        target.SubsetArray = newSubsets.release();
        target.SubsetNum = static_cast<unsigned short>(totalSubsetCount);

        // GPUバッファの所有権もモデルへ引き渡す。
        model->VertexBuffer = newVertexBuffer.pointer;
        newVertexBuffer.pointer = nullptr;

        model->IndexBuffer = newIndexBuffer.pointer;
        newIndexBuffer.pointer = nullptr;

        return S_OK;
    }
    catch (const std::bad_alloc&)
    {
        // 一時リソースは、それぞれの所有者が解放する。
        // 旧モデルを破棄する前のメモリ確保失敗では、旧モデルを維持する。
        return E_OUTOFMEMORY;
    }
}
