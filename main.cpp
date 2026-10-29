//=============================================================================
//
// メイン処理 [main.cpp]
// Author : 
//
//=============================================================================
#include "main.h"
#include "renderer.h"
#include "model.h"
#include "input.h"
#include "camera.h"
#include "debugproc.h"
#include "field.h"
#include "player.h"
#include "enemy.h"
#include "ObjImporter.h"
#include <shellapi.h>
#include <string>
#include <vector>
#include <cwctype>
#include <algorithm>

#pragma comment(lib, "Shell32.lib")

//*****************************************************************************
// マクロ定義
//*****************************************************************************
#define CLASS_NAME		"AppClass"			// ウインドウのクラス名
#define WINDOW_NAME		"メッシュ表示"		// ウインドウのキャプション名

//*****************************************************************************
// プロトタイプ宣言
//*****************************************************************************
LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
HRESULT Init(HINSTANCE hInstance, HWND hWnd, BOOL bWindow);
void Uninit(void);
void Update(void);
void Draw(void);
bool UpdatePendingObjLoad(void);

//*****************************************************************************
// グローバル変数:
//*****************************************************************************
long g_MouseX = 0;
long g_MouseY = 0;

namespace
{
	// 次の更新処理で読み込むOBJファイルのパス。
	// 空の場合は、読み込み要求がないことを表す。
	std::wstring g_PendingObjPath;

	// ドロップされたファイルを検証し、読み込み要求として保存する。
	void ReceiveDroppedObj(HWND hWnd, HDROP drop)
	{
		// 関数を抜ける際に、ドロップ情報を必ず解放する。
		// 途中でreturnした場合や例外が発生した場合も実行される。
		struct DropGuard
		{
			HDROP handle;

			~DropGuard()
			{
				DragFinish(handle);
			}
		};

		DropGuard guard{ drop };

		// 0xFFFFFFFFを指定すると、ドロップされたファイル数を取得できる。
		const UINT fileCount =
			DragQueryFileW(drop, 0xFFFFFFFFu, nullptr, 0);

		if (fileCount != 1)
		{
			MessageBoxW(
				hWnd,
				L"OBJファイルを1つだけドロップしてください。",
				L"モデルの読み込み",
				MB_OK | MB_ICONWARNING);

			return;
		}

		// バッファを指定せずに呼び出し、パスの文字数を調べる。
		// 戻り値には、終端のヌル文字は含まれない。
		const UINT pathLength =
			DragQueryFileW(drop, 0, nullptr, 0);

		if (pathLength == 0)
		{
			return;
		}

		// 終端のヌル文字を格納するため、1文字分多く確保する。
		std::vector<wchar_t> buffer(
			static_cast<std::size_t>(pathLength) + 1,
			L'\0');

		// 先頭のファイルのパスを取得する。
		const UINT copiedLength = DragQueryFileW(
			drop,
			0,
			buffer.data(),
			pathLength + 1);

		if (copiedLength != pathLength)
		{
			MessageBoxW(
				hWnd,
				L"ファイルパスを取得できませんでした。",
				L"モデルの読み込み",
				MB_OK | MB_ICONWARNING);

			return;
		}

		std::wstring path(buffer.data());

		// 最後の拡張子を取得する。
		const std::size_t dotPosition = path.find_last_of(L'.');

		std::wstring extension;

		if (dotPosition != std::wstring::npos)
		{
			extension = path.substr(dotPosition);
		}

		// 大文字の「.OBJ」も受け付けるため、小文字に統一する。
		for (wchar_t& character : extension)
		{
			character = static_cast<wchar_t>(
				std::towlower(character));
		}

		if (extension != L".obj")
		{
			MessageBoxW(
				hWnd,
				L"対応しているファイル形式は.objです。",
				L"モデルの読み込み",
				MB_OK | MB_ICONWARNING);

			return;
		}

		// この時点ではモデルを読み込まない。
		// パスだけを保存し、実際の読み込みはUpdateで行う。
		g_PendingObjPath = path;

		// 動作確認用。確認が済んだら削除する。
		//MessageBoxW(
		//	hWnd,
		//	g_PendingObjPath.c_str(),
		//	L"OBJファイルを受け取りました",
		//	MB_OK | MB_ICONINFORMATION);
	}

	// Windowsのワイド文字パスを、既存ローダー用の文字列へ変換する。
// 現在のANSIコードページで表現できないパスは受け付けない。
	bool ConvertObjPath(
		const std::wstring& widePath,
		std::string& path)
	{
		BOOL usedDefaultCharacter = FALSE;

		const int requiredSize = WideCharToMultiByte(
			CP_ACP,
			WC_NO_BEST_FIT_CHARS,
			widePath.c_str(),
			-1,
			nullptr,
			0,
			nullptr,
			&usedDefaultCharacter);

		if (requiredSize == 0 || usedDefaultCharacter)
		{
			return false;
		}

		std::vector<char> buffer(
			static_cast<std::size_t>(requiredSize));

		usedDefaultCharacter = FALSE;

		const int writtenSize = WideCharToMultiByte(
			CP_ACP,
			WC_NO_BEST_FIT_CHARS,
			widePath.c_str(),
			-1,
			buffer.data(),
			requiredSize,
			nullptr,
			&usedDefaultCharacter);

		if (writtenSize == 0 || usedDefaultCharacter)
		{
			return false;
		}

		path = buffer.data();

		// 既存の関連ファイル検索処理は「/」を区切りとして使用する。
		std::replace(path.begin(), path.end(), '\\', '/');

		return true;
	}
}


#ifdef _DEBUG
int		g_CountFPS;							// FPSカウンタ
char	g_DebugStr[2048] = WINDOW_NAME;		// デバッグ文字表示用

#endif


//=============================================================================
// メイン関数
//=============================================================================
int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);	// 無くても良いけど、警告が出る（未使用宣言）
	UNREFERENCED_PARAMETER(lpCmdLine);		// 無くても良いけど、警告が出る（未使用宣言）

	// 時間計測用
	DWORD dwExecLastTime;
	DWORD dwFPSLastTime;
	DWORD dwCurrentTime;
	DWORD dwFrameCount;

	WNDCLASSEX	wcex = {
		sizeof(WNDCLASSEX),
		CS_CLASSDC,
		WndProc,
		0,
		0,
		hInstance,
		NULL,
		LoadCursor(NULL, IDC_ARROW),
		(HBRUSH)(COLOR_WINDOW + 1),
		NULL,
		CLASS_NAME,
		NULL
	};
	HWND		hWnd;
	MSG			msg;
	
	// ウィンドウクラスの登録
	RegisterClassEx(&wcex);

	// ウィンドウの作成
	hWnd = CreateWindow(CLASS_NAME,
		WINDOW_NAME,
		WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT,																		// ウィンドウの左座標
		CW_USEDEFAULT,																		// ウィンドウの上座標
		SCREEN_WIDTH + GetSystemMetrics(SM_CXDLGFRAME) * 2,									// ウィンドウ横幅
		SCREEN_HEIGHT + GetSystemMetrics(SM_CXDLGFRAME) * 2 + GetSystemMetrics(SM_CYCAPTION),	// ウィンドウ縦幅
		NULL,
		NULL,
		hInstance,
		NULL);

	// 初期化処理(ウィンドウを作成してから行う)
	if(FAILED(Init(hInstance, hWnd, TRUE)))
	{
		return -1;
	}

	// 初期化に成功したウィンドウで、ファイルのドロップを受け付ける。
	DragAcceptFiles(hWnd, TRUE);

	// フレームカウント初期化
	timeBeginPeriod(1);	// 分解能を設定
	dwExecLastTime = dwFPSLastTime = timeGetTime();	// システム時刻をミリ秒単位で取得
	dwCurrentTime = dwFrameCount = 0;

	// ウインドウの表示(初期化処理の後に呼ばないと駄目)
	ShowWindow(hWnd, nCmdShow);
	UpdateWindow(hWnd);
	
	// メッセージループ
	while(1)
	{
		if(PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			if(msg.message == WM_QUIT)
			{// PostQuitMessage()が呼ばれたらループ終了
				break;
			}
			else
			{
				// メッセージの翻訳と送出
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
        }
		else
		{
			dwCurrentTime = timeGetTime();

			if ((dwCurrentTime - dwFPSLastTime) >= 1000)	// 1秒ごとに実行
			{
#ifdef _DEBUG
				g_CountFPS = dwFrameCount;
#endif
				dwFPSLastTime = dwCurrentTime;				// FPSを測定した時刻を保存
				dwFrameCount = 0;							// カウントをクリア
			}

			if ((dwCurrentTime - dwExecLastTime) >= (1000 / 60))	// 1/60秒ごとに実行
			{
				dwExecLastTime = dwCurrentTime;	// 処理した時刻を保存

#ifdef _DEBUG	// デバッグ版の時だけFPSを表示する
				wsprintf(g_DebugStr, WINDOW_NAME);
				wsprintf(&g_DebugStr[strlen(g_DebugStr)], " FPS:%d", g_CountFPS);
#endif

				Update();			// 更新処理
				Draw();				// 描画処理

#ifdef _DEBUG	// デバッグ版の時だけ表示する
				wsprintf(&g_DebugStr[strlen(g_DebugStr)], " MX:%d MY:%d", GetMousePosX(), GetMousePosY());
				SetWindowText(hWnd, g_DebugStr);
#endif

				dwFrameCount++;
			}
		}
	}

	timeEndPeriod(1);				// 分解能を戻す

	// ウィンドウクラスの登録を解除
	UnregisterClass(CLASS_NAME, wcex.hInstance);

	// 終了処理
	Uninit();

	return (int)msg.wParam;
}

//=============================================================================
// プロシージャ
//=============================================================================
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch(message)
	{
	case WM_DESTROY:
		// ウィンドウの終了時に、ファイルのドロップ受付を解除する。
		DragAcceptFiles(hWnd, FALSE);

		PostQuitMessage(0);
		break;

	case WM_KEYDOWN:
		switch(wParam)
		{
		case VK_ESCAPE:
			DestroyWindow(hWnd);
			break;
		}
		break;

	case WM_MOUSEMOVE:
		g_MouseX = LOWORD(lParam);
		g_MouseY = HIWORD(lParam);
		break;
	case WM_DROPFILES:
	{
		// wParamにはファイルパスではなく、
		// ドロップ情報を参照するためのハンドルが格納されている。
		HDROP drop = reinterpret_cast<HDROP>(wParam);

		try
		{
			ReceiveDroppedObj(hWnd, drop);
		}
		catch (...)
		{
			// C++の例外をウィンドウプロシージャの外へ伝播させない。
			MessageBoxW(
				hWnd,
				L"ドロップされたファイルを処理できませんでした。",
				L"モデルの読み込み",
				MB_OK | MB_ICONERROR);
		}

		return 0;
	}
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}

	return 0;
}

//=============================================================================
// 初期化処理
//=============================================================================
HRESULT Init(HINSTANCE hInstance, HWND hWnd, BOOL bWindow)
{
	// レンダラーの初期化
	InitRenderer(hInstance, hWnd, bWindow);

	// カメラの初期化
	InitCamera();

	// 入力処理の初期化
	InitInput(hInstance, hWnd);

	// フィールドの初期化
	InitField();

	// プレイヤーの初期化
	InitPlayer();

	InitEnemy();


	// 法線による陰影を確認するため、平行光源を設定する。
	LIGHT light{};

	const XMVECTOR direction = XMVector3Normalize(XMVectorSet(0.5f, -1.0f, 0.7f, 0.0f));

	XMStoreFloat3(&light.Direction, direction);

	light.Type = LIGHT_TYPE_DIRECTIONAL;
	light.Enable = TRUE;

	// 面の向きによる明暗差を付ける。
	light.Diffuse = XMFLOAT4(0.85f, 0.85f, 0.85f, 1.0f);

	// 光が当たらない面も確認できるよう、弱い環境光を加える。
	light.Ambient = XMFLOAT4(0.15f, 0.15f, 0.15f, 1.0f);

	// ライトを有効化
	SetLight(0, &light);
	SetLightEnable(TRUE);

	// 背面ポリゴンをカリング
	SetCullingMode(CULL_MODE_BACK);

	return S_OK;
}

//=============================================================================
// 終了処理
//=============================================================================
void Uninit(void)
{
	// フィールドの終了処理
	UninitField();

	// プレイヤーの終了処理
	UninitPlayer();

	UninitEnemy();

	// カメラの終了処理
	UninitCamera();

	//入力の終了処理
	UninitInput();

	// レンダラーの終了処理
	UninitRenderer();
}

//=============================================================================
// 更新処理
//=============================================================================
void Update(void)
{
	// 入力の更新処理
	UpdateInput();

	// 押し続けている間ではなく、押した瞬間だけ切り替える。
	static bool previousF1Pressed = false;

	const bool f1Pressed = GetKeyboardPress(DIK_F1) != 0;

	if (f1Pressed && !previousF1Pressed)
	{
		if (GetFillMode() == FILL_MODE_SOLID)
		{
			SetFillMode(FILL_MODE_WIREFRAME);
		}
		else
		{
			SetFillMode(FILL_MODE_SOLID);
		}
	}

	previousF1Pressed = f1Pressed;

	// ドロップされたOBJファイルの読み込み要求があれば処理する。
	if (UpdatePendingObjLoad())
	{
		// 読み込み要求があった場合は、通常の更新処理を行わない。
		return;
	}

	// カメラ更新
	UpdateCamera();

	// フィールドの更新処理
	UpdateField();

	// プレイヤーの更新処理
	UpdatePlayer();

	UpdateEnemy();
}

//=============================================================================
// 描画処理
//=============================================================================
void Draw(void)
{
	// バックバッファクリア
	Clear();

	// プレイヤー視点
	XMFLOAT3 pos = GetPlayer()->pos;
	pos.y = 0.0f;			// カメラ酔いを防ぐためにクリアしている
	SetCameraAT(pos);
	SetCamera();

	// フィールドの描画処理
	//DrawField();

	// プレイヤーの描画処理
	DrawPlayer();

	DrawEnemy();

	// デバッグ表示
	DrawDebugProc();

	// バックバッファ、フロントバッファ入れ替え
	Present();
}

bool UpdatePendingObjLoad(void)
{
	// ファイルのドロップに使用した左ボタンの解放を待つ。
	static bool waitForMouseRelease = false;

	if (!g_PendingObjPath.empty())
	{
		std::wstring widePath;

		// 要求を取り出し、同じファイルを毎フレーム読み込まないようにする。
		widePath.swap(g_PendingObjPath);

		// 以前のドラッグ操作を終了し、ドロップを切断操作と区別する。
		CancelPlayerCutGesture();

		std::string path;

		if (!ConvertObjPath(widePath, path))
		{
			MessageBoxW(
				GetActiveWindow(),
				L"このパスは既存のローダーで使用できません。\n"
				L"英数字のフォルダーとファイル名で試してください。",
				L"モデルの読み込み",
				MB_OK | MB_ICONWARNING);
		}
		else if (!ReplacePlayerModel(path.c_str()))
		{
			MessageBoxW(
				GetActiveWindow(),
				L"モデルを読み込めませんでした。\n"
				L"現在のモデルは変更されていません。",
				L"モデルの読み込み",
				MB_OK | MB_ICONWARNING);
		}

		waitForMouseRelease = true;
		return true;
	}

	if (waitForMouseRelease)
	{
		if (!IsMouseLeftPressed())
		{
			waitForMouseRelease = false;
		}

		return true;
	}

	return false;
}


long GetMousePosX(void)
{
	return g_MouseX;
}


long GetMousePosY(void)
{
	return g_MouseY;
}


#ifdef _DEBUG
char* GetDebugStr(void)
{
	return g_DebugStr;
}
#endif
