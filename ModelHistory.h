#pragma once
#include "ModelSnapshot.h"
#include <memory>
#include <stack>

// モデルの取り消し履歴とやり直し履歴を管理する。
class ModelHistory
{
public:
    ModelHistory() = default;

    ModelHistory(const ModelHistory&) = delete;
    ModelHistory& operator=(const ModelHistory&) = delete;

    // 直前の状態に戻す。
    // 履歴がない場合は S_FALSE を返す。
    HRESULT Undo(DX11_MODEL* model);

    // 取り消した操作をやり直す。
    // 履歴がない場合は S_FALSE を返す。
    HRESULT Redo(DX11_MODEL* model);

    // 両方の履歴を破棄する。
    void Clear();

    // 切り取り前の状態を取り消し履歴へ一時的に保存する。
    HRESULT PrepareCut(DX11_MODEL* model);

    // 切り取りの結果に応じて、履歴を確定または取り消す。
    void FinishCut(bool succeeded);

private:
    using SnapshotStack = std::stack<std::unique_ptr<ModelSnapshot>>;

    // 復元元の履歴から状態を取り出し、
    // 現在の状態を反対側の履歴に保存する。
    HRESULT RestoreFromHistory(
        DX11_MODEL* model,
        SnapshotStack& sourceHistory,
        SnapshotStack& destinationHistory);

    SnapshotStack m_UndoHistory;
    SnapshotStack m_RedoHistory;

    // 切り取り前の履歴を一時保存していることを示す。
    bool m_CutPending = false;
};