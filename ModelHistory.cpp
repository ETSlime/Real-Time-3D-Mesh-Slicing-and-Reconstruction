#include "ModelHistory.h"

HRESULT ModelHistory::Undo(DX11_MODEL* model)
{
    // 取り消し履歴から復元し、現在の状態をやり直し履歴へ保存する。
    return RestoreFromHistory(
        model,
        m_UndoHistory,
        m_RedoHistory);
}

HRESULT ModelHistory::Redo(DX11_MODEL* model)
{
    // やり直し履歴から復元し、現在の状態を取り消し履歴へ保存する。
    return RestoreFromHistory(
        model,
        m_RedoHistory,
        m_UndoHistory);
}

HRESULT ModelHistory::RestoreFromHistory(
    DX11_MODEL* model,
    SnapshotStack& sourceHistory,
    SnapshotStack& destinationHistory)
{
    // 切り取り処理の途中では履歴を復元しない。
    if (m_CutPending)
        return E_UNEXPECTED;

    // 操作対象のモデルを確認する。
    if (model == nullptr || model->modelData == nullptr)
        return E_INVALIDARG;

    // 復元する履歴がなければ、何も変更しない。
    if (sourceHistory.empty())
        return S_FALSE;

    try
    {
        // 復元前の状態を、反対側の履歴に保存するために取得する。
        std::unique_ptr<ModelSnapshot> currentSnapshot =
            ModelSnapshot::Capture(model);

        if (currentSnapshot == nullptr)
            return E_FAIL;

        // スタックへの追加にはメモリ確保が発生する可能性があるため、
        // モデルを変更する前に追加を完了させる。
        destinationHistory.push(std::move(currentSnapshot));
    }
    catch (const std::bad_alloc&)
    {
        // 保存に失敗した場合、モデルと両方の履歴は変更されない。
        return E_OUTOFMEMORY;
    }

    // 復元元のスナップショットは、成功するまでスタックに残しておく。
    const HRESULT result = sourceHistory.top()->Restore(model);

    if (FAILED(result))
    {
        // 復元できなかったため、直前に追加した履歴を取り消す。
        destinationHistory.pop();

        return result;
    }

    // 復元が完了したため、使用したスナップショットを破棄する。
    sourceHistory.pop();

    return S_OK;
}

void ModelHistory::Clear()
{
    // スナップショットの破棄に伴い、テクスチャ参照も解放される。
    while (!m_UndoHistory.empty())
    {
        m_UndoHistory.pop();
    }

    while (!m_RedoHistory.empty())
    {
        m_RedoHistory.pop();
    }

    m_CutPending = false;
}

HRESULT ModelHistory::PrepareCut(DX11_MODEL* model)
{
    // 前回の処理が完了していない場合は開始しない。
    if (m_CutPending)
        return E_UNEXPECTED;

    try
    {
        // モデルを変更する前の状態を保存する。
        std::unique_ptr<ModelSnapshot> snapshot = ModelSnapshot::Capture(model);

        if (snapshot == nullptr)
            return E_FAIL;

        // 入れ物のメモリ確保も、モデルを変更する前に完了させる。
        m_UndoHistory.push(std::move(snapshot));

        m_CutPending = true;

        return S_OK;
    }
    catch (const std::bad_alloc&)
    {
        return E_OUTOFMEMORY;
    }
}

void ModelHistory::FinishCut(bool succeeded)
{
    if (!m_CutPending)
        return;

    if (succeeded)
    {
        // 新しい編集が確定したため、以前のやり直し履歴を破棄する。
        while (!m_RedoHistory.empty())
        {
            m_RedoHistory.pop();
        }
    }
    else
    {
        // モデルを変更できなかったため、一時保存した履歴を取り消す。
        m_UndoHistory.pop();
    }

    m_CutPending = false;
}