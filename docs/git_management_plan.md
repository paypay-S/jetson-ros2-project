# Git LFSの導入と .gitignore の最適化

このプランでは、プロジェクト内の大容量ファイル（モデル、バイナリ）を Git LFS で管理するように変更し、同時に不要なデバッグ生成物やメタデータがリポジトリに混入しないよう `.gitignore` を最適化します。

## ユーザーへの確認事項

> [!IMPORTANT]
> **既存の履歴の扱いについて**
> 既にリポジトリに含まれている `.onnx` などの大容量ファイルを LFS に移行する場合、以下の2つの方法があります。どちらをご希望か教えてください。
> 1. **履歴を書き換えて完全に移行する:** リポジトリ全体のサイズが劇的に小さくなりますが、他のメンバーが再度クローンし直す必要があります。
> 2. **現在のバージョン以降のみ LFS で管理する:** 過去の履歴にはファイルが残るため、リポジトリ自体のサイズ（クローン時の時間）は変わりませんが、安全に移行できます。

> [!WARNING]
> システムに `git-lfs` がインストールされていないようです。本プランの実行前に以下のコマンドでのインストールが必要です（管理者権限が必要です）。
> ```bash
> sudo apt-get update && sudo apt-get install git-lfs
> ```

---

## 提案する変更内容

### 1. 共通設定 (Git LFS)
以下の拡張子を持つファイルを Git LFS の追跡対象にします。
- `.onnx`, `.engine`, `.pb` (モデルファイル)
- `.zip`, `.tar.gz` (圧縮アーカイブ)
- `.pth`, `.pt` (PyTorchモデル)

### 2. jetson-ros2-project

#### [MODIFY] [.gitignore](file:///home/yuta775/projects/jetson-ros2-project/.gitignore)
以下の不要な生成物を除外対象に追加します。
- `frames_*.gv`, `frames_*.pdf` (TFツリー可視化結果)
- `ros2_ws/install/`, `ros2_ws/build/`, `ros2_ws/log/` (既に記載済みですが再確認)

#### [NEW] [.gitattributes](file:///home/yuta775/projects/jetson-ros2-project/.gitattributes)
LFSの追跡設定を定義します。

---

### 3. f1tenth-rl-project

#### [MODIFY] [.gitignore](file:///home/yuta775/projects/f1tenth-rl-project/.gitignore)
以下の項目を追加します。
- `*.egg-info/` (Pythonのメタデータ)
- `gif/` (既に記載済みですが再確認)

#### [NEW] [.gitattributes](file:///home/yuta775/projects/f1tenth-rl-project/.gitattributes)
LFSの追跡設定を定義します。

---

## 実施手順

1. **git-lfs のインストールと初期化**
2. **.gitignore の更新**
3. **.gitattributes の作成と LFS 追跡の開始**
4. **既存ファイルの LFS への移動 (git lfs migrate または git add)**

## 検証計画

### 手動確認
- `git lfs status` で対象ファイルが LFS 管理下にあることを確認。
- `git ls-files` で不要なファイル（frames_*.pdf など）が管理対象から外れていることを確認。
