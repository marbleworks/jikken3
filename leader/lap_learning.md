# 学習ラップ方式の概要

このファイルでは、`leader/leader.ino` に実装されている学習ラップ（Learning Lap）と本気ラップ（Racing Lap）の流れ、および主要パラメータについてまとめます。

## 疑似距離カウンタ
- 5ms周期の制御ループごとに `setWheels` に渡された左右PWM値の平均を積算し、`pseudoDistance` として保持します。
- 周回の先頭で `pseudoDistance` を 0 に戻すことで、ラップ中の「地点」を PWM積分値で表現できます。
- 前進時のみ積算するため、急制動やバックの影響を受けません。

## 十字線（スタートライン）検出
1. **緩いしきい値**：`THRESHOLD - CROSS_THRESHOLD_OFFSET` を用いて、細いクロスラインでも黒判定しやすくしています。
2. **時間窓**：`CROSS_WINDOW_MS` の間に全センサが一度でも黒判定になれば「十字線1本通過」とみなします。
3. **クールタイム**：`CROSS_COOLDOWN_MS` の間は再検出を無効化し、ノイズを抑えます。
4. **ペア検出**：1本目と2本目の十字線が `CROSS_PAIR_TIMEOUT_MS` 以内に連続した場合にスタートラインと認識し、ラップ境界として `pseudoDistance` をリセットします。
5. **偏差制限**：走行偏差が `CROSS_MAX_ERROR` を超えている場合は十字線判定を無効化し、急カーブ中の誤検出を防ぎます。

## 学習ラップ（1周目）
- `learningLapActive` が真の間は従来の PID + 誤差減速で安全に周回しつつ、急カーブ候補を探索します。
- 前方誤差の絶対値が `LOOP_CURVE_ENTER_ERROR` を超えると区間記録を開始し、`LOOP_DISTANCE_MARGIN_BEFORE` だけ手前をブレーキ開始距離に含めます。
- 偏差が `LOOP_CURVE_EXIT_ERROR` を下回ったら区間を閉じ、`LOOP_DISTANCE_MARGIN_AFTER` を出口側へ加算します。
- 近い距離の区間は `LOOP_DISTANCE_MERGE_GAP` 以下であればマージし、最大 `LOOP_MAX_BRAKE_ZONES` 件まで保存します。

## 本気ラップ（2周目以降）
- ラップ開始時に `racingLapReady` が真であれば、記録済み区間を参照して先読み減速を有効にします。
- `pseudoDistance` がブレーキ区間内にあるときは基準PWMを `LOOP_CURVE_PWM_LIMIT` 以下へクリップし、それ以外は `BASE_FWD` の値を許可します。
- センサ偏差に基づく緊急減速ロジックは従来通り残しているため、予測が外れても安全に減速できます。

## 調整の目安
- **クロスラインしきい値**：センサ感度やコース塗装によって `CROSS_THRESHOLD_OFFSET` を調整してください。
- **減速マージン**：コース幅や速度によって `LOOP_DISTANCE_MARGIN_BEFORE/AFTER` を広げたり狭めたりします。
- **速度制限**：`LOOP_CURVE_PWM_LIMIT` を上げると立ち上がりが速くなりますが、曲率によってはオーバーシュートする可能性があります。

以上を踏まえ、学習ラップで得た知識を本気ラップに活用し、コース全体で安定かつ高速な走行を実現します。
