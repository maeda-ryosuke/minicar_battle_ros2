import numpy as np
import matplotlib.pyplot as plt
import cv2

# 基準とする畳四隅の写真上の座標（単位px）
pts1 = np.array([(172,40), (137,16), (30,37), (66,15)], dtype=np.float32)

# ---------------------------------------------------------
# 【変更点1】 オフセット（平行移動量）とスケールを定義する
# ---------------------------------------------------------
# これがないと、(0,0)より左上にある領域が全部切れてしまいます
offset_x = 400  # 画像を右にずらすピクセル数（画像の中心に持ってくるために大きめに）
offset_y = 600  # 画像を下にずらすピクセル数

# 必要ならスケールもかける（1cm = 1pxだと画像が小さい場合）
scale = 1.0     # 現在は 1cm = 1px

# 基準とする畳四隅の実際の座標（単位cm）にオフセットを加算
# 元の座標: [(176,180), (176,0), (0,180), (0,0)]
# これを「画面の中央付近」に来るようにずらします
raw_pts2 = np.array([(176,180), (176,0), (0,180), (0,0)], dtype=np.float32)
pts2 = (raw_pts2 * scale) + np.float32([offset_x, offset_y])

# ---------------------------------------------------------

# 射影行列の取得
M = cv2.getPerspectiveTransform(pts1, pts2) 

img1 = cv2.imread('output_05_lines_detected.jpg', cv2.IMREAD_COLOR)
# 注意: Matplotlibで表示しないならBGR->RGB変換は不要ですが、一応残します
img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)

# 【変更点2】 出力画像のサイズを十分に大きく確保する
# オフセット分を含めてサイズを決定します
output_w = int(offset_x * 2 + 200) # 左右に余裕を持たせる
output_h = int(offset_y + 400)     # 下方向にも余裕を持たせる

# 元画像を射影変換し鳥瞰画像へ
img2 = cv2.warpPerspective(img1, M, (output_w, output_h))

# 保存用にBGRに戻す（cv2.imwriteはBGR前提のため）
img2_bgr = cv2.cvtColor(img2, cv2.COLOR_RGB2BGR)
cv2.imwrite('output_syaei.jpg', img2_bgr)