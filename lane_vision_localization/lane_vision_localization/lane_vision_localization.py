import cv2
import numpy as np
import math

# ----------------------------------------------------
# 1. 設定定数
# ----------------------------------------------------
INPUT_IMAGE_PATH = '0_0_00269.jpg'

# 直線選別の条件
TARGET_POINT = (111, 140)  # 元画像内での基準点 (x, y)
ROI_Y_LIMIT = 140          # Y座標がこれ以下の領域のみ使用（画像上部）

# 俯瞰画像内でのロボット/カメラの基準位置（ピクセル）
# (495, 968) を原点 (0, 0) とし、前方をx+, 左方をy+ とする
CAMERA_PIXEL_X = 495  
CAMERA_PIXEL_Y = 968  
MM_PER_PIXEL = 10.0        # 1ピクセルあたりの実寸法 (mm)

# 射影変換用パラメータ
# pts1: 元画像上の4点, pts2: 俯瞰画像上での対応する4点
pts1 = np.array([(172,40), (137,16), (30,37), (66,15)], dtype=np.float32)
offset_x = 400
offset_y = 600
scale_factor = 1.0
raw_pts2 = np.array([(176,180), (176,0), (0,180), (0,0)], dtype=np.float32)
pts2 = (raw_pts2 * scale_factor) + np.float32([offset_x, offset_y])

# 射影行列 M の作成
M = cv2.getPerspectiveTransform(pts1, pts2)

# 出力画像のキャンバスサイズ
OUTPUT_W = int(offset_x * 2 + 200)
OUTPUT_H = int(offset_y + 400)

# ----------------------------------------------------
# 2. 補助関数
# ----------------------------------------------------

def calculate_distance(p1, p2, target):
    """ 線分の中点とターゲット点の距離を計算 """
    mid_x = (p1[0] + p2[0]) / 2
    mid_y = (p1[1] + p2[1]) / 2
    return math.sqrt((mid_x - target[0])**2 + (mid_y - target[1])**2)

def transform_to_robot_coords(u, v):
    """
    俯瞰ピクセル座標 (u, v) をロボット座標 (x, y) [メートル] に変換
    x: 前方 (+), y: 左方 (+)
    """
    # 1. カメラ基準のピクセル差分 (ROSの座標系に合わせる)
    delta_u = CAMERA_PIXEL_X - u  # uが小さい（左）ほどプラス
    delta_v = CAMERA_PIXEL_Y - v  # vが小さい（前）ほどプラス
    
    # 2. ミリメートル換算してからメートルに変換
    x_m = (delta_v * MM_PER_PIXEL) / 1000.0
    y_m = (delta_u * MM_PER_PIXEL) / 1000.0
    
    return x_m, y_m

# ----------------------------------------------------
# 3. メイン処理
# ----------------------------------------------------

def process():
    # 画像読み込み
    img = cv2.imread(INPUT_IMAGE_PATH)
    if img is None:
        print(f"Error: Could not load image {INPUT_IMAGE_PATH}")
        return

    # --- ステップ1: 前処理とマスク処理 ---
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blur, 50, 150)

    # ROIマスク作成 (Y座標140以下)
    mask = np.zeros_like(edges)
    mask[0:ROI_Y_LIMIT, :] = 255 
    masked_edges = cv2.bitwise_and(edges, mask)

    # --- ステップ2: 直線検出 ---
    # ハフ変換で直線を検出
    lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, 
                            threshold=30, minLineLength=30, maxLineGap=10)

    # --- ステップ3: 最も近い直線の選別 ---
    target_line = None
    if lines is not None:
        min_dist = float('inf')
        for line in lines:
            l_p1 = (line[0][0], line[0][1])
            l_p2 = (line[0][2], line[0][3])
            dist = calculate_distance(l_p1, l_p2, TARGET_POINT)
            if dist < min_dist:
                min_dist = dist
                target_line = line[0]

    # --- ステップ4: 俯瞰変換とロボット座標計算 ---
    # 結果表示用のキャンバス作成
    bev_img = cv2.warpPerspective(img, M, (OUTPUT_W, OUTPUT_H))
    debug_img = img.copy()

    if target_line is not None:
        lx1, ly1, lx2, ly2 = target_line
        
        # A. 元画像座標 -> 俯瞰ピクセル座標
        points_src = np.array([[[lx1, ly1]], [[lx2, ly2]]], dtype=np.float32)
        points_dst = cv2.perspectiveTransform(points_src, M)
        u1, v1 = points_dst[0][0]
        u2, v2 = points_dst[1][0]

        # B. 俯瞰ピクセル座標 -> ロボットメートル座標
        rx1, ry1 = transform_to_robot_coords(u1, v1)
        rx2, ry2 = transform_to_robot_coords(u2, v2)

        # 結果表示（コンソール）
        print(f"--- 検出結果 ---")
        print(f"選定直線(元画像): ({lx1}, {ly1}) -> ({lx2}, {ly2})")
        print(f"ロボット座標(始点): x={rx1:.3f}m, y={ry1:.3f}m")
        print(f"ロボット座標(終点): x={rx2:.3f}m, y={ry2:.3f}m")
        
        # 角度と中心点の計算
        yaw_deg = math.degrees(math.atan2(ry2 - ry1, rx2 - rx1))
        print(f"直線の角度(Yaw): {yaw_deg:.2f} deg")

        # --- ステップ5: 描画と保存 ---
        # 俯瞰画像に緑の線を描画
        cv2.line(bev_img, (int(u1), int(v1)), (int(u2), int(v2)), (0, 255, 0), 3)
        # カメラ位置（原点）を赤丸で表示
        cv2.circle(bev_img, (CAMERA_PIXEL_X, CAMERA_PIXEL_Y), 8, (0, 0, 255), -1)

        # デバッグ用（元画像に選別結果を表示）
        cv2.circle(debug_img, TARGET_POINT, 5, (0, 255, 255), -1)
        cv2.line(debug_img, (lx1, ly1), (lx2, ly2), (0, 255, 0), 2)
    else:
        print("直線が見つかりませんでした。")

    # ファイル保存
    cv2.imwrite('debug_original_select.jpg', debug_img)
    cv2.imwrite('output_bev_final.jpg', bev_img)
    print("\n画像を保存しました: debug_original_select.jpg, output_bev_final.jpg")

if __name__ == '__main__':
    process()