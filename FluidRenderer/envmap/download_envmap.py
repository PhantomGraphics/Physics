"""
SSFR 環境マップ生成スクリプト
Poly Haven から HDR 画像をダウンロードしてキューブマップ 6 面に展開する。
ダウンロード失敗時は Pillow だけで生成するグラデーション空にフォールバックする。

使い方:
    python download_envmap.py
"""

import os
import struct
import urllib.request
import urllib.error

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
FACE_NAMES = ["right.png", "left.png", "top.png", "bottom.png", "front.png", "back.png"]
FACE_SIZE = 512  # キューブフェースの解像度

HDR_URL = "https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/1k/kloofendal_48d_partly_cloudy_1k.hdr"
HDR_PATH = os.path.join(OUTPUT_DIR, "_tmp_env.hdr")


# ---------------------------------------------------------------------------
# HDR ローダー (Radiance RGBE 形式)
# ---------------------------------------------------------------------------

def _load_hdr(path):
    """Radiance .hdr ファイルを float RGB numpy 配列として読み込む。"""
    import numpy as np

    with open(path, "rb") as f:
        # ヘッダー解析
        while True:
            line = f.readline().decode("ascii", errors="replace").strip()
            if line == "":
                break  # 空行でヘッダー終了
        size_line = f.readline().decode("ascii").strip()  # -Y H +X W
        parts = size_line.split()
        height = int(parts[1])
        width  = int(parts[3])

        img = np.zeros((height, width, 3), dtype=np.float32)

        for y in range(height):
            # 新形式 RLE: 最初の 4 バイトを確認
            scanline_width_bytes = f.read(4)
            if len(scanline_width_bytes) < 4:
                break
            r, g, b, e = scanline_width_bytes
            if r == 2 and g == 2 and (b << 8 | e) == width:
                # 新形式 RLE
                channels = []
                for _ in range(4):
                    channel = []
                    while len(channel) < width:
                        code = ord(f.read(1))
                        if code > 128:
                            count = code - 128
                            val   = ord(f.read(1))
                            channel.extend([val] * count)
                        else:
                            channel.extend(f.read(code))
                    channels.append(channel[:width])
                rgbe = np.array(channels, dtype=np.uint8).T  # (width, 4)
            else:
                # 古い形式 (非 RLE) — 4 バイトを最初のピクセルとして扱う
                rest = f.read((width - 1) * 4)
                row_bytes = bytes([r, g, b, e]) + rest
                rgbe = np.frombuffer(row_bytes, dtype=np.uint8).reshape(width, 4)

            # RGBE → float 変換
            e_arr = rgbe[:, 3].astype(np.float32)
            scale = np.where(e_arr > 0, np.exp2(e_arr - 128 - 8), 0.0).astype(np.float32)
            img[y, :, 0] = rgbe[:, 0] * scale
            img[y, :, 1] = rgbe[:, 1] * scale
            img[y, :, 2] = rgbe[:, 2] * scale

    return img


# ---------------------------------------------------------------------------
# 等天頂 → キューブマップ変換
# ---------------------------------------------------------------------------

def _equirect_to_cube(equirect, face_size):
    """等天頂画像 (H×W×3 float) を 6 面 (FACE_SIZE×FACE_SIZE×3 float) のリストに変換する。"""
    import numpy as np

    H, W, _ = equirect.shape
    faces = []

    # Vulkan 規約: right(+X), left(-X), top(+Y), bottom(-Y), front(+Z), back(-Z)
    directions = [
        (+1,  0,  0),  # right  +X
        (-1,  0,  0),  # left   -X
        ( 0, +1,  0),  # top    +Y
        ( 0, -1,  0),  # bottom -Y
        ( 0,  0, +1),  # front  +Z
        ( 0,  0, -1),  # back   -Z
    ]
    # 各フェースの U/V ベクトル定義
    uv_axes = [
        # right +X:  U=+Z, V=-Y
        (( 0,  0,  1), ( 0, -1,  0)),
        # left  -X:  U=-Z, V=-Y
        (( 0,  0, -1), ( 0, -1,  0)),
        # top   +Y:  U=+X, V=+Z
        (( 1,  0,  0), ( 0,  0,  1)),
        # bottom -Y: U=+X, V=-Z
        (( 1,  0,  0), ( 0,  0, -1)),
        # front +Z:  U=-X, V=-Y
        ((-1,  0,  0), ( 0, -1,  0)),
        # back  -Z:  U=+X, V=-Y
        (( 1,  0,  0), ( 0, -1,  0)),
    ]

    xs = np.linspace(-1.0, 1.0, face_size)
    ys = np.linspace(-1.0, 1.0, face_size)
    gx, gy = np.meshgrid(xs, ys)  # (face_size, face_size)

    for i, (fwd, (u_ax, v_ax)) in enumerate(zip(directions, uv_axes)):
        fx, fy, fz = fwd
        ux, uy, uz = u_ax
        vx, vy, vz = v_ax

        dx = fx + gx * ux + gy * vx
        dy = fy + gx * uy + gy * vy
        dz = fz + gx * uz + gy * vz

        length = np.sqrt(dx*dx + dy*dy + dz*dz)
        dx /= length
        dy /= length
        dz /= length

        lon = np.arctan2(dx, dz)              # [-π, π]
        lat = np.arcsin(np.clip(dy, -1, 1))  # [-π/2, π/2]

        u_idx = ((lon / (2 * 3.14159265) + 0.5) * W).astype(int) % W
        v_idx = ((0.5 - lat / 3.14159265) * H).astype(int)
        v_idx = np.clip(v_idx, 0, H - 1)

        face = equirect[v_idx, u_idx, :]
        faces.append(face)

    return faces


def _float_to_uint8(face):
    """float RGB → uint8 RGBA (gamma 2.2 tonemap)"""
    import numpy as np
    # Reinhard トーンマッピング + gamma 2.2
    rgb = face / (face + 1.0)
    rgb = np.power(np.clip(rgb, 0, 1), 1.0 / 2.2)
    alpha = np.ones((*rgb.shape[:2], 1), dtype=np.float32)
    rgba = np.concatenate([rgb, alpha], axis=2)
    return (rgba * 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# フォールバック: グラデーション空を生成
# ---------------------------------------------------------------------------

def _generate_gradient_sky(face_size):
    """Pillow だけで生成するシンプルなグラデーション空。"""
    from PIL import Image
    import numpy as np

    faces = []
    for i in range(6):
        arr = np.zeros((face_size, face_size, 4), dtype=np.uint8)
        for y in range(face_size):
            t = y / float(face_size - 1)
            if i == 2:   # top (+Y): 空の青
                r, g, b = int(100 * (1 - t) + 30 * t), int(149 * (1 - t) + 80 * t), int(237 * (1 - t) + 180 * t)
            elif i == 3: # bottom (-Y): 地面色
                r, g, b = 80, 70, 60
            else:        # 側面: 水平線グラデーション
                r = int(135 * (1 - t) + 200 * t)
                g = int(206 * (1 - t) + 220 * t)
                b = int(235 * (1 - t) + 240 * t)
            arr[y, :] = (r, g, b, 255)
        faces.append(Image.fromarray(arr, mode="RGBA"))

    return faces


# ---------------------------------------------------------------------------
# メイン処理
# ---------------------------------------------------------------------------

def main():
    from PIL import Image

    faces = None

    # --- Poly Haven からダウンロード試行 ---
    try:
        print(f"[envmap] Downloading HDR from Poly Haven...")
        urllib.request.urlretrieve(HDR_URL, HDR_PATH)
        print(f"[envmap] Download complete: {HDR_PATH}")
        try:
            import numpy as np
            hdr = _load_hdr(HDR_PATH)
            cube_faces = _equirect_to_cube(hdr, FACE_SIZE)
            faces = [Image.fromarray(_float_to_uint8(f), mode="RGBA") for f in cube_faces]
            print("[envmap] HDR converted to cubemap faces.")
        except Exception as e:
            print(f"[envmap] HDR convert failed: {e}, using gradient fallback.")
    except urllib.error.URLError as e:
        print(f"[envmap] Download failed ({e}), using gradient fallback.")
    finally:
        if os.path.exists(HDR_PATH):
            os.remove(HDR_PATH)

    # --- フォールバック ---
    if faces is None:
        print("[envmap] Generating gradient sky fallback...")
        faces = _generate_gradient_sky(FACE_SIZE)

    # --- 保存 ---
    for img, name in zip(faces, FACE_NAMES):
        out = os.path.join(OUTPUT_DIR, name)
        img.save(out)
        print(f"[envmap] Saved {out}")

    print("[envmap] Done.")


if __name__ == "__main__":
    main()
