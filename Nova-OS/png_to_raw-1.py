from PIL import Image
import struct
import os

# ── Config ─────────────────────────────────
TARGET_W      = 320
TARGET_H      = 240
INPUT_FOLDER  = "screens_png"   # put all PNGs here
OUTPUT_FOLDER = "screens_raw"   # .raw files appear here
# ───────────────────────────────────────────

def png_to_raw(input_path, output_path):
    img = Image.open(input_path).convert("RGB")

    # Resize if not already 320x240
    if img.size != (TARGET_W, TARGET_H):
        print(f"  Resizing {img.size} → {TARGET_W}x{TARGET_H}")
        img = img.resize((TARGET_W, TARGET_H), Image.LANCZOS)

    with open(output_path, 'wb') as out:
        # Header: width(2) + height(2)
        out.write(struct.pack('>HH', TARGET_W, TARGET_H))

        # Pixels row by row
        for y in range(TARGET_H):
            for x in range(TARGET_W):
                r, g, b = img.getpixel((x, y))
                c = ((r & 0xF8) << 8) | \
                    ((g & 0xFC) << 3) | \
                    (b >> 3)
                out.write(struct.pack('>H', c))

    size_kb = os.path.getsize(output_path) / 1024
    print(f"  ✅ {os.path.basename(output_path)}  ({size_kb:.1f} KB)")

def convert_all():
    # Create output folder
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)

    # Find all image files
    files = [
        f for f in os.listdir(INPUT_FOLDER)
        if f.lower().endswith('.png')  or
           f.lower().endswith('.jpg')  or
           f.lower().endswith('.jpeg')
    ]

    if not files:
        print(f"❌ No images found in '{INPUT_FOLDER}' folder!")
        return

    print(f"Found {len(files)} images")
    print(f"Target: {TARGET_W}x{TARGET_H} landscape")
    print(f"Input:  {INPUT_FOLDER}/")
    print(f"Output: {OUTPUT_FOLDER}/")
    print()

    success = 0
    failed  = 0

    for fname in sorted(files):
        # Keep same filename, just change extension to .raw
        name     = os.path.splitext(fname)[0]
        in_path  = os.path.join(INPUT_FOLDER,  fname)
        out_path = os.path.join(OUTPUT_FOLDER, name + ".raw")

        print(f"Converting: {fname}")
        try:
            png_to_raw(in_path, out_path)
            success += 1
        except Exception as e:
            print(f"  ❌ Failed: {e}")
            failed += 1

    print()
    print(f"✅ Success: {success}")
    if failed > 0:
        print(f"❌ Failed:  {failed}")
    print(f"Done! Copy '{OUTPUT_FOLDER}/' to SD card as '/screens/'")

convert_all()
