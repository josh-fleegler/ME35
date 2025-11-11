import cv2
import numpy as np

# --- Load images ---
img1 = cv2.imread("/Users/joshuafleegler/Downloads/Apple1.png")
img2 = cv2.imread("/Users/joshuafleegler/Downloads/Apple2.jpg")

# Convert from BGR (OpenCV default) to RGB
rgb_img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
rgb_img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)


def color_pixel_percentage(img, target_rgb=(255, 0, 0), tolerance=60, min_intensity=0):
    """
    Returns:
        percentage of pixels within color tolerance of target_rgb,
        boolean mask (same size as image, True for counted pixels)

    target_rgb: (R, G, B) tuple of the color to detect
    tolerance: maximum Euclidean distance in RGB space for a match
    min_intensity: optional minimum brightness cutoff (to ignore dark/black pixels)
    """
    # Convert to float for safe math
    img_float = img.astype(np.float32)

    # Compute Euclidean distance from target color
    diff = img_float - np.array(target_rgb, dtype=np.float32)
    dist = np.linalg.norm(diff, axis=2)  # per-pixel distance

    # Optional: skip very dark pixels to avoid noise
    brightness = img_float.mean(axis=2)
    mask = (dist < tolerance) & (brightness > min_intensity)

    color_pixels = np.count_nonzero(mask)
    total_pixels = img.shape[0] * img.shape[1]
    percentage = (color_pixels / total_pixels) * 100.0

    return percentage, mask


# --- User controls ---
target_rgb = (150, 200, 50)  # red — change to any RGB color
tolerance = 80           # how close a pixel must be (lower = stricter)
min_intensity = 40        # ignore dark shadows, optional

# --- Compute for both images ---
pct1, mask1 = color_pixel_percentage(rgb_img1, target_rgb, tolerance, min_intensity)
pct2, mask2 = color_pixel_percentage(rgb_img2, target_rgb, tolerance, min_intensity)

print(f"Target RGB: {target_rgb}")
print(f"Image 1: {pct1:.2f}% matching pixels")
print(f"Image 2: {pct2:.2f}% matching pixels")

if pct1 > pct2:
    print("→ Image 1 has more pixels near the target color.")
elif pct2 > pct1:
    print("→ Image 2 has more pixels near the target color.")
else:
    print("→ Both images have about the same percentage of that color.")


# --- Visualization: highlight only matching pixels ---
highlight1 = np.zeros_like(rgb_img1)
highlight1[mask1] = rgb_img1[mask1]

highlight2 = np.zeros_like(rgb_img2)
highlight2[mask2] = rgb_img2[mask2]

# --- Show results ---
cv2.imshow("Original 1", cv2.cvtColor(rgb_img1, cv2.COLOR_RGB2BGR))
cv2.imshow("Original 2", cv2.cvtColor(rgb_img2, cv2.COLOR_RGB2BGR))
cv2.imshow("Matched Color 1", cv2.cvtColor(highlight1, cv2.COLOR_RGB2BGR))
cv2.imshow("Matched Color 2", cv2.cvtColor(highlight2, cv2.COLOR_RGB2BGR))

cv2.waitKey(0)
cv2.destroyAllWindows()

