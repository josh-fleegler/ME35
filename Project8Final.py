import cv2
import numpy as np

# --- Load images ---
img1 = cv2.imread("/Users/joshuafleegler/Downloads/Apple1.png")
img2 = cv2.imread("/Users/joshuafleegler/Downloads/Apple2.jpg")

# Ensure images loaded
if img1 is None or img2 is None:
    raise ValueError("Could not load one or both image paths.")

# Convert from BGR (OpenCV default) to RGB for consistent color math
rgb_img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
rgb_img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)


def color_pixel_percentage(img, target_rgb=(255, 0, 0), tolerance=100, min_intensity=30):
    """
    Calculates what % of pixels are within a color tolerance of target_rgb.
    Returns both % and a mask (True for matched pixels).
    """
    img_float = img.astype(np.float32)
    target = np.array(target_rgb, dtype=np.float32)

    # Euclidean distance in RGB space
    dist = np.linalg.norm(img_float - target, axis=2)

    # Optional: brightness filter (ignore very dark pixels)
    brightness = img_float.mean(axis=2)
    mask = (dist < tolerance) & (brightness > min_intensity)

    percent = np.count_nonzero(mask) / mask.size * 100
    return percent, mask


# --- Settings ---
target_rgb = (255, 0, 0)  # red — change this
tolerance = 100           # increase if nothing detected
min_intensity = 30

# --- Compute ---
pct1, mask1 = color_pixel_percentage(rgb_img1, target_rgb, tolerance, min_intensity)
pct2, mask2 = color_pixel_percentage(rgb_img2, target_rgb, tolerance, min_intensity)

print(f"Target color: {target_rgb}")
print(f"Image 1: {pct1:.2f}% matching")
print(f"Image 2: {pct2:.2f}% matching")

# --- Visualization ---
def overlay_mask(img, mask, overlay_color=(0, 255, 0), alpha=0.5):
    """
    Overlay matching pixels semi-transparently.
    overlay_color is in RGB.
    """
    overlay = np.zeros_like(img, dtype=np.uint8)
    overlay[mask] = overlay_color
    blended = cv2.addWeighted(img, 1.0, overlay, alpha, 0)
    return blended

overlay1 = overlay_mask(rgb_img1, mask1, overlay_color=(255, 0, 0), alpha=0.5)
overlay2 = overlay_mask(rgb_img2, mask2, overlay_color=(255, 0, 0), alpha=0.5)

# --- Show ---
cv2.imshow("Original 1", cv2.cvtColor(rgb_img1, cv2.COLOR_RGB2BGR))
cv2.imshow("Original 2", cv2.cvtColor(rgb_img2, cv2.COLOR_RGB2BGR))
cv2.imshow("Overlay 1", cv2.cvtColor(overlay1, cv2.COLOR_RGB2BGR))
cv2.imshow("Overlay 2", cv2.cvtColor(overlay2, cv2.COLOR_RGB2BGR))

cv2.waitKey(0)
cv2.destroyAllWindows()

