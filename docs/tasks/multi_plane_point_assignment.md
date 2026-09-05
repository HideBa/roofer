# Task: Multi-Plane Point Assignment for Ridge/Intersection Points

## Problem

The current plane detection (region growing) assigns each point to **exactly one plane** based on first-match. Points lying on **ridges or plane intersections** are assigned only to the first detected plane, causing:

1. **Missing points** from adjacent intersecting planes
2. **Inaccurate intersection line extraction** (LineDetector, PlaneIntersector)
3. **Reduced accuracy in graph-cut optimization** (data term uses height map from incomplete plane points)

### Visual Example

```
Current Behavior:              Desired Behavior:
     Plane A                        Plane A
    ╱ ● ● ●                        ╱ ● ● ●
   ╱  ● ● ●                       ╱  ● ● ●
  ╱   ● ● ●                      ╱   ● ● ◆ ← Point belongs to BOTH
 ╱────●────╲  ← Ridge points    ╱────◆────╲    Plane A and Plane B
      ● ● ●  ╲                      ◆ ● ●  ╲
      ● ● ●   ╲                     ● ● ●   ╲
      ● ● ●    ╲ Plane B            ● ● ●    ╲ Plane B

Ridge points assigned to       Ridge points assigned to
Plane A only (first detected)  BOTH planes
```

## Task

Modify the plane detection to handle points near plane intersections by either:

### Option A: Multi-Label Assignment

Assign multiple plane IDs to points near intersections.

- Change `plane_id` from `vec1i` to `std::vector<std::vector<int>>` or similar
- Update downstream components to handle multi-labels

### Option B: Point Duplication (Recommended)

Duplicate points that lie on intersection lines, adding them to all relevant planes.

- Keep `plane_id` as `vec1i` (simpler downstream)
- Duplicate points in `pts_per_roofplane` for each plane they belong to
- Minimal changes to downstream pipeline

## Key Files

| File                                                  | Purpose                                              |
| ----------------------------------------------------- | ---------------------------------------------------- |
| `src/core/reconstruction/PlaneDetector.cpp`           | Main plane detection implementation                  |
| `include/roofer/reconstruction/PlaneDetector.hpp`     | Interface definition                                 |
| `include/roofer/reconstruction/PlaneDetectorBase.hpp` | `DistAndNormalTester` - point-to-plane distance test |
| `include/roofer/reconstruction/RegionGrower.hpp`      | Generic region growing algorithm                     |

## Implementation Steps

1. **Identify intersection points**: After initial plane detection, for each point:

   - Check distance to **all detected planes** (not just assigned plane)
   - If distance < `metrics_plane_epsilon` for multiple planes → mark as intersection point

2. **Handle intersection points** (Option B):

   - For each intersection point, add it to `pts_per_roofplane` for **all** planes within threshold
   - Keep original `plane_id` assignment (for visualization compatibility)

3. **Add configuration parameter**:
   - `intersection_point_threshold`: Max distance to consider a point as belonging to a plane intersection (suggested default: same as `metrics_plane_epsilon`)

## Acceptance Criteria

- [x] Points near ridges appear in `pts_per_roofplane` for **all** adjacent planes
- [x] AlphaShaper generates boundaries that extend to the ridge line for **both** planes
- [x] PlaneIntersector computes more accurate intersection segments
- [ ] Existing unit tests pass
- [ ] Optional: Visualize intersection points differently in Rerun (e.g., different color)

## Notes

- Focus on **minimal changes** to preserve existing pipeline behavior
- The `pts_per_roofplane` output is used by: AlphaShaper, PlaneIntersector, LineDetector
- Consider computational cost for buildings with many planes

---

## Implementation Summary

Implemented **Option B: Point Duplication** with minimal changes to the existing pipeline.

### Changes Made

1. **`include/roofer/reconstruction/PlaneDetector.hpp`**:

   - Added `intersection_epsilon` parameter to `PlaneDetectorConfig` (default `-1.0f` = use `metrics_plane_epsilon`)
   - Added `intersection_point_additions` counter to `PlaneDetectorInterface` for monitoring

2. **`src/core/reconstruction/PlaneDetector.cpp`**:

   - Added post-processing step after plane detection that:
     - Iterates through all segmented points
     - Checks each point's distance to all other detected planes
     - If distance < threshold, adds the point to that plane's `pts_per_roofplane` entry
     - Tracks count of additions via `intersection_point_additions`

3. **`apps/roofer-app/config.hpp`**:

   - Added `intersection_epsilon` to `RooferConfig`
   - Added CLI parameter `--intersection-epsilon` with description

4. **`apps/roofer-app/reconstruct_building.hpp`**:
   - Passes `intersection_epsilon` to `PlaneDetectorConfig`

### Usage

```bash
# Use default (same as plane_detect_epsilon = 0.3m)
roofer ...

# Use custom threshold
roofer --intersection-epsilon 0.5 ...

# Disable multi-plane assignment
roofer --intersection-epsilon 0 ...
```

### Algorithm Complexity

- O(N × P) where N = number of points, P = number of planes
- For typical buildings (N < 10000, P < 50), this is negligible overhead
