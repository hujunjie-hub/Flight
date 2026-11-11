# S06-100..102 R24 feasibility analysis (NOT_RUN)

Status: **NOT_RUN**. No new differential test is implemented; R7 design was
withdrawn and is not reused.

## Inputs

- Product envelope: `fms_quad_x4_policy` / `fms_product_build_safety_envelope`,
  8 m/s, 4 m/s^2, jerk 16/16/8.
- Map: 16x16x16 cells, 0.5 m cell size, 8 m half extent.
- Float coordinates: binary32 relative resolution ~1.2e-7; at 8 m ~1e-6 m.
- Consumer-exact difference measured by S06-104: coefficient error bound
  `1.9823549122673993e-08 m` (coefficient-only); measured placement bounds are
  far below 0.5 m.

## Channel analysis

1. **Static collision**: minimum cell/obstacle placement resolution is 0.5 m.
   A consumer-vs-exact position difference of ~1e-8 m cannot move a point across
   a voxel boundary except when the point lies within ~1e-8 m of the boundary.
   Such a case is mathematically possible but not a stable, repeatable test:
   it requires constructing an exact boundary coincidence to <1e-7 m, then
   perturbing by the consumer error. The test would be dominated by float
   rounding and would not distinguish the consumer path generically.
2. **Dynamic collision**: same voxel/placement limit plus track prediction
   uncertainty; no stable consumer-vs-exact crossing case exists at 0.5 m
   resolution without placing the swept volume on a boundary.
3. **Geofence**: geofence uses a continuous half-space, so an exact-boundary
   case is possible; however, the validator's conservative inflation and
   fail-closed behavior intentionally absorb sub-ULP differences. A test that
   forces a pass/fail flip would test the inflation policy, not the consumer
   curve.

## Conclusion

No repeatable test case was established that (a) is inside the product envelope,
(b) distinguishes consumer from exact at the required resolution, and (c) is
not a knife-edge float coincidence. S06-100..102 remain **NOT_RUN** with the
reason: `MATHEMATICAL_FEASIBILITY_NOT_ESTABLISHED_AT_PRODUCT_RESOLUTION`.

Before implementation, an independent reviewer must approve either:
- a higher-resolution/analytic oracle that is not a finite sample, or
- a formally justified boundary-construction with exact rational inputs and a
  proven non-knife-edge margin.

No old R7 test may be revived without that approval. This document does not
claim collision safety or H723 validation.
