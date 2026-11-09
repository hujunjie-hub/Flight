/*
 * Flight public contract - version and compiler/linkage primitives.
 *
 * Authority: spec/02_接口契约与公共数据规范.md (V1.9.1-GLOBAL-AUDIT-R3),
 *            section 1.4 "ContractHeader、时间和坐标" and section 2.15.
 *
 * This header is the only place where the public schema/Port revisions are
 * defined.  The product contract revision is owned by the selected product
 * contract header because each product build has its own ABI manifest.
 *
 * Schema 6.0 is an incompatible boundary: Major 2/3/4/5 layouts must never be
 * mixed with these headers.  All public structures use fixed-width integer
 * types plus IEEE-754 float32/double64 only, natural alignment (max 8), no
 * packing, and no enum member storage.
 */
#ifndef CONTRACT_VERSION_H
#define CONTRACT_VERSION_H

#define CONTRACT_SCHEMA_MAJOR 6u
#define CONTRACT_SCHEMA_MINOR 0u

/* Public Port API revision (QUAD_X4).  CR-V191-M45-01 changed the product
 * payload layout; Schema 6.0 stays because the common envelope/messages did
 * not change.  Consumers must recompile and match this revision. */
#define PORT_API_REVISION 0x00010906u

/* Canonical ABI model used by the ABI lock/report tooling.  Bumping this
 * value requires regenerating the selected product ABI lock in middleware/. */
#define CONTRACT_ABI_MODEL 1u

/* Documented ContractHeader size (spec 00 section 4.1).  Kept as a macro so
 * ABI checks and build manifests can assert it without duplicating a number. */
#define CONTRACT_HEADER_BYTES 88u

#if defined(__cplusplus)
#  if __cplusplus < 201103L
#    error "Flight public contracts require C++11 or newer"
#  endif
#  define EXTERN_C_BEGIN extern "C" {
#  define EXTERN_C_END }
#  define CONTRACT_STATIC_ASSERT(cond, msg) static_assert((cond), msg)
#  define CONTRACT_ALIGNOF(type) alignof(type)
#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#  define EXTERN_C_BEGIN
#  define EXTERN_C_END
#  define CONTRACT_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)
#  define CONTRACT_ALIGNOF(type) _Alignof(type)
#else
/* The existing Flight firmware build compiles with -std=c99 (rtconfig.py).
 * Keep the contract usable there with a portable compile-time assertion, and
 * rely on tests/abi for the exhaustive size/alignment/offset
 * checks.  A negative array bound is rejected by every conforming compiler. */
#  define EXTERN_C_BEGIN
#  define EXTERN_C_END
#  define CONTRACT_SA_CAT_(a, b) a##b
#  define CONTRACT_SA_CAT(a, b) CONTRACT_SA_CAT_(a, b)
#  if defined(__COUNTER__)
#    define CONTRACT_STATIC_ASSERT(cond, msg) \
       typedef char CONTRACT_SA_CAT(contract_static_assert_, __COUNTER__)[(cond) ? 1 : -1]
#  else
#    define CONTRACT_STATIC_ASSERT(cond, msg) \
       typedef char CONTRACT_SA_CAT(contract_static_assert_line_, __LINE__)[(cond) ? 1 : -1]
#  endif
#  if defined(__GNUC__)
#    define CONTRACT_ALIGNOF(type) __alignof__(type)
#  else
#    define CONTRACT_ALIGNOF(type) 0 /* only the ABI probe needs alignof; see above */
#  endif
#endif

/* All public headers are self-contained and only require <stdint.h> and
 * <stddef.h>; they must never rely on HAL, RT-Thread or vendor headers. */
#include <stdint.h>

#endif /* CONTRACT_VERSION_H */
