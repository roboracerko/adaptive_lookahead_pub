# Method Notes

This workspace includes five lookahead-assignment variants.

## `kw_sim`

- file: `src/pp_adaptive/pp_adaptive/compute_adaptive_lookahead_kw2.py`
- idea: use a short forward curvature window to screen candidate lookaheads, then retain candidates that satisfy a simulated lateral-acceleration constraint

## `alg1_org`

- file: `src/pp_adaptive/pp_adaptive/compute_adaptive_lookahead_org.py`
- idea: reproduce a prior adaptive-lookahead assignment procedure with its original candidate set

## `alg1_ext`

- file: `src/pp_adaptive/pp_adaptive/compute_adaptive_lookahead_org.py`
- idea: reuse the same scoring structure as `alg1_org` with a larger lookahead candidate set

## `kv`

- file: `src/pp_adaptive/pp_adaptive/compute_adaptive_lookahead_kv.py`
- idea: set lookahead proportional to speed

## `curv`

- file: `src/pp_adaptive/pp_adaptive/compute_adaptive_lookahead_curv.py`
- idea: set lookahead as an inverse function of curvature magnitude
