[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/Nr1414-dev/sokoban-solver-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/Nr1414-dev/sokoban-solver-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

# Sokoban Solver — make.fun

Entry for the [Build the best Sokoban solver, 21 November 2026](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty on [make.fun](https://make.fun/bounty/sokoban-solver-01-run-20261121).

`main.ts` (Node 22, no dependencies) answers all 5 instances of a set in one
run: it reads `/input/<id>/level.txt` (XSB) and writes
`/output/<id>/solution.txt` (moves in `u/d/l/r`).

## Approach

A generated level is a hallway with one to four rooms off it, each room
holding 3 or 4 boxes and as many goals, and the level's push-optimal count P\*
is the sum of the rooms' optima. So the solver:

1. **Decomposes** the level into rooms by peeling articulation points off the
   floor graph (smallest equal-boxes/goals component first).
2. **Solves each room push-optimally** with A\* over pushes:
   - Hungarian-assignment heuristic on wall-only shortest-path distances
     (admissible and consistent),
   - a deadlock table from reverse pull search,
   - 2×2 freeze deadlock pruning,
   - a transposition table keyed on (sorted boxes, canonical player cell),
     where the canonical player cell is the smallest cell of the player's
     reachable region.
3. **Stitches** the room solutions with player walks along the hallway.
4. Falls back to a whole-level push A\* if decomposition ever fails, and
   verifies every answer by replaying it on the level before writing it.

Everything is deterministic: fixed direction order, fixed heap tie-breaks,
expansion-count budgets only — the wall clock is never read, so two runs on
the same input write byte-identical outputs.

## Results (practice set, kit scorer)

| instance | pushes | P\* | points |
|----------|--------|-----|--------|
| i001 | 13 | 13 | 1000 |
| i002 | 16 | 16 | 1000 |
| i003 | 29 | 29 | 1000 |
| i004 | 47 | 47 | 1000 |
| i005 | 60 | 60 | 1000 |

**Score 1,000,000 / 1,000,000** (all five at P\*), in under 1 second locally.
The house baseline (plain-astar 1.0.0) scores 200,000 on the same set. The
solver also scores 1,000,000 on three further sets drawn from the kit's
generator with different seeds.

## Run locally

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

## License

MIT — see [LICENSE](LICENSE).
