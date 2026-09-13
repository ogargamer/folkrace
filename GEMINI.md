# Workspace Rules for Da Winna

## Text & Math Formatting Guidelines (ABSOLUTE & UNCONDITIONAL BAN ON DOLLAR SIGNS)

- **CRITICAL ZERO-DOLLAR-SIGN RULE**: NEVER emit the dollar sign character (`$`) anywhere in the output.
  - Do NOT use single `$...$` or double `$$...$$` anywhere.
  - Do NOT use `\text{...}`, `\mathcal{...}`, `\times`, `\le`, `\ge`, `\approx`, `\mu`, or any LaTeX syntax.
  - The chat markdown parser cannot render LaTeX and displays it as ugly raw unparsed code.

### Positive & Negative Examples:
- **Quantities & Ranges**:
  - ALWAYS write: `30 to 50 ms`, `3.5 to 4.0 m/s`, `100%`, `15 cm`, `0.05 to 0.10 s`, `1000 points`, `0.25 m`, `2.4 m/s^2`.
  - NEVER write: dollar signs around numbers, units, ranges, or percentages.
- **Big-O & Complexity**:
  - ALWAYS write: `O(N)`, `O(1)`, `O(N log N)`.
  - NEVER write: `$\mathcal{O}(N)$`.
- **Formulas & Variables**:
  - ALWAYS write: `v = sqrt(mu * g / kappa)`, `L_lookahead = 0.25 m`, `2 * Wheelbase`, `W_curvature`, `alpha(s)`, `kappa_c`, `ds`.
  - NEVER write: `$\sqrt{...}$`, `$W_{\kappa}$`, `$v = \sqrt{\mu g R}$`.
- **Symbols**:
  - Use clean text: `<=`, `>=`, `~`, `x`, `->`, `mu`, `kappa`.

## File Hygiene & Organization
- Scratch/temporary scripts belong ONLY in `scratch/`. Delete one-off test scripts after running.
- NEVER create `*_v2`, `*_old`, `*_test2`, `*.bak`, or `*.dll.old`. Edit in place; use git for history.
- NEVER dump raw datasets, CSVs, or `.npz` into source folders. Use `data/` or `scratch/`.
- `launchers/`: Single source of truth for runnable scripts/GUIs. Do not scatter runners elsewhere.
- `controllers/`: Real Webots controllers only, 1:1 matching world files. No library/helper folders.
- `worlds/`: Only active worlds matching controllers. No orphan `.wbproj` or scratch worlds.
- Hardware: Firmware and test benches strictly belong in `esp32/`, `stm32/`, or `raspberry_pi/`.

