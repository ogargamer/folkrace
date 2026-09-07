---
trigger: always_on
description: Mandatory rule strictly banning dollar signs ($) and LaTeX notation across all output.
---

# Output & Formatting Rules

## Absolute Zero Dollar Sign Policy (STRICTLY ENFORCED)

- **NEVER use the dollar sign character (`$`) anywhere in responses.**
- The chat markdown client does not support LaTeX rendering.

### Formatting Guide:
- **Numbers, Ranges & Units**:
  - Write: `30 to 50 ms`, `3.5 to 4.0 m/s`, `100%`, `15 cm`, `0.05 to 0.10 s`, `1000 points`, `0.25 m`, `2.4 m/s^2`, `5.04 s`.
  - Do NOT wrap numbers or units in dollar signs.
- **Big-O & Math**:
  - Write: `O(N)`, `sqrt(mu * g / kappa)`, `L_lookahead = 0.25 m`, `2 * Wheelbase`.
  - Do NOT write: `\mathcal{O}(N)`, `\text{...}`, `\sqrt{...}`, `\approx`.
- **Symbols**:
  - Use plain text: `<=`, `>=`, `~`, `x`, `->`, `mu`, `kappa`.
