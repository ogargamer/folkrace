---
trigger: always_on
description: Chat formatting rules: strictly ban LaTeX and dollar sign math delimiters in chat text.
---

# Chat Formatting Rules

## Ban on LaTeX Math Delimiters in Chat Responses

1. **CHAT TEXT ONLY**:
   - In all conversational text, explanations, and markdown lists, NEVER use dollar signs (`$`) for LaTeX math delimiters (no `$...$` or `$$...$$`).
   - The chat UI cannot parse LaTeX and displays it as broken raw code.
   - NEVER use LaTeX commands: no `\rightarrow`, `\approx`, `\text{...}`, `\le`, `\ge`, `\times`, `\mathcal{...}`.
   - Use plain text equivalents:
     - Use `->` (NEVER `$\rightarrow$`).
     - Use `~` or `approx` (NEVER `\approx`).
     - Use `1.0 ms`, `2.5 m/s` (NEVER `$1.0\text{ ms}$`).
     - Use `O(N)` (NEVER `$\mathcal{O}(N)$`).
     - Use `<=`, `>=` (NEVER `\le`, `\ge`).

2. **CODE EXCEPTION**:
   - In programming code blocks and shell commands, dollar signs ARE permitted when required by the language syntax (e.g. PowerShell `$env:PATH`, bash `$VAR`, regex `^pattern$`).
