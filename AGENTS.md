# AGENTS.md

## Comments Policy

- Keep comments short: max two lines.
- Write comments in English only.
- Add a comment ONLY if the code cannot be understood without it.
- Do NOT restate values, names, or constants already defined in the documented entity.
  - Bad: `// timeout = 30` above `const TIMEOUT = 30`.
  - Reason: when the value changes, the comment goes stale and misleads.
- Never duplicate identifiers, default values, ranges, or type info already visible in the signature or declaration.
- Prefer self-documenting code (clear names, small functions) over explanatory comments.
- If a comment is needed, explain WHY, not WHAT.

## Editing Existing Comments

- If you touch or modify a comment, translate it to English at the same time.
- Do not leave partially translated or mixed-language comments behind.
- If the comment becomes unnecessary after refactoring, delete it instead of rewriting.