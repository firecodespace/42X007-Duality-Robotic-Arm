# LLM Integration – Gemini 2.5 / 3.0

## Responsibilities
- Convert natural language tasks into structured actions.
- Follow strict JSON schema.
- Avoid hallucination.
- Support fallback plans.

---

## Pipeline
1. Build prompt with:
   - system role
   - tools list
   - schemas
   - instructions
2. Send to Gemini via API.
3. Parse and validate output.
4. Forward to executor.

---

## Version Comparison (Bonus)
- Gemini 3.0 → more accurate, fewer steps.
- Gemini 2.5 → sometimes verbose but stable.

Record differences in `experiments.md`.
