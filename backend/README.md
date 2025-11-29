# Backend Development Guide

**Workflow**: SpecKit Plus (`/sp.plan` → `/sp.implement`)

---

## Quick Start

### Step 1: Read Constitution
Read `backend/constitution.md` - yeh backend development ke principles hain.

### Step 2: Plan (SpecKit Plus)
1. Copy prompt from `backend/PLAN-PROMPTS.md`
2. Run `/sp.plan` in Claude Code
3. Paste prompt when asked
4. Get detailed implementation plan

### Step 3: Implement (SpecKit Plus)
1. Copy phase prompt from `backend/IMPLEMENT-PROMPTS.md`
2. Run `/sp.implement` in Claude Code
3. Paste prompt when asked
4. Claude will implement that phase

### Step 4: Repeat
Move to next phase, repeat Step 3

---

## Files Structure

```
backend/
├── constitution.md          # Backend principles (READ FIRST)
├── PLAN-PROMPTS.md          # Plan prompts (copy-paste in Claude)
├── IMPLEMENT-PROMPTS.md     # Implement prompts (phase by phase)
└── README.md               # This file
```

---

## Features

1. **Feature 1**: RAG Chatbot (Base - 100 points)
2. **Feature 3**: Better Auth + User Background (Bonus - 50 points)
3. **Feature 4**: Content Personalization (Bonus - 50 points)
4. **Feature 5**: Urdu Translation (Bonus - 50 points)

**Note**: Feature 2 (Subagents) already complete ✅

---

## Workflow Example

```
1. Read: backend/constitution.md
2. Plan: /sp.plan → Copy from PLAN-PROMPTS.md → Paste prompt
3. Implement Phase 1: /sp.implement → Copy from IMPLEMENT-PROMPTS.md → Paste prompt
4. Test: Review code, test functionality
5. Implement Phase 2: /sp.implement → Copy next phase → Paste prompt
6. Repeat until all phases complete
```

---

## SpecKit Plus Commands

- **Plan**: `/sp.plan` - Generate implementation plan
- **Implement**: `/sp.implement` - Implement a phase

**Note**: We skip `/sp.specify` and `/sp.clarify` to save tokens. Constitution already defines principles.

---

## Benefits

✅ **Token Saving**: Skip specify/clarify, only plan/implement  
✅ **SpecKit Plus**: Uses proper workflow with history tracking  
✅ **Organized**: Phase-by-phase implementation  
✅ **Constitution-Based**: All code follows backend principles

---

**Start with**: `backend/constitution.md` → `backend/PLAN-PROMPTS.md` → `backend/IMPLEMENT-PROMPTS.md`

