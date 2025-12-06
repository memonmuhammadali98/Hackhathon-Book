---
description: "Task list for Physical AI & Humanoid Robotics Textbook implementation"
---

# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-physical-ai-robotics-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Not explicitly requested, so no test tasks will be generated.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Initialize Docusaurus v3.0.0+ project in physical-ai-book
- [X] T002 Configure Docusaurus presets, navbar, footer in docusaurus.config.js
- [X] T003 Create Docusaurus folder structure (/docs, /docs/intro, /docs/chapters, etc.)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Create `/rag-backend` folder
- [X] T005 Create Qdrant collection "physical_ai_book"
- [X] T006 Set Qdrant vector size (1536) and add metadata (page_number, chapter, heading, text, chunk_id)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Learning Physical AI Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Understand the core concepts of Physical AI and embodied intelligence.

**Independent Test**: Review comprehension of chapters 1-3 (e.g., quizzes, concept summaries).

### Implementation for User Story 1

- [X] T007 [US1] Generate introductory chapters (1-3) on Physical AI fundamentals, embodiment, affordances, reactive vs. deliberative control in /docs/chapters/

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Practical Simulation and Control (Priority: P2)

**Goal**: Learn how to set up and control humanoid robots in simulation environments.

**Independent Test**: Successfully run provided code examples for robot control in each simulation environment.

### Implementation for User Story 2

- [X] T008 [US2] Generate chapters on ROS 2, Gazebo, Unity, and NVIDIA Isaac simulation environments in /docs/chapters/
- [X] T009 [US2] Create the capstone humanoid robot project chapter in /docs/chapters/capstone.md

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Advanced Navigation and Perception (Priority: P3)

**Goal**: Understand and implement advanced capabilities like Visual SLAM (VSLAM) and Navigation2 (Nav2).

**Independent Test**: Deploy VSLAM and Nav2 algorithms in a simulated environment and observe the robot's autonomous navigation.

### Implementation for User Story 3

- [X] T010 [US3] Generate chapters on Visual SLAM (VSLAM) and Navigation2 (Nav2) in /docs/chapters/

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: RAG Chatbot Implementation

**Purpose**: Build and integrate the RAG chatbot functionality.

- [X] T011 Implement `/embed`, `/query`, `/upsert` endpoints in `/rag-backend/main.py`
- [X] T012 Integrate OpenAI Embedding model (text-embedding-3-large or mini) and Qdrant Cloud Free tier in `/rag-backend/main.py`
- [X] T013 Create document chunker (markdown → chunks) in `/rag-backend/utils.py`
- [X] T014 Add CORS + rate limit to FastAPI backend in `/rag-backend/main.py`
- [X] T015 Create ingestion script `/scripts/ingest.py` for document pipeline (walk through `/docs`, convert MD to clean text, chunk into 512–1024 token segments, embed & upsert to Qdrant)
- [X] T016 Add floating chatbot UI inside Docusaurus book using OpenAI ChatKit SDK in `/src/components/Chatbot.jsx`
- [X] T017 Implement "Ask based on whole book" feature in `/src/components/Chatbot.jsx`
- [X] T018 Implement "Ask based on selected text" feature in `/src/components/Chatbot.jsx`
- [X] T019 Implement source citations in `/src/components/Chatbot.jsx`
- [X] T020 Implement highlighting relevant paragraphs in `/src/components/Chatbot.jsx`

---

## Phase 7: Advanced Features (Bonus Marks)

**Purpose**: Implement advanced features for enhanced functionality.

- [ ] T021 Create Claude Subagents definitions (`/agents/*.yaml`) for ResearchAgent, ChapterWriterAgent, CodeAgent, QAAgent
- [ ] T022 Implement Skills (`/skills/*.md` or `.yaml`) for WriteROS2LaunchFile, GenerateNav2Config, ExplainVSLAMPipeline, UnityArticulationBodyCreator
- [ ] T023 Implement GitHub Action (`.github/workflows/book-regenerate.yml`) to regenerate chapters on `/specs` changes
- [ ] T024 Implement GitHub Action for auto-deploy to GitHub Pages in `.github/workflows/book-regenerate.yml`
- [ ] T025 Add personalization button + Urdu translation button to each chapter in Docusaurus components
- [ ] T026 Implement Better-Auth signup/signin with user background questions

---

## Phase 8: Deployment Plan

**Purpose**: Deploy the book and RAG backend.

- [X] T027 Configure `docusaurus.config.js` with `url` and `baseUrl` for GitHub Pages deployment
- [ ] T028 Deploy FastAPI backend to Render or Vercel (manual step - requires user credentials)
- [X] T029 Point ChatKit UI to hosted API in `/src/components/Chatbot.jsx`
- [ ] T030 Create full demo video for YouTube (manual step - requires user action)

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements that affect multiple user stories.

- [X] T031 Auto-generate Preface in `/docs/preface.md`
- [X] T032 Auto-generate Glossary (AI, robotics, sensor, VSLAM terms) in `/docs/glossary.md`
- [ ] T033 Auto-generate Index / tag pages in `/docs/index.md` (optional - Docusaurus auto-generates)
- [X] T034 Ensure MDX/Markdown is Docusaurus-compatible (Title, Section, Subsection)
- [ ] T035 Run `npm run build` for Docusaurus book (manual step - requires user to test locally)
- [ ] T036 Run `npm run deploy` for Docusaurus book (manual step - requires user credentials)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3, 4, 5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **RAG Chatbot Implementation (Phase 6)**: Depends on Foundational phase completion and content generation from User Stories.
- **Advanced Features (Phase 7)**: Depends on Foundational phase completion and content generation from User Stories.
- **Deployment Plan (Phase 8)**: Depends on all implementation phases being complete.
- **Polish (Phase 9)**: Depends on all desired user stories and features being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each Task Group

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Tasks within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Example: There are no explicit parallel tasks in this simplified User Story 1.
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
