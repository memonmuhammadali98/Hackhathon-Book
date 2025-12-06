# SP.PLAN — Physical AI & Humanoid Robotics Textbook
# Feature: physical-ai-robotics-book
# Purpose: Provide a complete development plan for generating the Docusaurus book + RAG chatbot

============================================================
1. BOOK GENERATION WORK PLAN
============================================================

1.1 Generate Book Skeleton (Docusaurus 3)
-----------------------------------------
Tasks:
- Initialize a Docusaurus v3.0.0+ project:
  npx create-docusaurus@latest physical-ai-book classic
- Configure presets, navbar, footer
- Create folder structure:
  /docs
  /docs/intro
  /docs/chapters
  /docs/humanoid-control
  /docs/isaac
  /docs/ros2
  /docs/nav2
  /docs/vslam
  /docs/simulations
  /docs/capstone

Deliverables:
- docusaurus.config.js fully configured
- Sidebars.js auto-generated based on directory structure

1.2 Generate Markdown Chapters using the Constitution
-----------------------------------------------------
Tasks:
- Use the constitution + sp.goal + sp.spec to generate:
  ✔ 12–15 chapters
  ✔ text-based diagrams
  ✔ code blocks (Python, ROS2 launch files, Unity C#, Nav2 params)
  ✔ Q&A RAG micro-sections inside each chapter
- Ensure MDX/Markdown is Docusaurus-compatible:
  # Title
  ## Section
  ### Subsection

Deliverables:
- All chapters placed under `/docs/chapters/<chapterX>.md`

1.3 Add Summary, Glossary, and Index
------------------------------------
Tasks:
- Auto-generate:
  ✔ Preface
  ✔ Glossary (AI, robotics, sensor, VSLAM terms)
  ✔ Index / tag pages

Deliverables:
- `/docs/preface.md`
- `/docs/glossary.md`
- `/docs/index.md`


============================================================
2. RAG CHATBOT DEVELOPMENT PLAN
============================================================

2.1 Backend (FastAPI)
---------------------
Tasks:
- Create `/rag-backend` folder
- Implement `/embed`, `/query`, `/upsert` endpoints
- Integrate:
  ✔ OpenAI Embedding model (text-embedding-3-large or mini)
  ✔ Qdrant Cloud Free tier
- Create document chunker (markdown → chunks)
- Add CORS + rate limit

Deliverables:
- FastAPI server running on Vercel/Render/Local

2.2 Qdrant Vector DB Setup
--------------------------
Tasks:
- Create collection: "physical_ai_book"
- Set vector size (1536)
- Add metadata:
  page_number
  chapter
  heading
  text
  chunk_id

Deliverables:
- Qdrant Cloud collection populated

2.3 Document Pipeline
---------------------
Tasks:
- Script: `/scripts/ingest.py`
- Steps:
  - Walk through `/docs`
  - Convert MD to clean text
  - Chunk into 512–1024 token segments
  - Embed & upsert to Qdrant

Deliverables:
- Automatic ingestion system

2.4 Client-side Chatbot UI
--------------------------
Tasks:
- Use OpenAI ChatKit SDK
- Floating chatbot UI inside Docusaurus book
- Add:
  ✔ Ask based on whole book
  ✔ Ask based on “selected text”
  ✔ Source citations
  ✔ Highlight relevant paragraphs

Deliverables:
- `/src/components/Chatbot.jsx`


============================================================
3. ADVANCED FEATURES (Bonus Marks)
============================================================

3.1 Claude Subagents
--------------------
Create:
- “ResearchAgent” → extracts structure
- “ChapterWriterAgent” → writes chapters
- “CodeAgent” → generates ROS2/Gazebo/Unity samples
- “QAAgent” → creates question lists

Deliverables:
- `/agents/*.yaml` definitions

3.2 Skills (Matrix-style reusable intelligence)
-----------------------------------------------
Skills to implement:
- “WriteROS2LaunchFile”
- “GenerateNav2Config”
- “ExplainVSLAMPipeline”
- “UnityArticulationBodyCreator”

Deliverables:
- `/skills/*.md` or `.yaml` reusable skills

3.3 Automatically Regenerate Chapters on Update
-----------------------------------------------
Tasks:
- GitHub Action:
  - When `/specs` changes → regenerate chapters
  - Auto-deploy to GitHub Pages

Deliverables:
- `.github/workflows/book-regenerate.yml`


============================================================
4. DEPLOYMENT PLAN
============================================================

4.1 Host Book on GitHub Pages
-----------------------------
Tasks:
- Configure `docusaurus.config.js → url + baseUrl`
- Run:
  npm run build
  npm run deploy

Deliverables:
- Live book URL

4.2 Host RAG Backend
--------------------
Tasks:
- Deploy FastAPI to Render or Vercel
- Point ChatKit UI to hosted API

Deliverables:
- Live API endpoint

4.3 Full Demo Video (YouTube)
-----------------------------
Checklist:
- Show book
- Show chapters
- Show chatbot answering questions
- Show selected-text mode
- Show Qdrant dashboard
- Show GitHub Pages deployment

Deliverables:
- YouTube video link



============================================================
5. KEY DECISIONS AND RATIONALE
============================================================
(To be filled: Options considered, trade-offs, rationale for major architectural decisions - e.g., choice of Docusaurus, FastAPI, Qdrant, embedding model.)

============================================================
6. INTERFACES AND API CONTRACTS
============================================================
(To be filled: For RAG Backend API endpoints (/embed, /query, /upsert), detail inputs, outputs, and explicit error taxonomy with status codes. Versioning strategy.)

============================================================
7. NON-FUNCTIONAL REQUIREMENTS (NFRs) AND BUDGETS
============================================================
(To be filled: Performance (p95 latency, throughput, resource caps), Reliability (SLOs, error budgets, degradation strategy), Security (AuthN/AuthZ, data handling, secrets, auditing), Cost (unit economics beyond Qdrant free tier).)

============================================================
8. DATA MANAGEMENT AND MIGRATION
============================================================
(To be filled: Source of truth, schema evolution for Qdrant, migration/rollback strategies, data retention policies.)

============================================================
9. OPERATIONAL READINESS
============================================================
(To be filled: Observability (logs, metrics, traces), Alerting (thresholds, on-call owners), Runbooks, Deployment and Rollback strategies, Feature Flags.)

============================================================
10. RISK ANALYSIS AND MITIGATION
============================================================
(To be filled: Identify top 3 risks, blast radius, mitigation strategies, kill switches/guardrails.)

============================================================
11. ARCHITECTURAL DECISION RECORDS (ADR)
============================================================
(To be filled: Link to any significant ADRs created during planning and implementation.)

============================================================
6. ACCEPTANCE CRITERIA
============================================================

- Book builds successfully (npm run build)
- Book is live on GitHub Pages or Vercel
- All chapters follow Constitution rules
- RAG chatbot works
- Selected-text answering works
- Qdrant DB populated
- Code examples are runnable
- Demo video provided

============================================================

END OF SP.PLAN