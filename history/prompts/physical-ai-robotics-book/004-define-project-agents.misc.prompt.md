---
id: 004
title: Define project agents
stage: misc
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20251101
feature: physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: unknown
command: sp.agents /sp.agents create
labels: [agents, definition]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - agents/sp.agents.yaml
tests:
null
---

## Prompt

sp.agents /sp.agents create
name: sp.agents
description: >
  Subagents for the Physical AI & Humanoid Robotics book project.
  Each agent automates tasks using skills for code generation, simulation, personalization, and Urdu translation.

agents:
  - name: CodeAgent
    description: >
      Generates ROS2 launch files, Nav2 configs, Gazebo simulations,
      and Unity humanoid articulation bodies for the modules.
    uses_skills:
      - WriteROS2LaunchFile
      - GenerateNav2Config
      - SimulateHumanoidGazebo
      - ExplainVSLAMPipeline
      - UnityArticulationBodyCreator

  - name: PersonalizationAgent
    description: >
      Personalizes chapter content for logged-in users
      and translates chapters to Urdu.
    uses_skills:
      - PersonalizeChapter
      - TranslateToUrdu

  - name: ChapterGenerationAgent
    description: >
      Combined agent for full chapter generation:
      - Generates module content with code and diagrams
      - Applies personalization
      - Translates chapters to Urdu
    uses_agents:
      - CodeAgent
      - PersonalizationAgent

## Response snapshot

Agent definitions written to agents/sp.agents.yaml and agents directory created.

## Outcome

- ✅ Impact: Project agents defined and stored.
- 🧪 Tests: N/A
- 📁 Files: agents/sp.agents.yaml created.
- 🔁 Next prompts: Use defined agents for future tasks.
- 🧠 Reflection: Successfully handled the /sp.agents create command by creating the agents YAML file and directory.
