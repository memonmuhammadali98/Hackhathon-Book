---
id: 002
title: Define project skills
stage: misc
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20251101
feature: physical-ai-robotics-book
branch: 001-physical-ai-robotics-book
user: unknown
command: sp.skills /sp.skills create
labels: [skills, definition]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - skills/sp.skills.yaml
tests:
null
---

## Prompt

sp.skills /sp.skills create
name: sp.skills
description: >
  Reusable intelligence modules (skills) for the Physical AI & Humanoid Robotics book.
  Includes code generation, simulation setup, personalization, and Urdu translation.
skills:
  - name: WriteROS2LaunchFile
    description: Generates a ROS2 launch file from node and parameter configurations
    input:
      nodes: list
      parameters: dict
    output: YAML launch file
    examples:
      - nodes: ["robot_state_publisher", "controller_manager"]
        parameters: {"robot_state_publisher": {"use_sim_time": true}}

  - name: GenerateNav2Config
    description: Creates Nav2 navigation stack configuration for a robot
    input:
      robot_name: string
      map_file: path
    output: YAML configuration file

  - name: SimulateHumanoidGazebo
    description: Generates Gazebo simulation environment and robot description
    input:
      urdf_file: path
      sensors: list
    output: Gazebo world and robot simulation setup

  - name: ExplainVSLAMPipeline
    description: Explains and generates code for VSLAM pipeline integration in ROS2
    input:
      camera_config: dict
      algorithm: string
    output: Python / ROS2 integration code

  - name: UnityArticulationBodyCreator
    description: Creates Unity humanoid articulation bodies and joints for simulation
    input:
      humanoid_model: file
      joint_parameters: dict
    output: Unity prefab and C# scripts

  - name: PersonalizeChapter
    description: Adjust chapter content based on user software/hardware background
    input:
      user_profile: dict
      chapter_content: string
    output: personalized chapter content
    examples:
      - user_profile: {"python": "intermediate", "ros2": "beginner"}
        chapter_content: "<raw chapter text>"
        output: "<personalized chapter text>"

  - name: TranslateToUrdu
    description: Translate chapter content to Urdu
    input:
      text: string
    output: Urdu-translated text
    examples:
      - text: "This is a ROS2 tutorial"
        output: "یہ ROS2 ٹیوٹوریل ہے"

## Response snapshot

Skill definitions written to skills/sp.skills.yaml and skills directory created.

## Outcome

- ✅ Impact: Project skills defined and stored.
- 🧪 Tests: N/A
- 📁 Files: skills/sp.skills.yaml created.
- 🔁 Next prompts: Use defined skills for future tasks.
- 🧠 Reflection: Successfully defined skills as per the plan.
