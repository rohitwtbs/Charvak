# Charvak

![Your Mascot](charvak.png)  // Replace with the actual path and filename





Charvak , the gaming engine. Open source engine  written to make AAA games in Cython.
Long term goal - Write in game logic in Python , and build for all platforms (VR , Windows, linux , mac , android , ios , web ,etc..)




 ┌────────────────────────────────────────────────────────────────────┐
 │                          Quest 3 Hardware                          │
 └────────────────────────────────────────────────────────────────────┘
                  │
                  ▼
 ┌────────────────────────────────────────────────────────────────────┐
 │                       Android Runtime (NDK)                        │
 │    - ARM64 Linux Kernel                                           │
 │    - Android NDK Libraries (libc, libm, etc.)                     │
 │    - OpenXR Mobile Loader (via Oculus/Meta OpenXR SDK)            │
 └────────────────────────────────────────────────────────────────────┘
                  │
                  ▼
 ┌────────────────────────────────────────────────────────────────────┐
 │                        Cython Core Engine                          │
 │ ┌──────────────────────────────┐  ┌──────────────────────────────┐ │
 │ │  OpenXR Manager (VR API)     │  │  Vulkan Renderer (Graphics)  │ │
 │ │  - Session management        │  │  - Swapchain handling        │ │
 │ │  - Headset tracking          │  │  - Shader pipeline           │ │
 │ │  - Controller input          │  │  - Frame synchronization      │ │
 │ └──────────────────────────────┘  └──────────────────────────────┘ │
 │ ┌──────────────────────────────┐  ┌──────────────────────────────┐ │
 │ │  Audio Engine               │  │  Physics Engine (Optional)   │ │
 │ │  - Spatial audio            │  │  - Simple collision system   │ │
 │ │  - OpenAL / OVR Audio       │  │  - Rigid body dynamics       │ │
 │ └──────────────────────────────┘  └──────────────────────────────┘ │
 └────────────────────────────────────────────────────────────────────┘
                  │
                  ▼
 ┌────────────────────────────────────────────────────────────────────┐
 │                        Embedded Python VM                          │
 │ - Game logic scripts (Python)                                      │
 │ - Entity/component systems                                         │
 │ - Event handling                                                   │
 │ - Scripting APIs exposed from Cython engine                       │
 │   (e.g., `get_controller_pose()`, `spawn_entity()`, etc.)         │
 └────────────────────────────────────────────────────────────────────┘
                  │
                  ▼
 ┌────────────────────────────────────────────────────────────────────┐
 │                     Game/Application Layer                         │
 │ - Scenes, objects, behaviors                                       │
 │ - User interface                                                   │
 │ - Game state and logic                                             │
 └────────────────────────────────────────────────────────────────────┘
