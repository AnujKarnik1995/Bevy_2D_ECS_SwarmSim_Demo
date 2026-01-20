# ECS Warehouse Swarm Simulation


**Language:** Rust<br>
**Engine:** Bevy (v0.18.0)<br>
**Platform:** Ubuntu 24.04 (WSL 2)

A data-driven warehouse simulation demonstrating **Entity-Component-System (ECS)** architecture, **Finite State Machines (FSM)**, and **Swarm Intelligence**.

This project simulates a fleet of autonomous robots that manage tasks (Pickups, Dropoffs, Charging) while negotiating resource contention and avoiding collisions without a central "Hive Mind" controller.

It implements a primitive collision avoidance system, a primitive tasking/reservation system and a primitive state machine.

Demo videos:

https://github.com/user-attachments/assets/0de67407-e0a7-4233-8f69-6ab00b4d7428

This project was inspired from a similar project I have been building using C++ and the EnTT framework: https://github.com/AnujKarnik1995/EnTT_ECS_2D_SwarmSim

## Key Features

* **Pure ECS Architecture:** Logic is strictly separated from data. Robots are composed of atomic components (`Position`, `Battery`, `State`), allowing systems to run in parallel.
* **Swarm Collision Avoidance:** Uses a distributed "Boids" model where robots calculate separation forces locally. Deadlocks are resolved deterministically using unique Entity IDs.
* **Resource Locking:** Implements a reservation system ("The Dispatcher") ensuring robots do not swarm a single optimal station.
* **Finite State Machine (FSM):** Exhaustive state handling (`Idle` -> `Moving` -> `Working` -> `Charging`) ensures robust behavior and prevents undefined states.
* **Battery Drain Simulation:** Battery drain is calculated based on current robot actions and activities. Can be tuned to match real-world scenarios.
* **Visual Debugging:**
    * **Battery System:** Robots change color based on charge levels and turn black upon battery depletion.
    * **Real-time Tuning:** Configuration (speed, robot count, battery drain) is loaded from `assets/simulation.ron` to allow iteration without recompilation.

## Installation & Prerequisites

This project is built with Rust. Ensure you have the Rust toolchain installed.

### 1. Install Rust
If you do not have Rust installed, run the following commands:
```bash
curl --proto '=https' --tlsv1.2 -sSf [https://sh.rustup.rs](https://sh.rustup.rs) | sh
source $HOME/.cargo/env
```
### 2. Install 
Bevy requires specific system libraries for audio and window management. Run the following command on Ubuntu 24.04:
```
sudo apt-get update
sudo apt-get install g++ pkg-config libx11-dev libasound2-dev libudev-dev libwayland-dev libxkbcommon-dev
```
### 3. How to run
Extract the project
```
unzip ecs_warehouse_sim.zip
cd ecs_warehouse_sim
```
Run the simulation
```
cargo run
```
